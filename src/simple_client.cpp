#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "boost/thread.hpp"
//---mavros_msgs
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/PositionTarget.h>
//---
#include "utils.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//---



#include <mutex>          // std::mutex
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>


using namespace Eigen;
using namespace std;



class SIMPLE_CLIENT {


    public:
        SIMPLE_CLIENT();
        void position_controller();
		void run();
		void localization_cb ( geometry_msgs::PoseStampedConstPtr msg );
		void mavros_state_cb( mavros_msgs::State mstate);
		void takeoff( const double altitude );
		bool linear_motion( Vector3d dest, double yaw, double t_lock, bool heading );
		bool linear_motion( Vector3d dest, double t_lock );
		bool rotate( double angle, double t_lock );
		bool lock_rotation( double cmd, double t_lock);	
		bool yaw_reached (  double cmd, double mes  );	
		void select_action();
		void state_machine();
		void state_machine_2();
		void land();  
        void joy_cb( sensor_msgs::Joy j );
    
    private:

        ros::NodeHandle _nh;
        ros::Publisher _target_pub;
        ros::Subscriber _localization_sub;
        ros::Subscriber _joy_data_sub;
        ros::Subscriber _mavros_state_sub;
        bool _first_local_pos;

        // --- Desired state
        Vector3d _cmd_p;
        Vector3d _ref_p;
        Vector3d _ref_dp;
        double _cmd_yaw;
        int _rate;
        double _cruise_vel;
        // --- Drone state ---
        Vector3d _e_p;
        Vector3d _w_p;
        Vector3d _vel_joy;
        Vector4d _w_q;
        float _mes_yaw;
        Vector3d _w_lin_vel;
        Vector3d _w_ang_vel;
        double _yaw_motion_threshold;
        double _linear_motion_threshold;
        mavros_msgs::State _mstate;

        ros::ServiceClient _arming_client;
        ros::ServiceClient _set_mode_client;
        ros::ServiceClient _land_client;


};


SIMPLE_CLIENT::SIMPLE_CLIENT() {

    if( !_nh.getParam("rate", _rate)) {
        _rate = 100;
    }

    if(!_nh.getParam("linear_motion_threshold", _linear_motion_threshold)) {
        _linear_motion_threshold = 0.15;
    }

	_first_local_pos = false;
    _target_pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    _localization_sub = _nh.subscribe( "/mavros/vision_pose/pose", 1, &SIMPLE_CLIENT::localization_cb, this);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 1, &SIMPLE_CLIENT::mavros_state_cb, this);
    _joy_data_sub = _nh.subscribe("/joy", 1, &SIMPLE_CLIENT::joy_cb, this);

	// --- Services ---
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    //---

    _vel_joy << 0.0, 0.0, 0.0;


}



void SIMPLE_CLIENT::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;
}


void SIMPLE_CLIENT::localization_cb ( geometry_msgs::PoseStampedConstPtr msg ) {

    _w_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg->pose.orientation.w,  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z) ) );
    _mes_yaw = rpy(2);

    Quaternionf q;
    q = AngleAxisf(0.0, Vector3f::UnitX())
        * AngleAxisf(0.0, Vector3f::UnitY())
        * AngleAxisf(_mes_yaw, Vector3f::UnitZ());
    Vector4d w_q ( q.w(), q.x(), q.y(), q.z() );
    _w_q = w_q / w_q.norm() ;

   
    _first_local_pos = true;

    
}


void SIMPLE_CLIENT::takeoff( const double altitude ) {
  
  ros::Rate rate(10);
  
  //Set up control mode
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  //---

  if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    ROS_INFO("OFFBOARD mode enabled");
  }



  //---Arm
  if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
  }

  while(!_mstate.armed ) usleep(0.1*1e6);
  ROS_INFO("Vehicle armed");
  //---

  while(_mstate.mode != "OFFBOARD" ) usleep(0.1*1e6);
  ROS_INFO("Vehicle in offboard");

  _cmd_p(2) = altitude;
  
  while( fabs( _cmd_p(2) - _w_p(2)) > 0.1 ) {
    usleep(0.1*1e6);
  }

  ROS_INFO("Takeoff completed");
  
}



void SIMPLE_CLIENT::position_controller(){

    ros::Rate r(10);
    double ref_T = 1/10.0;

    double ref_omega0_xyz;
    if( !_nh.getParam("ref_omega0_xyz", ref_omega0_xyz)) {
        ref_omega0_xyz = 1.0;
    }
    double ref_zita;
    if( !_nh.getParam("ref_zita", ref_zita)) {
        ref_zita = 0.75;
    }
    double ref_jerk_max;
    if( !_nh.getParam("ref_jerk_max", ref_jerk_max)) {
        ref_jerk_max = 15.0;
    }
    double ref_acc_max;
    if( !_nh.getParam("ref_acc_max", ref_acc_max)) {
        ref_acc_max = 5.0;
    }
    double ref_vel_max;
    if( !_nh.getParam("ref_vel_max", ref_vel_max)) {
        ref_vel_max = 2.0;
    }
    if( !_nh.getParam("max_cruise_vel", _cruise_vel)) {
        _cruise_vel = 2.0;
    }

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::FORCE |
    mavros_msgs::PositionTarget::IGNORE_YAW |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // |

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    _cmd_p = _ref_p = _w_p;

    Vector3d ref_dp;
    Vector3d ref_ddp;
    _e_p = Vector3d(0.0, 0.0, 0.0);

    _cmd_yaw = _mes_yaw;

    while (ros::ok()) {


				//cout << "_mes_yaw: " << _mes_yaw << endl;
        if( _mstate.mode != "OFFBOARD" ) {
            _ref_p = _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
            ref_dp = Vector3d(0.0, 0.0, 0.0);
            ref_ddp = Vector3d(0.0, 0.0, 0.0);
        }
        else {
            //---Position
            Vector3d ddp = Vector3d(0.0, 0.0, 0.0);
            Vector3d dp  = Vector3d(0.0, 0.0, 0.0);
            
            //Errore di posizione
            _e_p = _cmd_p - _ref_p;    
            ddp(0) = ref_omega0_xyz * ref_omega0_xyz * _e_p(0) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(0);
            ddp(1) = ref_omega0_xyz * ref_omega0_xyz * _e_p(1) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(1);
            ddp(2) = ref_omega0_xyz * ref_omega0_xyz * _e_p(2) - 2.0 * ref_zita * ref_omega0_xyz * ref_dp(2);
            
            Vector3d jerk = (ddp - ref_ddp)/ref_T;
            double n_jerk = jerk.norm();
    
            if( n_jerk > ref_jerk_max) {
                jerk *= (ref_jerk_max/n_jerk);
            }
            
            ddp = ref_ddp + jerk*ref_T;

            double sa = 1.0;
            double n_acc = ddp.norm();
            if(n_acc > ref_acc_max) {
                sa = ref_acc_max/n_acc ;
            }
            ref_ddp = ddp * sa;
                        
            dp = ref_dp + ref_ddp * ref_T;
            double n_vel = dp.norm();

            if(n_vel > _cruise_vel ) {
                for(int i = 0; i<3; i++) {
                    if(ref_dp[i] * _e_p[i] > 0) {
                        ref_ddp[i] = 0.0;
                    }
                }
                dp = ref_dp + ref_ddp*ref_T;
                ref_dp = dp / dp.norm() * _cruise_vel;
            }
            else {
                //Aggiornamento velocita'
                ref_dp += ref_ddp * ref_T ;
            }
            //Aggiornamento posizione
            _ref_p  += ref_dp*ref_T;
            //----
        }

        //---Publish command
        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = _ref_p[0];
        ptarget.position.y = _ref_p[1];
        ptarget.position.z = _ref_p[2];
        ptarget.yaw = _cmd_yaw;
        _target_pub.publish( ptarget );
        //---

        r.sleep();
    }
}


void SIMPLE_CLIENT::land() {
  mavros_msgs::CommandTOL land_srv;
  _land_client.call( land_srv );  


  //wait for landing
  cout << "Waiting disarm" << endl;
  while( _mstate.armed ) usleep(0.1*1e6);
  cout << "Disarmed!" << endl;

}


void SIMPLE_CLIENT::joy_cb( sensor_msgs::Joy j ) {


    _vel_joy[0] = j.axes[1]*0.2;
    _vel_joy[1] = j.axes[0]*0.2;
    _vel_joy[2] = j.axes[3]*0.2;
    

}

bool SIMPLE_CLIENT::linear_motion( Vector3d dest, double t_lock ) {

		_cmd_p(0) = dest(0);
		_cmd_p(1) = dest(1);
		_cmd_p(2) = dest(2);

		ros::Rate r(10);

		if( t_lock < 0.0 ) {
		    return true;
		}
		else if( t_lock > 0.0 ) {
		    float t = 0.0;
		    float Ts = 1.0/10.0;

		    while( t<t_lock && ( dest - _w_p ).norm() > _linear_motion_threshold ) {
		        t+=Ts;
		        r.sleep();
		    }
		}
		else {
		    while( ( dest - _w_p ).norm()  > _linear_motion_threshold ) {
		        r.sleep();
		    }
		}


	return true;
} //Linear motion: first rotation, than navigation


void SIMPLE_CLIENT::state_machine() {

	/*
	string line = "";
	cout << "Type s to start" << endl;

	while ( line != "s" ) {
		getline( cin, line );
	}
	*/
	takeoff(3.0);

	cout << "GOto first wp!" << endl;
	linear_motion( Vector3d( -0.39, 2.61, 3.0 ), 0.0);
	_cruise_vel = 0.1;
	cout << "GOto second wp!" << endl;
	linear_motion( Vector3d( -0.39, 2.61, 1.5 ), 0.0);

}

void SIMPLE_CLIENT::select_action() {

  string line;

  while(ros::ok()) {
    
    cout << "--------------------" << endl;
    cout << "Insert new action: " << endl;
    cout << "1 - takeoff" << endl;
    cout << "2 - land" << endl;
    cout << "3 - machine 1" << endl;
    cout << "--------------------" << endl;

    getline(cin, line);

    if( line == "1" ) {
        takeoff(1.8);
    }
    
    else if( line == "2") {
      land();
    }
    else if( line == "3") {
        boost::thread state_machine_t( &SIMPLE_CLIENT::state_machine, this );
    }
  }
}

void SIMPLE_CLIENT::run(){
    boost::thread position_controller_t( &SIMPLE_CLIENT::position_controller, this);
    boost::thread select_action_t( &SIMPLE_CLIENT::select_action, this );
    ros::spin();
}





int main(int argc, char** argv ) {


    ros::init(argc, argv, "exploration");
    SIMPLE_CLIENT sc;
    
    sc.run();

    return 0;

}
