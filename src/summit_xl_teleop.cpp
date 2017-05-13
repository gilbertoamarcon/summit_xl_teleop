/*
 * Based on the node "summit_xl_pad" from Robotnik Automation, SLL
 * Gilberto Marcon dos Santos
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <robotnik_msgs/ptz.h>
#include <unistd.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/ptz.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS	16
#define DEFAULT_AXIS_LINEAR		1
#define DEFAULT_AXIS_PAN 		0
#define DEFAULT_AXIS_ANGULAR	2
#define DEFAULT_AXIS_TILT		3
#define DEFAULT_AXIS_ZOOM_WIDE	12
#define DEFAULT_AXIS_ZOOM_TELE	13
#define DEFAULT_SCALE_LINEAR	1.0
#define DEFAULT_SCALE_ANGULAR	2.0
#define DEFAULT_SCALE_TILT		1.0
#define DEFAULT_SCALE_ZOOM		1.0

//Used only with ps4
#define AXIS_PTZ_TILT_UP		0
#define AXIS_PTZ_TILT_DOWN		1
#define AXIS_PTZ_PAN_LEFT		2
#define AXIS_PTZ_PAN_RIGHT		3


class SummitXLPad{

public:

	SummitXLPad();
	void Update();

private:

	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;

	int linear_;
	int angular_;
	int pan_;
	int tilt_;
	int zoom_wide_;
	int zoom_tele_;

	double scale_linear_;
	double scale_angular_;
	double scale_pan_; 
	double scale_tilt_; 
	double scale_zoom_; 

	// It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
	ros::Publisher vel_pub_, ptz_pub_;

	// It will be suscribed to the joystick
	ros::Subscriber pad_sub_;

	// Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel_;

	// Name of the topic where it will be publishing the pant-tilt values	
	std::string cmd_topic_ptz_;

	// Velocity scalling factors
	double ptz_vel;
	double current_vel;

	// Pad type
	std::string pad_type_;

	// Number of the DEADMAN button
	int dead_man_button_;

	// Number of the PTZ button
	int button_update_ptz_;

	// Number of the button for resetting PTZ
	int button_ptz_reset_;

	// Number of the button for increase or decrease the speed max of the joystick	
	int button_vel_inc_;
	int button_vel_dec_;
	int button_ptz_vel_inc_;
	int button_ptz_vel_dec_;

	// Service to modify the digital outputs
	ros::ServiceClient set_digital_outputs_client_;  

	// Number of buttons of the joystick
	int num_of_buttons_;

	// Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];

	// Pointer to a vector for controlling the event when pushing directional arrows (UNDER AXES ON PX4!)
	bool bRegisteredDirectionalArrows[4];

	// DIAGNOSTICS

	// Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 

	// Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 

	// General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	

	// Diagnostics min freq
	double min_freq_command, min_freq_joy; // 

	// Diagnostics max freq
	double max_freq_command, max_freq_joy; //
 	
	// Flag to enable/disable the communication with the publishers topics
	bool bEnable;

};


SummitXLPad::SummitXLPad():
angular_(0),
linear_(1),
pan_(2),
tilt_(3),
zoom_wide_(12),
zoom_tele_(13)
{
	current_vel = 0.1;
	ptz_vel = 1.0;

	//JOYSTICK PAD TYPE
	nh_.param<std::string>("pad_type",pad_type_,"ps3");
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

	// Analog Axis
	nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
	nh_.param("axis_linear", linear_, DEFAULT_AXIS_LINEAR);
	nh_.param("axis_pan", pan_, DEFAULT_AXIS_PAN);
	nh_.param("axis_tilt", tilt_, DEFAULT_AXIS_TILT);
	nh_.param("axis_zoom_wide", zoom_wide_, DEFAULT_AXIS_ZOOM_WIDE);
	nh_.param("axis_zoom_tele", zoom_tele_, DEFAULT_AXIS_ZOOM_TELE);

	// Axis Scaling
	nh_.param("scale_angular", scale_angular_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", scale_linear_, DEFAULT_SCALE_LINEAR);
	nh_.param("scale_pan", scale_pan_, DEFAULT_SCALE_TILT);
	nh_.param("scale_tilt", scale_tilt_, DEFAULT_SCALE_TILT);
	nh_.param("scale_zoom", scale_zoom_, DEFAULT_SCALE_TILT);

	// Digital Buttons
	nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
	nh_.param("button_update_ptz", button_update_ptz_, button_update_ptz_);
	nh_.param("button_ptz_reset", button_ptz_reset_, button_ptz_reset_);
	nh_.param("button_vel_inc", button_vel_inc_, button_vel_inc_);
	nh_.param("button_vel_dec", button_vel_dec_, button_vel_dec_);
	nh_.param("button_ptz_vel_dec", button_ptz_vel_dec_, button_ptz_vel_dec_);
	nh_.param("button_ptz_vel_inc", button_ptz_vel_inc_, button_ptz_vel_inc_);

	// Rover velocity output topic
	nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);

	// PTZ output topic
	nh_.param("cmd_topic_ptz", cmd_topic_ptz_, cmd_topic_ptz_);
	
	ROS_INFO("SummitXLPad num_of_buttons_ = %d", num_of_buttons_);

	for(int i = 0; i < num_of_buttons_; i++){
		bRegisteredButtonEvent[i] = false;
		ROS_INFO("bREG %d", i);
	}

	for(int i = 0; i < 3; i++)
		bRegisteredDirectionalArrows[i] = false;


	// Publish through the node handle Twist type messages to the guardian_controller/command topic
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);

	//  Publishes msgs for the pant-tilt cam
	ptz_pub_ = nh_.advertise<robotnik_msgs::ptz>(cmd_topic_ptz_, 1);

	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to summit_xl_controller/command)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SummitXLPad::padCallback, this);

	// Diagnostics
	updater_pad.setHardwareID("None");

	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
		diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel_.c_str(), updater_pad,
		diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	bEnable = false;	// Communication flag disabled by default

}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void SummitXLPad::Update(){
	updater_pad.update();
}



void SummitXLPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy){

	geometry_msgs::Twist vel;
	robotnik_msgs::ptz ptz;

	vel.linear.x = 0.0;
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;	
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;
	
	bEnable = (joy->buttons[dead_man_button_] == 1);

  	// Actions dependant on dead-man button
	if(bEnable){

		// Set the current velocity level
		if ( joy->buttons[button_vel_dec_] == 1 ){
			if(!bRegisteredButtonEvent[button_vel_dec_]) 
				if(current_vel > 0.1){
					current_vel = current_vel - 0.1;
					bRegisteredButtonEvent[button_vel_dec_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);
					char buf[50]="\0";
					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
				}
		}else
			bRegisteredButtonEvent[button_vel_dec_] = false;
		if (joy->buttons[button_vel_inc_] == 1){
			if(!bRegisteredButtonEvent[button_vel_inc_])
				if(current_vel < 0.9){
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[button_vel_inc_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);
					char buf[50]="\0";
					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
				}
		}else
			bRegisteredButtonEvent[button_vel_inc_] = false;

		// Set linear and angular velocities
		vel.linear.x = current_vel*scale_linear_*joy->axes[linear_];
		vel.angular.z = current_vel*scale_angular_*joy->axes[angular_];

	}

	// PTZ Velocity
	if(joy->buttons[button_ptz_vel_dec_] == 1) {
		if(!bRegisteredButtonEvent[button_ptz_vel_dec_]){
			if(ptz_vel > 0.2){
				ptz_vel = ptz_vel - 0.1;
				bRegisteredButtonEvent[button_ptz_vel_dec_] = true;
				ROS_INFO("PTZ Velocity: %f%%", ptz_vel*100.0);
				char buf[50]="\0";
				int percent = (int) (ptz_vel*100.0);
				sprintf(buf," %d percent", percent);
			}
		}
	}else
		bRegisteredButtonEvent[button_ptz_vel_dec_] = false;
	if(joy->buttons[button_ptz_vel_inc_] == 1) {
		if(!bRegisteredButtonEvent[button_ptz_vel_inc_]){
			if(ptz_vel < 0.9){
				ptz_vel = ptz_vel + 0.1;
				bRegisteredButtonEvent[button_ptz_vel_inc_] = true;
				ROS_INFO("PTZ Velocity: %f%%", ptz_vel*100.0);
				char buf[50]="\0";
				int percent = (int) (ptz_vel*100.0);
				sprintf(buf," %d percent", percent);
			}
		}
	}else
		bRegisteredButtonEvent[button_ptz_vel_inc_] = false;

	// PTZ Movements
	if (joy->buttons[button_update_ptz_] == 1) {
		if(!bRegisteredButtonEvent[button_update_ptz_]){
			ptz.relative = true;
			ptz.pan = -scale_pan_*ptz_vel*joy->axes[pan_];
			ptz.tilt = scale_tilt_*ptz_vel*joy->axes[tilt_];
			ptz.zoom = scale_zoom_*(joy->axes[zoom_wide_] - joy->axes[zoom_tele_]);
			ptz_pub_.publish(ptz);
			pub_command_freq->tick();
			bRegisteredButtonEvent[button_update_ptz_] = true;
		}
	}else{
		bRegisteredButtonEvent[button_update_ptz_] = false;
	}

	// PTZ Reset
	if (joy->buttons[button_ptz_reset_] == 1) {
		if(!bRegisteredButtonEvent[button_ptz_reset_]){
			ptz.relative = false;
			ptz.pan = 0;
			ptz.tilt = 0;
			ptz.zoom = 0;
			ptz_pub_.publish(ptz);
			pub_command_freq->tick();
			bRegisteredButtonEvent[button_ptz_reset_] = true;
		}
	}else{
		bRegisteredButtonEvent[button_ptz_reset_] = false;
	}

	// Publishing
	vel_pub_.publish(vel);
	sus_joy_freq->tick();

}

int main(int argc, char** argv){

	ros::init(argc, argv, "summit_xl_teleop");
	SummitXLPad summit_xl_teleop;

	ros::Rate r(50.0);

	while( ros::ok() ){
		// UPDATING DIAGNOSTICS
		summit_xl_teleop.Update();
		ros::spinOnce();
		r.sleep();
	}
}

