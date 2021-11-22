#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include <dynamic_reconfigure/server.h>
#include <position_controller/PIDConfig.h>
#include <position_controller/msgData.h>
#include <position_controller/msgCoordinates.h>
#include <math.h>
#include <string>
using namespace std;

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include <gazebo/gazebo_client.hh>

#define PI 3.14159265358979
struct sCoordinates
{
	double x,y,z,yaw;
};

class PositionController
{
	private:
		string drone_name;
		bool state_flying;
		sCoordinates target_pose, actual_pose, error, error_prev, integral, velocity, prev_vel;
		sCoordinates Kp, Ki, Kd;  // wzmocnienia regulatorow PID
		double max_vel_xy, max_vel_z, max_vel_yaw;  // ograniczenia wyjścia regulatorow
		double error_ctrl_x, error_ctrl_y;  // uchyby po przeliczeniu z katem yaw
		geometry_msgs::Twist cmd;  // velocity command
		
	public:	
		PositionController(sCoordinates z, string name): actual_pose(z), error(z), error_prev(z), integral(z), velocity(z), prev_vel(z), drone_name(name)
		{		
		}
		
		~PositionController();
	
		ros::Subscriber odom_sub, target_sub, land_sub, takeoff_sub, reset_sub;
		ros::Publisher vel_pub, data_pub, odom_conv_pub, state_pub;
		gazebo::transport::SubscriberPtr odom_sub_gz;
		
		void change_state_land(const std_msgs::Empty::ConstPtr& msg);
		void change_state_takeoff(const std_msgs::Empty::ConstPtr& msg);
		void update_pose_gz(ConstPosesStampedPtr &msg);
		void update_target(const position_controller::msgCoordinates::ConstPtr& msg);
		void control();
		void reconfig_callback(position_controller::PIDConfig &config);
		position_controller::msgData NewData();	
		bool flying();
		void target_after_takeoff(double altitude);
};

PositionController::~PositionController()
{
}

// aktualizacja stanu flying na false po ladowaniu
void PositionController::change_state_land(const std_msgs::Empty::ConstPtr& msg)
{
	std_msgs::Bool s;
	s.data = false;
	state_flying = false;
	state_pub.publish(s);
}
// aktualizacja stanu flying na true po starcie
void PositionController::change_state_takeoff(const std_msgs::Empty::ConstPtr& msg)
{
	std_msgs::Bool s;
	s.data = true;
	state_flying = true;
	state_pub.publish(s);
}
bool PositionController::flying()
{
	return state_flying;
}

// aktualizacja zadanej pozycji i orientacji (dane pobierane z topica target)
void PositionController::update_target(const position_controller::msgCoordinates::ConstPtr& msg)
{
	target_pose.x = msg->x;
	target_pose.y = msg->y;
	target_pose.z = msg->z;
	target_pose.yaw = msg->yaw;
}

// funkcja tuż przed startem ustawia pierwszy punkt docelowy, tak aby dron wzniosl sie pionowo w gore na zadana wysokosc
void PositionController::target_after_takeoff(double altitude)
{	
	target_pose.x = actual_pose.x;
	target_pose.y = actual_pose.y;
	target_pose.z = altitude;
	target_pose.yaw = actual_pose.yaw;
}

// aktualizacja danych odometrycznych drona z gazebo
void PositionController::update_pose_gz(ConstPosesStampedPtr &msg)
{
	position_controller::msgCoordinates conv;
	 for (int i =0; i < msg->pose_size(); ++i)
    {
		const ::gazebo::msgs::Pose &pose = msg->pose(i);
		if (pose.name() == drone_name)
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion q = pose.orientation();
			actual_pose.yaw = conv.yaw = atan2(  (2*(q.w()*q.z()+q.x()*q.y())) , (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() - q.z()*q.z()) )  * 180 / PI;
			actual_pose.x = conv.x = position.x();
			actual_pose.y = conv.y = position.y();
			actual_pose.z = conv.z = position.z();
			ROS_INFO("Actual pose - x: [%f], y: [%f], z:  [%f], yaw: [%f]\n", actual_pose.x, actual_pose.y, actual_pose.z, actual_pose.yaw);
		}
	}
	odom_conv_pub.publish(conv);
}

// przypisanie zmiennym programowym wartości zmiennych rekonfigurowalnych (zadeklarowanych w pliku PID.cfg)
void PositionController::reconfig_callback(position_controller::PIDConfig &config)
{
	max_vel_xy = config.max_vel_xy / 100.0;
	max_vel_z = config.max_vel_z / 100.0;
	max_vel_yaw = config.max_vel_yaw / 100.0;
	Kp.x = config.Kp_x;
	Ki.x = config.Ki_x;
	Kd.x = config.Kd_x;
	Kp.y = config.Kp_y;
	Ki.y = config.Ki_y;
	Kd.y = config.Kd_y;
	Kp.z = config.Kp_z;
	Ki.z = config.Ki_z;
	Kd.z = config.Kd_z;
	Kp.yaw = config.Kp_yaw;
	Ki.yaw = config.Ki_yaw;
	Kd.yaw = config.Kd_yaw;
}

// przygotowanie danych do wyslania do topica (/position_controller_data) ktory jest rejestrowany do baga na podstawie ktorego powstaja wykresy
position_controller::msgData PositionController::NewData()
{
   position_controller::msgData msgNewData;
	
	msgNewData.target_pos.x = target_pose.x;
	msgNewData.target_pos.y = target_pose.y;
	msgNewData.target_pos.z = target_pose.z;
	msgNewData.target_pos.yaw = target_pose.yaw;
	
	msgNewData.actual_pos.x = actual_pose.x;
    msgNewData.actual_pos.y = actual_pose.y;
	msgNewData.actual_pos.z = actual_pose.z;
	msgNewData.actual_pos.yaw = actual_pose.yaw;
	
	msgNewData.error_pos.x = error.x;
	msgNewData.error_pos.y = error.y;
	msgNewData.error_pos.z = error.z;
	msgNewData.error_pos.yaw = error.yaw;
	
	msgNewData.error_ctrl_x = error_ctrl_x;
	msgNewData.error_ctrl_y = error_ctrl_y;
		
	msgNewData.velocity_cmd.x = velocity.x;
	msgNewData.velocity_cmd.y = velocity.y;
	msgNewData.velocity_cmd.z = velocity.z;
	msgNewData.velocity_cmd.yaw = velocity.yaw;
	
	msgNewData.max_vel = max_vel_xy;
	msgNewData.min_vel = -max_vel_xy;
	
	msgNewData.Kp.x = Kp.x;
	msgNewData.Ki.x = Ki.x;
	msgNewData.Kd.x = Kd.x;
	msgNewData.Kp.y = Kp.y;
	msgNewData.Ki.y = Ki.y;
	msgNewData.Kd.y = Kd.y;
	msgNewData.Kp.z = Kp.z;
	msgNewData.Ki.z = Ki.z;
	msgNewData.Kd.z = Kd.z;
	msgNewData.Kp.yaw = Kp.yaw;
	msgNewData.Ki.yaw = Ki.yaw;
	msgNewData.Kd.yaw = Kd.yaw;
	
	return msgNewData;
}	

// główna funkcja regulacji PID
void PositionController::control()
{
	//------------------------------------------- control errors ----------------------------------------------
	error.x = target_pose.x - actual_pose.x;
	error.y = target_pose.y - actual_pose.y;
	error.z = target_pose.z - actual_pose.z;
	error.yaw = target_pose.yaw - actual_pose.yaw;
	
	// warunki zapewniające obroty we właściwą stronę (najkrótszą drogą):
	if(error.yaw < -180.0)  
		error.yaw = error.yaw + 360.0;
	else if(error.yaw > 180.0)
		error.yaw = error.yaw - 360.0;
	
	// przeliczenie uchybu z układu świata do układu drona
	error_ctrl_x = error.x * cos(actual_pose.yaw*PI/180.0) + error.y * sin(actual_pose.yaw*PI/180.0);
	error_ctrl_y = error.y * cos(actual_pose.yaw*PI/180.0) - error.x * sin(actual_pose.yaw*PI/180.0);
	
	//-------------------------------------------- anty-windup ------------------------------------------------------------
	if( ((prev_vel.x > max_vel_xy) && (error_ctrl_x >0)) || ((prev_vel.x < -max_vel_xy) && (error_ctrl_x <0)) )
		integral.x = 0;
	else
		integral.x += error_ctrl_x;
	
	if( ((prev_vel.y > max_vel_xy) && (error_ctrl_y >0)) || ((prev_vel.y < -max_vel_xy) && (error_ctrl_y <0)) )
		integral.y = 0;
	else
		integral.y += error_ctrl_y;
	
	if( ((prev_vel.z > max_vel_z) && (error.z >0)) || ((prev_vel.z < -max_vel_z) && (error.z <0)) )
		integral.z = 0;
	else
		integral.z += error.z;
	
	if( ((prev_vel.yaw > max_vel_yaw) && (error.yaw >0)) || ((prev_vel.yaw < -max_vel_yaw) && (error.yaw <0)) )
		integral.yaw = 0;
	else
		integral.yaw += error.yaw;
	
	//------------------------------------- calculate outputs -----------------------------------------------------
	velocity.x  =  	  Kp.x * error_ctrl_x   + Ki.x * integral.x 		 + Kd.x * (error_ctrl_x - error_prev.x);
	velocity.y  = 	  Kp.y * error_ctrl_y  + Ki.y * integral.y 		 + Kd.y * (error_ctrl_y - error_prev.y);
	velocity.z  =	  Kp.z * error.z 	  	   + Ki.z * integral.z 		 + Kd.z * (error.z - error_prev.z);
	velocity.yaw = Kp.yaw * error.yaw + Ki.yaw * integral.yaw + Kd.yaw * (error.yaw - error_prev.yaw);
	
	//------------------------------- save last output and error-----------------------------------------------
	prev_vel.x = velocity.x;
	prev_vel.y = velocity.y;
	prev_vel.z = velocity.z;
	prev_vel.yaw = velocity.yaw;
	// ----------------------------------
	error_prev.x = error_ctrl_x;
	error_prev.y = error_ctrl_y;
	error_prev.z = error.z;
	error_prev.yaw = error.yaw;
	
	//------------------------------------------ saturation -----------------------------------------------
	if (velocity.x < -max_vel_xy)
		velocity.x = -max_vel_xy;
	else if (velocity.x > max_vel_xy)
		velocity.x = max_vel_xy;
	if (velocity.y < -max_vel_xy)
		velocity.y = -max_vel_xy;
	else if (velocity.y > max_vel_xy)
		velocity.y = max_vel_xy;
	if (velocity.z < -max_vel_z)
		velocity.z = -max_vel_z;
	else if (velocity.z > max_vel_z)
		velocity.z = max_vel_z;
	// velocity in ,,z" axis is also limited by SpeedSettingsMaxVerticalSpeedCurrent parameter (default 1 m/s)
	if (velocity.yaw < -max_vel_yaw)
		velocity.yaw = -max_vel_yaw;
	else if (velocity.yaw > max_vel_yaw)
		velocity.yaw = max_vel_yaw;
	
	//---------------------------------- prepare command --------------------------------------------
	cmd.linear.x = velocity.x;	
	cmd.linear.y = velocity.y;	
	cmd.linear.z = velocity.z;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = velocity.yaw;	
	
	//--------------------------------- publish command -----------------------------------------------
	vel_pub.publish(cmd);
	data_pub.publish(this->NewData()); // send data to topic position_controller_data
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);  // in Hz
	sCoordinates zeros = {0,0,0,0};
	double init_altitude = 1.5;  // zadana wysokosc po starcie

	PositionController pc = PositionController(zeros, "bebop_leader");

// ----------------- gazebo -----------------------------------------------------------
	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
//------------------- publishers -------------------------------------------------------------	
	pc.vel_pub = n.advertise<geometry_msgs::Twist>("/bebop_leader/cmd_vel", 100);
	pc.data_pub = n.advertise<position_controller::msgData>("/position_controller_data", 100);
	pc.state_pub = n.advertise<std_msgs::Bool>("/bebop_leader/state", 100);
	pc.odom_conv_pub = n.advertise<position_controller::msgCoordinates>("/bebop_leader/odom_conv", 100);
//------------------- subscribers -----------------------------------------------------
	pc.odom_sub_gz = node->Subscribe("/gazebo/default/pose/info", &PositionController::update_pose_gz, &pc);
	pc.target_sub = n.subscribe("/bebop_leader/target", 100, &PositionController::update_target, &pc);
	pc.land_sub = n.subscribe("/bebop_leader/land", 100, &PositionController::change_state_land, &pc);
	pc.reset_sub = n.subscribe("/bebop_leader/reset", 100, &PositionController::change_state_land, &pc);
	pc.takeoff_sub = n.subscribe("/bebop_leader/takeoff", 100, &PositionController::change_state_takeoff, &pc);
//----------------------------------------------------------------------------------------------	
	dynamic_reconfigure::Server<position_controller::PIDConfig> server;
	dynamic_reconfigure::Server<position_controller::PIDConfig>::CallbackType f;
	f = boost::bind(&PositionController::reconfig_callback, boost::ref(pc), _1);
	server.setCallback(f);
	
	pc.target_after_takeoff(init_altitude);
		
	while (ros::ok())
	{
		if(pc.flying())
			pc.control();
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}
