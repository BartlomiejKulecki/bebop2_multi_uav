#include "ros/ros.h"
#include <position_controller/msgCoordinates.h>
#include <math.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <vector>
#define STOP 1
#define TRYING 2

struct sCoordinates
{
	double x,y,z,yaw;
};

double Ra; 
double Rz; 	
double Va = 0.001;
//========================
ros::Publisher reset_L, reset_f1;
ros::Publisher hover_L, hover_f1; 
ros::Publisher col_status;
ros::Subscriber odom_L, odom_f1;
ros::Subscriber  target_L, target_f1;
std_msgs::Empty emptyMsg;
std_msgs::Bool BoolMsg;

sCoordinates  pos_L, pos_f1;
sCoordinates  act_target_L, act_target_f1;
sCoordinates try_target1;
double d_prev1, d_prev2;
double dp1, dp2;
int stan_kolizji1;
std::vector<sCoordinates> bad_targets1;

// aktualizacja pozycji dronow
void update_pose_L(const position_controller::msgCoordinates::ConstPtr& msg)
{
	pos_L.x = msg->x;
	pos_L.y = msg->y;
	pos_L.z = msg-> z;
	pos_L.yaw = msg->yaw;
}
void update_pose_f1(const position_controller::msgCoordinates::ConstPtr& msg)
{
	pos_f1.x = msg->x;
	pos_f1.y = msg->y;
	pos_f1.z = msg-> z;
	pos_f1.yaw = msg->yaw;
}

// aktualizacja punktow docelowych
void update_target_L(const position_controller::msgCoordinates::ConstPtr& msg)
{
	act_target_L.x = msg->x;
	act_target_L.y = msg->y;
	act_target_L.z = msg-> z;
	act_target_L.yaw = msg->yaw;
}
void update_target_f1(const position_controller::msgCoordinates::ConstPtr& msg)
{
	act_target_f1.x = msg->x;
	act_target_f1.y = msg->y;
	act_target_f1.z = msg-> z;
	act_target_f1.yaw = msg->yaw;
}


/*  check_collision sprawdza odleglosci pomiedzy dronami i zwraca ogolny status kolizji:
0 - brak kolizji
1 - odleglosc miedzy Liderem a followerem1 jest mniejsza niz Ra
10 - odleglosc miedzy Liderem a followerem1 jest mniejsza niz Rz
*/
int check_collision(int id, sCoordinates p1, sCoordinates p2)
{
	double d = sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2) );
	int status = 0;
	
	if( d <= Rz)
		status = id*10;
	else if( d <= Ra)
	{
		double temp_d_prev;
		dp1 = d - d_prev1; 
		status = id;
	}
	else
		status = 0;
	
	d_prev1 = d;
	
	return status;
}

int check_status()
{
	int var = 0;
	int col1;
	col1 = check_collision(1, pos_L, pos_f1);
	
	if( col1 )
		var = col1;
	
	return var;
}

// reakcja na kolizje o podanym id (statusie ogolnym)
void react(int id_kolizji)
{
	switch(id_kolizji)
	{
		case 0:{
			BoolMsg.data = false;
			hover_L.publish(BoolMsg);
			hover_f1.publish(BoolMsg);
			stan_kolizji1 = STOP;
			bad_targets1.clear();
			break;
		}
		case 1:{
			switch(stan_kolizji1){
				case STOP:{
					BoolMsg.data = true;
					hover_L.publish(BoolMsg);
					hover_f1.publish(BoolMsg);
					bool target_is_bad = false;
					for(auto& pkt : bad_targets1)
					{
						if( (act_target_f1.x == pkt.x) && (act_target_f1.y == pkt.y) && (act_target_f1.z == pkt.z)) 
							target_is_bad = true;	
					}
					if(!target_is_bad)
					{
						if(  sqrt( pow(act_target_f1.x-pos_L.x,2) + pow(act_target_f1.y-pos_L.y,2) + pow(act_target_f1.z-pos_L.z,2)) > Ra)
						{
							try_target1.x = act_target_f1.x;
							try_target1.y = act_target_f1.y;
							try_target1.z = act_target_f1.z;
							BoolMsg.data = false;
							hover_f1.publish(BoolMsg);
							stan_kolizji1 = TRYING;
						}
					}
					break;
				}
				case TRYING:{
					if(dp1 < 0) // d sie zmniejsza
					{
						bad_targets1.push_back(try_target1);
						stan_kolizji1 = STOP;
					}
					else
						stan_kolizji1 =TRYING;
				break;
				}
			}			
		break;}
		
		case 10:
			reset_f1.publish(emptyMsg);
			break;
	}	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_avoidance");
	ros::NodeHandle n("~");
	ros::Rate loop_rate = 80;
	
	n.param("Ra", Ra, 1.0);
	n.param("Rz", Rz, 0.7);
	
	std_msgs::Int32 status;
	col_status = n.advertise<std_msgs::Int32>("/collision_avoidance/status", 10);
	
	odom_L = n.subscribe("/bebop_leader/odom_conv", 10, &update_pose_L);
	target_L = n.subscribe("/bebop_leader/target", 10, &update_target_L);
	odom_f1 = n.subscribe("/bebop_follower1/odom_conv", 10, &update_pose_f1);
	target_f1 = n.subscribe("/bebop_follower1/target", 10, &update_target_f1);
	
	ros::Duration(1.0).sleep();
	
	reset_L = n.advertise<std_msgs::Empty>("/bebop_leader/reset",1,true);
	reset_f1 = n.advertise<std_msgs::Empty>("/bebop_follower1/reset",1,true);
	hover_L = n.advertise<std_msgs::Bool>("/bebop_leader/hover",1,true);
	hover_f1 = n.advertise<std_msgs::Bool>("/bebop_follower1/hover",1,true);
	
	while (ros::ok())
	{
		status.data = check_status();
		col_status.publish(status);
		react(status.data);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


