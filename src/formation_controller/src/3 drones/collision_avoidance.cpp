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
//================================
ros::Publisher reset_L, reset_f1, reset_f2;
ros::Publisher hover_L, hover_f1, hover_f2;
ros::Publisher col_status;
ros::Subscriber odom_L, odom_f1, odom_f2;
ros::Subscriber  target_L, target_f1, target_f2;
std_msgs::Empty emptyMsg;
std_msgs::Bool BoolMsg;

sCoordinates  pos_L, pos_f1, pos_f2;
sCoordinates  act_target_L, act_target_f1, act_target_f2;
sCoordinates try_target1, try_target2, try_target3, try_target4;
double d_prev1, d_prev2, d_prev3;
double dp1, dp2, dp3;
int stan_kolizji1, stan_kolizji2, stan_kolizji3, stan_kolizji4;
std::vector<sCoordinates> bad_targets1, bad_targets2, bad_targets3, bad_targets4;

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
void update_pose_f2(const position_controller::msgCoordinates::ConstPtr& msg)
{
	pos_f2.x = msg->x;
	pos_f2.y = msg->y;
	pos_f2.z = msg-> z;
	pos_f2.yaw = msg->yaw;
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
void update_target_f2(const position_controller::msgCoordinates::ConstPtr& msg)
{
	act_target_f2.x = msg->x;
	act_target_f2.y = msg->y;
	act_target_f2.z = msg-> z;
	act_target_f2.yaw = msg->yaw;
}

/*  check_collision sprawdza odleglosci pomiedzy dronami i zwraca ogolny status kolizji:
0 - brak kolizji
1 - odleglosc miedzy Liderem a followerem1 jest mniejsza niz Ra
2 - odleglosc miedzy Liderem a followerem2 jest mniejsza niz Ra
3 - odleglosc miedzy followerem1 a followerem2 jest mniejsza niz Ra
4 - gdy jednoczesnie wystepuje przypadek 1 i 2  lub  1 i 3  lub  2 i 3
10 - odleglosc miedzy Liderem a followerem1 jest mniejsza niz Rz
20 - odleglosc miedzy Liderem a followerem2 jest mniejsza niz Rz
30 - odleglosc miedzy followerem1 a followerem2 jest mniejsza niz Rz
40 - gdy jednoczesnie wystepuje przypadek 1 i 2  lub  1 i 3  lub  2 i 3
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
		switch(id){
		case 1:  dp1 = d - d_prev1; break;
		case 2:  dp2 = d - d_prev2; break;
		case 3:  dp3 = d - d_prev3; break;
		}
		
		status = id;
	}
	else
		status = 0;
	
	switch(id){
		case 1:  d_prev1 = d; break;
		case 2:  d_prev2 = d; break;
		case 3:  d_prev3 = d; break;
	}
	return status;
}

int check_status()
{
	int var = 0;
	int col1, col2, col3;
	col1 = check_collision(1, pos_L, pos_f1);
	col2 = check_collision(2, pos_L, pos_f2);
	col3 = check_collision(3, pos_f1, pos_f2);
	
	if( col1 )
	{
		if(col2 || col3)
		{
			if((col1==10)||(col2==20)||(col3==30))
				var=40;
			else
				var = 4;
		}
		else
			var = col1;
	}
	if( col2 )
	{
		if(col1 || col3)
		{
			if((col1==10)||(col2==20)||(col3==30))
				var=40;
			else
				var = 4;
		}
		else
			var = col2;
	}
	if( col3 )
	{
		if(col2 || col1)
		{
			if((col1==10)||(col2==20)||(col3==30))
				var=40;
			else
				var = 4;
		}
		else
			var = col3;
	}
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
			hover_f2.publish(BoolMsg);
			stan_kolizji1 = STOP;
			stan_kolizji2 = STOP;
			stan_kolizji3 = STOP;
			bad_targets1.clear();
			bad_targets2.clear();
			bad_targets3.clear();
			break;
		}
		case 1:{
			stan_kolizji2 = STOP;
			stan_kolizji3 = STOP;
			stan_kolizji4 = STOP;
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
		case 2:{
			stan_kolizji1 = STOP;
			stan_kolizji3 = STOP;
			stan_kolizji4 = STOP;
			switch(stan_kolizji2){
				case STOP:{
					BoolMsg.data = true;
					hover_L.publish(BoolMsg);
					hover_f2.publish(BoolMsg);
					bool target_is_bad = false;
					for(auto& pkt : bad_targets2)
					{
						if( (act_target_f2.x == pkt.x) && (act_target_f2.y == pkt.y) && (act_target_f2.z == pkt.z)) 
							target_is_bad = true;	
					}
					if(!target_is_bad)
					{
						if(  sqrt( pow(act_target_f2.x-pos_L.x,2) + pow(act_target_f2.y-pos_L.y,2) + pow(act_target_f2.z-pos_L.z,2)) > Ra)
						{
							try_target2.x = act_target_f2.x;
							try_target2.y = act_target_f2.y;
							try_target2.z = act_target_f2.z;
							BoolMsg.data = false;
							hover_f2.publish(BoolMsg);
							stan_kolizji2 = TRYING;
						}
					}
					break;
				}
				case TRYING:{
					if(dp2 < 0) // d sie zmniejsza
					{
						bad_targets2.push_back(try_target2);
						stan_kolizji2 = STOP;
					}
					else
						stan_kolizji2 =TRYING;
					break;
				}
			}			
			break;
		}
		case 3:{
			stan_kolizji1 = STOP;
			stan_kolizji2 = STOP;
			stan_kolizji4 = STOP;
			switch(stan_kolizji3){
				case STOP:{
					BoolMsg.data = true;
					hover_f1.publish(BoolMsg);
					hover_f2.publish(BoolMsg);
					bool target_is_bad = false;
					for(auto& pkt : bad_targets3)
					{
						if( (act_target_f1.x == pkt.x) && (act_target_f1.y == pkt.y) && (act_target_f1.z == pkt.z)) 
							target_is_bad = true;	
					}
					if(!target_is_bad)
					{
						if(  sqrt( pow(act_target_f1.x-pos_f2.x,2) + pow(act_target_f1.y-pos_f2.y,2) + pow(act_target_f1.z-pos_f2.z,2)) > Ra)
						{
							try_target3.x = act_target_f1.x;
							try_target3.y = act_target_f1.y;
							try_target3.z = act_target_f1.z;
							BoolMsg.data = false;
							hover_f1.publish(BoolMsg);
							stan_kolizji3 = TRYING;
						}
					}
					break;
				}
				case TRYING:{
					if(dp3 < 0) // d sie zmniejsza
					{
						bad_targets3.push_back(try_target3);
						stan_kolizji3 = STOP;
					}
					else
						stan_kolizji3 =TRYING;
					break;
				}
			}			
			break;
		}
		case 4:{ 
			stan_kolizji1 = STOP;
			stan_kolizji2 = STOP;
			stan_kolizji3 = STOP;
			switch(stan_kolizji4){
				case STOP:{
					BoolMsg.data = true;
					hover_L.publish(BoolMsg);
					hover_f1.publish(BoolMsg);
					hover_f2.publish(BoolMsg);
					bool target_is_bad = false;
					for(auto& pkt : bad_targets4)
					{
						if( (act_target_f2.x == pkt.x) && (act_target_f2.y == pkt.y) && (act_target_f2.z == pkt.z)) 
							target_is_bad = true;	
					}
					if(!target_is_bad)
					{
						double d_L = sqrt( pow(act_target_f2.x-pos_L.x,2) + pow(act_target_f2.y-pos_L.y,2) + pow(act_target_f2.z-pos_L.z,2));
						double d_f1 = sqrt( pow(act_target_f2.x-pos_f1.x,2) + pow(act_target_f2.y-pos_f1.y,2) + pow(act_target_f2.z-pos_f1.z,2));
						if(  (d_L > Ra) && (d_f1 > Ra) )
						{
							try_target4.x = act_target_f2.x;
							try_target4.y = act_target_f2.y;
							try_target4.z = act_target_f2.z;
							BoolMsg.data = false;
							hover_f2.publish(BoolMsg);
							stan_kolizji4 = TRYING;
						}
					}
					break;
				}
				case TRYING:{
					if((dp2 < 0)||(dp3 < 0)) // d sie zmniejsza
					{
						bad_targets4.push_back(try_target4);
						stan_kolizji4 = STOP;
					}
					else
						stan_kolizji4 =TRYING;
				break;
				}
			}		
			break;
		}
		case 10:
			reset_f1.publish(emptyMsg);
			break;
		case 20:
			reset_f2.publish(emptyMsg);
			break;
		case 30:
			reset_f2.publish(emptyMsg);
			break;
		case 40:
			reset_f1.publish(emptyMsg);
			reset_f2.publish(emptyMsg);
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
	odom_f2 = n.subscribe("/bebop_follower2/odom_conv", 10, &update_pose_f2);
	target_f2 = n.subscribe("/bebop_follower2/target", 10, &update_target_f2);
	
	ros::Duration(1.0).sleep();
	
	reset_L = n.advertise<std_msgs::Empty>("/bebop_leader/reset",1,true);
	reset_f1 = n.advertise<std_msgs::Empty>("/bebop_follower1/reset",1,true);
	reset_f2 = n.advertise<std_msgs::Empty>("/bebop_follower2/reset",1,true);
	hover_L = n.advertise<std_msgs::Bool>("/bebop_leader/hover",1,true);
	hover_f1 = n.advertise<std_msgs::Bool>("/bebop_follower1/hover",1,true);
	hover_f2 = n.advertise<std_msgs::Bool>("/bebop_follower2/hover",1,true);
	
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


