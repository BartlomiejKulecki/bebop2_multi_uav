#include "ros/ros.h"
#include <position_controller/msgTrajectory.h>
#include <position_controller/msgCoordinates.h>
#include <math.h>
#include <fstream>
#include <string>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <ncurses.h>
using namespace std;
#define PI 3.14159265358979
#define sind(x) (sin((x) * PI / 180.0))
#define cosd(x) (cos((x) * PI / 180.0))
#define atan2d(y,x) (atan2(y,x) * 180.0/PI)
#define acosd(x)  (acos(x) * 180.0/PI)

struct sCoordinates
{
	double x,y,z,yaw;
};
struct sPoint2D
{
	double x, y;
};
struct sRectangle
{
	// corners:
	vector<sPoint2D> corners; //corners[0], corners[1], corners[2], corners[3];
};

class Drone
{
	private:
	
	public:
		string drone_name;
		position_controller::msgTrajectory trajectory;
		sCoordinates actual_pos, error_pos, target_pos, tolerance;
		int point_idx;
		bool position_reached;
		bool flying;

		ros::Publisher target_pub, takeoff_pub, land_pub, reset_pub;
		ros::Subscriber odom_sub, state_sub;
	
		Drone(string name, sCoordinates z, sCoordinates t) :  drone_name(name), error_pos(z), actual_pos(z), tolerance(t)
		{		
			point_idx = 0;
			flying = false;
			position_reached = false;
		}
		~Drone();
		
		void update_pose(const position_controller::msgCoordinates::ConstPtr& msg);
		void update_state(const std_msgs::Bool::ConstPtr& msg);
		void check_position_status();
		void publish_target();
		void target_after_takeoff(double altitude);
};

// ------------------------------ global variables ------------------------------------------------
string csv_path = "";
double d;
double angle;
double init_altitude;
double trajectory_altitude;
double object_distance;
double krok_prosta;
double krok_luk;
int trajectory_mode;
int main_loop_rate;
bool start = false;
bool ready = false;
bool during_trajectory = false;
bool last_point = false;
position_controller::msgTrajectory trajectory_raw;
sRectangle object;
ros::ServiceServer service_start;
std_msgs::Empty emptyMsg;

// ------------------------------ global functions ----------------------------------------------------

// odczyt trajektorii z pliku csv
void get_trajectory()
{
	int row = 0;
	string x,y,z,yaw;

	ifstream file(csv_path.c_str(),ifstream::in);
	if(!file.is_open())
		ROS_INFO("ERROR: File Not Open\n");
	else
	{
		while(file.good())
		{
			getline(file,x,',');
			getline(file,y,',');
			getline(file,z,',');
			getline(file,yaw,'\n');
			if(file.eof())
			{
				file.close(); 
				break;
			}
			trajectory_raw.x.push_back(stod(x));
			trajectory_raw.y.push_back(stod(y));
			trajectory_raw.z.push_back(stod(z));
			trajectory_raw.yaw.push_back(stod(yaw));
			row++;
		}
		file.close();
		printw("___________ Odczytano trajektorie z pliku ____________ \n");
	}
}

//  funkcja przypisujaca podana w pliku trajektorie do lidera
void set_trajectory(Drone& leader)
{
	int i = 0;
	while ( i < trajectory_raw.x.size() )
	{
		leader.trajectory.x.push_back(trajectory_raw.x[i]);
		leader.trajectory.y.push_back(trajectory_raw.y[i]);
		leader.trajectory.z.push_back(trajectory_raw.z[i]);
		leader.trajectory.yaw.push_back(trajectory_raw.yaw[i]);
		i++;		
	}
	printw("___________ Trajektoria lidera wyznaczona ____________ \n");
	ready = true;
	printw("____________ GOTOWY DO STARTU TRAJEKTORII ____________ \n");
}

// odczyt wspolrzednych obiektu z pliku csv
void get_object()
{
	int row = 0;
	string x,y;
	sPoint2D pkt;
	ifstream file(csv_path.c_str(),ifstream::in);
	if(!file.is_open())
		ROS_INFO("ERROR: File Not Open\n");
	else
	{
		while(file.good())
		{
			getline(file,x,',');
			getline(file,y,'\n');
			if(file.eof())
			{
				file.close(); 
				break;
			}
			pkt.x = stod(x);
			pkt.y = stod(y);
			object.corners.push_back(pkt);
			row++;
		}
		file.close();
		printw("__________ Odczytano wspolrzedne obiektu z pliku ___________ \n");
	}
}

// funkcja ktora na podstawie wspolrzednych obiektu i parametrow oblicza trajektorie (wektor punktow) dla lidera
void make_trajectory(Drone& leader, double altitude, double dist, double krokProsta, double krokKat)
{
	double alfa = atan2d(object.corners[0].y - object.corners[1].y , object.corners[1].x - object.corners[0].x);
	sPoint2D pstart;
	// punkt startowy w połowie lewego boku
	pstart.x = (object.corners[0].x + object.corners[1].x) / 2.0 + dist*sind(alfa);  
	pstart.y = (object.corners[0].y + object.corners[1].y) / 2.0 + dist*cosd(alfa);
	// poczatkowy kat yaw
	double KatYaw = -90.0-alfa;
	// aktualny punkt - zmienna pomocnicza
	sPoint2D act;  
	// obliczenie kroku - odleglosci miedzy kolejnymi punktami na prostych
	double krokA , krokB;
	krokA = krokB = krokProsta;
	double dlugosc_bokuA = sqrt( pow((object.corners[1].x - object.corners[0].x),2) + pow((object.corners[1].y - object.corners[0].y),2) );
	double dlugosc_bokuB = sqrt( pow((object.corners[1].x - object.corners[2].x),2) + pow((object.corners[1].y - object.corners[2].y),2) );
	double liczba_pktA =  dlugosc_bokuA / krokA;
	double liczba_pktB =  dlugosc_bokuB / krokB;
	liczba_pktA = round(liczba_pktA);
	liczba_pktB = round(liczba_pktB);
	krokA = dlugosc_bokuA / liczba_pktA;
	krokB = dlugosc_bokuB / liczba_pktB;

	double katLuk = alfa; 
	double liczba_pktLuk = 90.0/krokKat;
		
	// zapisanie pierwszego punktu
	act.x = pstart.x;
	act.y = pstart.y;
	leader.trajectory.x.push_back(act.x);
	leader.trajectory.y.push_back(act.y);
	leader.trajectory.z.push_back(altitude);
	leader.trajectory.yaw.push_back(KatYaw);
	int licznik = 1;
	printw("___________ Kat obrotu obiektu = %.2f ____________ \n", alfa);

	// ===================== W PRZÓD	=================
	do 
	{
		act.x += krokA*cosd(alfa);
		act.y -= krokA*sind(alfa);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
	}
	while( licznik < 1+0.5*liczba_pktA );
	// ===================== ŁUK 1	=================	
	do 
	{
		KatYaw -= krokKat;
		katLuk += krokKat;
		act.x = object.corners[1].x + dist * sind(katLuk);
		act.y = object.corners[1].y + dist * cosd(katLuk);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
		if(KatYaw <= -180.0)
			KatYaw = 180.0;
	}
	while( licznik < 1+0.5*liczba_pktA+liczba_pktLuk );
	// ===================== W PRAWO	=================
	do 
	{
		act.x -= krokB*sind(alfa);
		act.y -= krokB*cosd(alfa);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
	}
	while( licznik < 1+0.5*liczba_pktA+liczba_pktLuk+liczba_pktB );
	// ===================== ŁUK 2	=================
	do  
	{
		KatYaw -= krokKat;
		katLuk += krokKat;
		act.x = object.corners[2].x + dist * sind(katLuk);
		act.y = object.corners[2].y + dist * cosd(katLuk);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
		if(KatYaw <= -180.0)
			KatYaw = 180.0;
	}
	while( licznik < 1+0.5*liczba_pktA+2*liczba_pktLuk+liczba_pktB );
	// ===================== W TYŁ	=================
	do 
	{
		act.x -= krokA*cosd(alfa);
		act.y += krokA*sind(alfa);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
	}
	while( licznik < 1+1.5*liczba_pktA+2*liczba_pktLuk+liczba_pktB);
	// ===================== ŁUK 3	=================
	do  
	{
		KatYaw -= krokKat;
		katLuk += krokKat;
		act.x = object.corners[3].x + dist * sind(katLuk);
		act.y = object.corners[3].y + dist * cosd(katLuk);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
		if(KatYaw <= -180.0)
			KatYaw = 180.0;
	}
	while( licznik < 1+1.5*liczba_pktA+3*liczba_pktLuk+liczba_pktB );
	// ===================== W LEWO	=================
	do 
	{
		act.x += krokB*sind(alfa);
		act.y += krokB*cosd(alfa);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
	}
	while( licznik < 1+1.5*liczba_pktA+3*liczba_pktLuk+2*liczba_pktB );
	// ===================== ŁUK 4	=================
	do  
	{
		KatYaw -= krokKat;
		katLuk += krokKat;
		act.x = object.corners[0].x + dist * sind(katLuk);
		act.y = object.corners[0].y + dist * cosd(katLuk);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
		if(KatYaw <= -180.0)
			KatYaw = 180.0;
	}
	while( licznik < 1+1.5*liczba_pktA+4*liczba_pktLuk+2*liczba_pktB );
	// ===================== W PRZÓD	=================
	do 
	{
		act.x += krokA*cosd(alfa);
		act.y -= krokA*sind(alfa);
		leader.trajectory.x.push_back(act.x);
		leader.trajectory.y.push_back(act.y);
		leader.trajectory.z.push_back(altitude);
		leader.trajectory.yaw.push_back(KatYaw);
		licznik++;
	}
	while( licznik < 1+2*liczba_pktA+4*liczba_pktLuk+2*liczba_pktB );
	// ================================================

	// punkt dodatkowy - odlot od boxa
	leader.trajectory.x.push_back(pstart.x);
	leader.trajectory.y.push_back(pstart.y+0.2);
	leader.trajectory.z.push_back(altitude);
	leader.trajectory.yaw.push_back(KatYaw);
	
	printw("___________ Trajektoria lidera wyznaczona ____________ \n");
	ready = true;
	printw("____________ GOTOWY DO STARTU TRAJEKTORII ____________ \n");
}

// funkcja wykonujaca sie w watku obslugujacym klawiature 
void key_thread(Drone& L) 
{
	ros::Rate r(30);
	int key;
	printw("Obsluga programu: \n  1 - Takeoff \n  2 - Land \n  3 - Start trajektorii \n  ESC - Emergency Stop \nCtrl+C aby zakonczyc \n \n");
	refresh();
	while (ros::ok()) 
	{
		r.sleep();
		key = getch();
		if( key == 27)  // ESC key
		{
			printw("Wyslano komende: ________ EMERGENCY STOP!!! _________\n");
			refresh();
			L.reset_pub.publish(emptyMsg);
		}
		else if ( key == '1')
		{
			printw("Wyslano komende: __________ Takeoff ___________\n");
			refresh();
			L.target_after_takeoff(init_altitude);
			L.takeoff_pub.publish(emptyMsg);
		}
		else if ( key == '2')
		{
			printw("Wyslano komende: __________ Landing ___________\n");
			refresh();
			L.land_pub.publish(emptyMsg);
		}
		else if ( ( key == '3') &&  L.flying && ready)
		{
			printw("Wyslano komende: ________ Start trajektorii! ________\n");
			refresh();
			during_trajectory = true;
		}		
	}
	endwin();
}


// ----------------------- Drone class methods -------------------------------------------------------------------------------
Drone::~Drone()
{
}

// aktualizacja biezacej pozycji drona
void Drone::update_pose(const position_controller::msgCoordinates::ConstPtr& msg)
{
	actual_pos.x = msg->x;
	actual_pos.y = msg->y;
	actual_pos.z = msg-> z;
	actual_pos.yaw = msg->yaw;
}

// aktualizacja stanu flying
void Drone::update_state(const std_msgs::Bool::ConstPtr& msg)
{
	flying = msg->data;
	if (!flying)
	{
		during_trajectory = false;
		point_idx = 0;
		position_reached = false;
	}
}

// funkcja tuż przed startem ustawia pierwszy punkt docelowy, tak aby dron wzniosl sie pionowo w gore na zadana wysokosc
void Drone::target_after_takeoff(double altitude)
{
	position_controller::msgCoordinates target_msg;
	
	target_pos.x = target_msg.x = actual_pos.x;
	target_pos.y = target_msg.y = actual_pos.y;
	target_pos.z = target_msg.z = altitude;
	target_pos.yaw = target_msg.yaw = actual_pos.yaw;
	
	target_pub.publish(target_msg);
}

//  wysyłanie danego punktu z trajektorii
void Drone::publish_target()
{
	position_controller::msgCoordinates target_msg;
	
	target_pos.x = target_msg.x = trajectory.x[point_idx];
	target_pos.y = target_msg.y = trajectory.y[point_idx];
	target_pos.z = target_msg.z = trajectory.z[point_idx];
	target_pos.yaw = target_msg.yaw = trajectory.yaw[point_idx];
	
	target_pub.publish(target_msg);
	point_idx++;
}

// funkcja sprawdza czy uchyb pozycji drona miesci sie w zakresie tolerancji
void Drone::check_position_status()
{ 
	position_reached = false;
	bool x_ok, y_ok, z_ok, yaw_ok = false;

	// obliczenie uchybów
	error_pos.x = fabs( target_pos.x - actual_pos.x );
	error_pos.y = fabs( target_pos.y - actual_pos.y );
	error_pos.z = fabs( target_pos.z - actual_pos.z );
	if ((target_pos.yaw > 180.0 - tolerance.yaw)  && (actual_pos.yaw < 0.0)) 
		error_pos.yaw = fabs( target_pos.yaw - fabs(actual_pos.yaw) );
	else if ((target_pos.yaw < -180.0 + tolerance.yaw)  && (actual_pos.yaw > 0.0)) 
		error_pos.yaw = fabs( fabs(target_pos.yaw) - actual_pos.yaw );
	else
		error_pos.yaw = fabs( target_pos.yaw - actual_pos.yaw );

	// sprawdź czy aktualna pozycja w danej osi, mieści się w dozwolonej tolerancji
	if(error_pos.x <= tolerance.x) 
		x_ok = true;
	if(error_pos.y <= tolerance.y)
		y_ok = true;
	if(error_pos.z <= tolerance.z)
		z_ok = true;
	if(error_pos.yaw <= tolerance.yaw)
		yaw_ok = true;

	//Jeśli każda aktualna współrzędna mieści się w dopuszczalnym zakresie to uznaj pozycję za osiągniętą
	if(x_ok && y_ok && z_ok && yaw_ok) 
		position_reached = true;
	else
		position_reached = false;	
}

// ----------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char **argv)
{
	ros::init(argc, argv, "formation_controller");
	ros::NodeHandle n("~");
	n.param("main_loop_rate", main_loop_rate, 1);
	n.param("trajectory_mode", trajectory_mode, 1);
	n.param("d", d, 1.0);
	n.param("angle", angle, 45.0);
	n.param("initial_altitude", init_altitude, 1.0);
	n.param("trajectory_altitude", trajectory_altitude, 1.0);
	n.param("object_distance", object_distance, 1.0);
	n.param("krok_prosta", krok_prosta, 0.1);
	n.param("krok_luk", krok_luk, 5.0);
	n.getParam("csv_path", csv_path);
	ros::Rate loop_rate(main_loop_rate);
	
	sCoordinates zeros = {0,0,0,0};
	sCoordinates tolerance;
	n.param("tolerance/x", tolerance.x, 0.1);
	n.param("tolerance/y", tolerance.y, 0.1);
	n.param("tolerance/z", tolerance.z, 0.1);
	n.param("tolerance/yaw", tolerance.yaw, 1.0);
	
	Drone leader = Drone("bebop_leader", zeros, tolerance);

// ----------------------- publishers ---------------------------
	leader.target_pub = n.advertise<position_controller::msgCoordinates>("/bebop_leader/target", 100);
	leader.takeoff_pub = n.advertise<std_msgs::Empty>("/bebop_leader/takeoff",1,true);
	leader.land_pub = n.advertise<std_msgs::Empty>("/bebop_leader/land",1,true);
	leader.reset_pub = n.advertise<std_msgs::Empty>("/bebop_leader/reset",1,true);
	
// --------------------------- subscribers ---------------------------------	
	leader.odom_sub = n.subscribe("/bebop_leader/odom_conv", 100, &Drone::update_pose, &leader);
	leader.state_sub = n.subscribe("/bebop_leader/state", 10, &Drone::update_state, &leader);

	
	initscr();
        nocbreak();  // cbreak  lub  nocbreak
        noecho();
	nodelay(stdscr, TRUE);
	thread Key_thread(key_thread, ref(leader));  //start thread for keyboard service
	
	// pobranie danych i przeliczenie trajektorii
	switch(trajectory_mode){
		case 1:
			get_object();
			make_trajectory(leader, trajectory_altitude, object_distance, krok_prosta, krok_luk);
			break;
		case 2:
			get_trajectory();
			set_trajectory(leader);
			break;
	}
	
	while (ros::ok())
	{
		if(during_trajectory)
		{	
			if(leader.point_idx == 0)
			{
				printw(" ______ Zadaje punkt nr 1 _____\n");
				refresh();
				leader.publish_target();
			}
			leader.check_position_status();
			
			if( leader.point_idx < leader.trajectory.x.size()) // Sprawdza czy wczytano ostatni zestaw współrzędnych
				last_point = false;
			else
				last_point = true;
			
			if( leader.position_reached  && !last_point) //	Jeśli doleciał do punktu to poślij mu następny
			{
				printw("______ Cel osiagniety! Zadaje kolejny, punkt nr %d _____\n", leader.point_idx+1);
				refresh();
				leader.publish_target();
				leader.position_reached = false;
			}
			else if( leader.position_reached && last_point) // Jeśli doleciał do punktu i był to ostatni punkt to zdejmij flagę during_trajectory i wraca do wyboru opcji
			{
				printw("______________ KONIEC trajektorii ______________\n");
				refresh();
				last_point = false;
				during_trajectory = false;
				leader.point_idx = 0;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	endwin();
	return 0;
}
