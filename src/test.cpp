#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>


using namespace std;
//Variables
int intOp;
uint8_t battery;
int F=1,vx,vy=0;
float posex, posey, poseth;
geometry_msgs::Twist emma_cmd;

//Funciones
void c_vel (float lin_x, float lin_y, float lin_z, float ang_z);

//Funciones Callback
void opt_callback(const std_msgs::Int32::ConstPtr& msg1){
	intOp=msg1->data;
}

void My_callback(const std_msgs::Int32::ConstPtr& msg2){
	vy=msg2->data;
}

void pose_callback(const turtlesim::Pose::ConstPtr msg2){
	posex=msg2->x;
	posey=msg2->y;
	poseth=msg2->theta;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "MCN");
  ros::NodeHandle nodo_;
  ros::Publisher takeoff_pub_ = nodo_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
  ros::Publisher land_pub_ = nodo_.advertise<std_msgs::Empty>("/bebop/land", 1);
  ros::Publisher fb_pub = nodo_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
  //teclado	
  ros::Subscriber tec_sub = nodo_.subscribe("/copt",10,opt_callback);
  ros::Subscriber pos_sub = nodo_.subscribe("/turtle1/pose",10,pose_callback);
  ros::Subscriber Mx_sub = nodo_.subscribe("/MY",1,My_callback);  
  //ros::Subscriber My_sub = nodo_.subscribe("/MY",1,My_callback);								
  	
  std_msgs::Empty takeoff_cmd;
  std_msgs::Empty land_cmd;	

  //Hover

  while(ros::ok()){
	switch(intOp){
		case 49: //Tecla 1
			switch(F){
				case 1:
					if(vy < 300){
						std::cout<<"Prueba 2 "<<"B = "<<" % "<<"Avanzando F = "<< F << " " << vy << endl;
						c_vel(1,0,0,0);
						fb_pub.publish(emma_cmd);
						F=1;
					}else{
						std::cout<<"Prueba 2 "<<"B = "<<(int)battery<<" % "<<"Hover F = "<< F << endl;
						c_vel(0,0,0,0);
						fb_pub.publish(emma_cmd);
						F=2;
					}
				break;
		
				case 2:
					std::cout<<"Prueba 2 "<<"B = "<<(int)battery<<" % "<<" Subiendo :) F = " << F << endl;
					c_vel(0,0,0,1);
					fb_pub.publish(emma_cmd);
					if(poseth > 1.50){ F=3; }					
					else { F=2; }
				break;
		
				case 3:
					std::cout<<"Prueba 2 "<<"B = "<<(int)battery<<" % "<<" Subiendo :) F = " << F << endl;
					c_vel(1,0,0,0);
					fb_pub.publish(emma_cmd);
					if(posey > 7.0){ F=4; }					
					else { F=3; }		
				break;
		
				case 4:
					std::cout<<"Prueba 2 "<<"B = "<<(int)battery<<" % "<<" Pasando el Muro  F = " << F << endl;
					c_vel(0,0,0,-1);
					fb_pub.publish(emma_cmd);
					if(poseth > 0.0){ F=4; }					
					else { F=5; }		
				break;
		
				case 5:
					std::cout<<"Prueba 2 "<<"B = "<<(int)battery<<" % "<<" Pasando el Muro  F = " << F << endl;
					c_vel(1,0,0,0);
					fb_pub.publish(emma_cmd);
					if(posex > 9){ F=6; }					
					else { F=5; }		
				break;
			}
		break;
		case 50:// Tecla 2
			std::cout<<"Take Off "<<"B "<<(int)battery<<" % \n";
			takeoff_pub_.publish(takeoff_cmd);
		break;
		case 51: //Tecla 3
			std::cout<<"Land "<<"B "<<(int)battery<<" % \n";
			land_pub_.publish(land_cmd);
		break;
		case 52://Tecla 4
			std::cout<<"Hover "<<"B "<<(int)battery<<" % \n";
			c_vel(0.0,0.0,0.0,0.0);
			fb_pub.publish(emma_cmd);
		break;
		default:	
			std::cout<<"Hover "<<"B "<<(int)battery<<" % \n";
			c_vel(0.0,0.0,0.0,0.0);
			fb_pub.publish(emma_cmd);
		break;
	}      	
	ros::spinOnce();
	//ros::spin();
  }
  return 0;	
}

void c_vel (float lin_x, float lin_y, float lin_z, float ang_z){
	emma_cmd.linear.x = lin_x;
  	emma_cmd.linear.y = lin_y;
  	emma_cmd.linear.z = lin_z;
  	emma_cmd.angular.z = ang_z;
	
}

