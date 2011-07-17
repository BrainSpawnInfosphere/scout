/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 12/20/2010
 *********************************************************************
 *
 * Change Log:
 * 20 Dec 2010 Created
 *
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>        // rviz visual markers
#include <tf/transform_broadcaster.h>         // transforms
#include <nav_msgs/Odometry.h>                // odometry
#include <geometry_msgs/Twist.h>              // command and velocity
#include <sensor_msgs/Imu.h>                  // IMU messages
#include <scout/IR.h> // all ir sensors?
#include <scout/Battery.h> // power?
#include <cereal_port/CerealPort.h> // C++ serial port

#include <string> // C++ for CerealPort

// Navigation -----------------------
#define DISTANCE_PER_CNT 5.4978f // mm/cnt 42*PI/24 = Dia*PI/CNT - linear distance
#define RADIANS_PER_CNT 0.0618424f // rads/cnt (42mm/3.5")*PI/24 = D/W*PI/CNT
#define TWO_PI 2.0*M_PI
#define EARTH_GRAVITY 9.81
#define AXLE_LENGTH 0.127 // 5 inches in meters

///////////////////////////////////////////////////////////////////////////////
//------------------------//
// Robot commands

#define CMD_START 128
#define CMD_STOP 129

#define CMD_FORWARD 130
#define CMD_RIGHT 131
#define CMD_LEFT 132
#define CMD_REVERSE 133
#define CMD_HALT 134
#define CMD_MOTOR_SPEED 135

#define CMD_SERVO_0 136
#define CMD_SERVO_1 137

#define CMD_BEEP 138
#define CMD_SONG 139
#define CMD_RESET 140

#define CMD_ALL_SENSORS 141
#define CMD_PROX_SENSORS 142
#define CMD_NAV_SENSORS 143
#define CMD_STATUS_SENSORS 144
#define CMD_IR_SENSORS 145

#define CMD_EEPROM_CLEAR 1
#define CMD_EEPROM_READ 1
#define CMD_EEPROM_WRTIE 1

//------------------------//
// Size of data returns

#define RTN_ALL_SENSORS 25
#define RTN_BOOT 7
#define RTN_START 7
#define RTN_STOP 6

#define BATTERY_CAPACITY (4.8*1000) // mAhrs
#define BATTERY_VOLTAGE  4.8 // V

#define BIT_1 1
#define BIT_2 2
#define BIT_3 4
#define BIT_4 8
#define BIT_5 16

//------------------------//
// Sensor cache
// 23 bytes not counting start/end chars
typedef struct {
	unsigned char ir; // bit7 [x x x stair1 stair0 ir2 ir1 ir0] bit0
	short compass;
	float x,y,z,w; // accel(x,y,z) gyro(w)
	float v_ref;
	float batt;
	long en0,en1;
	unsigned int time;      // milli-sec from uC
	unsigned int last_time; // milli-sec from uC
	//float distance;
	//float angle;
} sensors_t;

/**
 * All calculations come from the encoders
 */
typedef struct {
	float x,y,theta;   // position state from encoders
	float xdot,ydot,w; // velocity state from encoders
	float distance;    // distance from encoders

	float gtheta;      // angle from gyro integration

	unsigned int time;
	unsigned int last_time;
} navigation_t;


///////////////////////////////////////////////////////////////////////////////

cereal::CerealPort *serial_port = 0;

float battery = 0.0f;
bool blink = false;
bool irObj = false;
unsigned short dist = 0;


// *****************************************************************************
// Close the serial port
int openSerialPort(cereal::CerealPort *sp, std::string &port_name, int baud)
{
	//this->drive(0.0, 0.0);
	//usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	try{ sp->open(port_name.c_str(), baud); }
	catch(cereal::Exception& e){ return(-1); }
	
	return(0); 
}


// *****************************************************************************
// Close the serial port
int closeSerialPort(cereal::CerealPort *sp)
{
	//this->drive(0.0, 0.0);
	//usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	serial_port->write("<S>",3);
	
	try{ sp->close(); }
	catch(cereal::Exception& e){ return(-1); }
	
	return(0); 
}

// *****************************************************************************
// Calculate Roomba odometry


// *****************************************************************************
// Reset Roomba odometry
void resetOdometry()
{
	//this->setOdometry(0.0, 0.0, 0.0);
}

namespace kevin {
	
	float map(int x, float l1, float h1, float l2, float h2){
		float ans = ((float)(x)-l1)*(h2-l2)/(h1-l1)+l2;
		return ans;
	}
	
	float map(float x, float l1, float h1, float l2, float h2){
		float ans = (x-l1)*(h2-l2)/(h1-l1)+l2;
		return ans;
	}
	
}

inline float toV(int a){
	//float ans = (float)(a)/1024.0f*3.3f;
	float ans = kevin::map((float)a,0,1023,0,3.3);
	return ans;
}

float toG(int a){
	//float g = 0.330f*toV(a);
	float g = kevin::map(a,0,1023,-3.6,3.6);
	return g;
}

/*
 5V->2.5V
 FIXME: KJW 20101128
 */
float toW(int z, int ref)
{
	float w; 
	//float r = kevin::map(ref,0.0f,1023.0f,0.0f,3.3f);    // ref in V
	//float t = 2.0f*kevin::map(z,0.0f,1023.0f,0.0f,3.3f); // w in V
	//w = kevin::map(t-r,-2.5f,2.5f,-150.0f,150.0f);    // w in deg/sec
	
	w = kevin::map( z-ref, -512, 511, -150.0, 150.0f);
	return w;
}

void printNavigation(navigation_t &n)
{
	ROS_INFO("------- Navigation ---------\n");
	ROS_INFO("Pose[x,y,t] %.2g %.2g %.2g",n.x,n.y,n.theta);
	ROS_INFO("Pose Vel[x,y,t] %.2g %.2g %.2g",n.xdot,n.ydot,n.w);
	ROS_INFO("Gyro Angle: %.2g",n.gtheta);
	ROS_INFO("Distance[?]: %.2g\n",n.distance);
}


void printSensors(sensors_t &s)
{
	ROS_INFO("------- Sensors ---------\n");
	ROS_INFO("Time[s]: %d",s.time/1000);
	ROS_INFO("IR: %d",s.ir);
	ROS_INFO("BATT[V]: %g ",s.batt);
	ROS_INFO("Compass[Deg]: %d",s.compass/10);
	ROS_INFO("Accel[g]: x %.3g y %.3g z %.3g",s.x, s.y, s.z);
	ROS_INFO("mag accel: %g",sqrt(s.x*s.x+s.y*s.y+s.z*s.z));
	ROS_INFO("Gyro[deg/sec]: w %g  V_ref[V] %g",s.w, s.v_ref);
	ROS_INFO("Encoders: left %d  right %d\n",s.en0, s.en1);
	//ROS_INFO("Distance[?]: %g\n",s.distance);
	//ROS_INFO("Angle[deg]: %g\n",s.angle);
}

void getSensors(sensors_t &sensors)
{
	unsigned char msg[128];
	int numbytes = 0;
	char cmd[4];
	cmd[0] = '<';
	cmd[2] = '>';
	cmd[3] = '\0';
	
	cmd[1] = CMD_ALL_SENSORS;
	
	//port.clear();	
	serial_port->write(cmd,3);
	
	//dataLog.pushMsg((const char*) cmd);
	//dataLog.pushMsg("<d>");
	//printf("<d>\n");
	
	memset(msg,0,128*sizeof(unsigned char)); // clear buffer
	
	usleep(10000);
	
	std::string buffer;
	serial_port->readBetween(&buffer,'<','>');
	
	numbytes = buffer.size();
	memcpy(msg,buffer.c_str(),numbytes);
	
	if(numbytes == 25){ // got message
							  // Parse data
							  // msg[0] = '<' and msg[24] = '>'
		sensors.ir = msg[1];
		sensors.compass = (msg[2]<<8 | msg[3]);
		sensors.x = EARTH_GRAVITY*toG(msg[4]<<8 | msg[5]);
		sensors.y = EARTH_GRAVITY*toG(msg[6]<<8 | msg[7]);
		sensors.z = EARTH_GRAVITY*toG(msg[8]<<8 | msg[9]);
		sensors.w = toW(msg[10]<<8 | msg[11],(msg[12]<<8 | msg[13]));
		sensors.v_ref = toV(2*(msg[12]<<8 | msg[13]));
		sensors.batt = toV(2*(msg[14]<<8 | msg[15]));
		sensors.en0 += long(msg[16]<<8 | msg[17]);
		sensors.en1 += long(msg[18]<<8 | msg[19]);

		// swap time
		sensors.last_time = sensors.time;
		sensors.time = (msg[20]<<24 | msg[21]<<16 | msg[22]<<8 | msg[23]);

		/*
		// normalize gravity
		float dg = sqrt(sensors.x*sensors.x+sensors.y*sensors.y+sensors.z*sensors.z);
		sensors.x /= dg;
		sensors.y /= dg;
		sensors.z /= dg;
		*/

		printSensors(sensors);
	}
	else if(numbytes < 0) ROS_ERROR("Couldn't read serial port");
	else {
		ROS_ERROR("Bad data %d %s\n",numbytes,msg);
	}
}

/**
 * Calculates the robot pose based on the encoders
 */
void navigation(sensors_t &sensors, navigation_t &nav)
{
	// grab the current time and calculate the period since last call
	float dt = (float)(sensors.time-sensors.last_time)/1000.0; // seconds
	float l = float(sensors.en0);
	float r = float(sensors.en1);

	// handle division by zero
	if(dt < 0.00000000001f) dt = 0.0001f;
	
	float deltaDistance = (l + r)*DISTANCE_PER_CNT/2.0f;
	
	float deltaHeading = (r - l)*RADIANS_PER_CNT;
	nav.theta += deltaHeading;
	
	if (nav.theta > M_PI) nav.theta -= TWO_PI;
	else if(nav.theta <= -M_PI) nav.theta += TWO_PI;
	
	float dx = deltaDistance * cos(nav.theta);
	float dy = deltaDistance * sin(nav.theta);
	
	nav.x += dx;
	nav.y += dy;
	
	nav.xdot = dx/dt;
	nav.ydot = dy/dt;

	nav.distance += deltaDistance;

	nav.gtheta += sensors.w/dt; // integrate gyro to get heading
	
	printNavigation(nav);
}


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	// setup data packet
	char data[3] = {'<',' ','>'};
	
	// determine direction
	// \todo change this to handle more directions!! [20101228]
	if(cmd_vel->linear.x > .1) data[1] = 'f';
	else if(cmd_vel->linear.x < -.1) data[1] = 'z';
	else if(cmd_vel->angular.z > .1) data[1] = 'l';
	else if(cmd_vel->angular.z < -.1) data[1] = 'r';
	else data[1] = 'h';
	
	// send command
	serial_port->write(data,3);
	
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "scout");
	int baud = 57600;
	std::string port_name = "/dev/ttyUSB0";
	
	ros::NodeHandle n;
	ros::Rate r(10.0);
	//n.param<std::string>("scout/port", port_name, "/dev/cu.usbserial-A7004mS2");
	n.param<std::string>("scout/port", port_name, "/dev/ttyUSB0");
	
	///////////////////////////////////////////////
	serial_port = new cereal::CerealPort();
	int ret = openSerialPort(serial_port,port_name,baud);
	
	if(ret < 0){
		ROS_FATAL("Couldn't open serial port [%s]",port_name.c_str());
		ROS_BREAK();
	}
	else ROS_INFO("Opened serial port [%s]",port_name.c_str());
	
	// Publish info for other nodes
	//ROS_INFO("... Pub ..");
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Publisher battery_pub = n.advertise<scout::Battery>("/battery", 50);
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 50);
	ros::Publisher irbumper_pub = n.advertise<scout::IR>("/ir_bumper", 50);
	
	tf::TransformBroadcaster tf_broadcaster;
	
	//ROS_INFO("... Sub ...");  
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);
	
	
	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;
	
	ROS_INFO("... Start ...");
	
	serial_port->write("<s>",3);
	
	sensors_t sensors;
	navigation_t nav;

	memset(&sensors,0,sizeof(sensors_t));
	memset(&nav,0,sizeof(navigation_t));
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	while (ros::ok())
	{
		// Main loop functions
		//ROS_INFO("... Loop ...");
		getSensors(sensors);
		navigation(sensors, nav);
		
		// publish
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time::now();
		
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "scout";
		marker.id = 0;
		
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;
		
		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = double(dist)/100.0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.01; // 10 cm
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		
		// Set the color -- be sure to set alpha to something non-zero!
		if(irObj){
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
		}
		else{
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
		}
		marker.color.b = 0.0f;
		marker.color.a = 0.5f;
		
		marker.lifetime = ros::Duration();
		
		// Publish the marker
		marker_pub.publish(marker);
		
		//////////////////////////////////////////////////////////////////////////
		
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = nav.x;
		odom_trans.transform.translation.y = nav.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(nav.theta);
		tf_broadcaster.sendTransform(odom_trans);
		
		//TODO: Finish brodcasting the tf for all the ir sensors on the Roomba
		/*geometry_msgs::TransformStamped cliff_left_trans;
		 cliff_left_trans.header.stamp = current_time;
		 cliff_left_trans.header.frame_id = "base_link";
		 cliff_left_trans.child_frame_id = "base_cliff_left";
		 cliff_left_trans.transform.translation.x = 0.0;
		 cliff_left_trans.transform.translation.y = 0.0;
		 cliff_left_trans.transform.translation.z = 0.0;
		 cliff_left_trans.transform.rotation = ;
		 tf_broadcaster.sendTransform(cliff_left_trans);	*/
		
		// ******************************************************************************************
		// next, we'll publish the odometry message over ROS
		// these are based off odometry readings only
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		
		//set the position
		odom.pose.pose.position.x = nav.x; // encoders
		odom.pose.pose.position.y = nav.y; // encoders
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(nav.theta); // encoders
		odom.pose.covariance[0] = 0.1;
		odom.pose.covariance[8] = 0.1;
		odom.pose.covariance[14] = 0.1;
		odom.pose.covariance[20] = 0.1;
		odom.pose.covariance[26] = 0.1;
		odom.pose.covariance[32] = 0.1;

		//set the velocity
		odom.twist.twist.linear.x = nav.xdot; // encoders
		odom.twist.twist.linear.y = nav.ydot; // encoders
		odom.twist.twist.angular.z = nav.w; // ?
		odom.twist.covariance[0] = 0.1;
		odom.twist.covariance[8] = 0.1;
		odom.twist.covariance[14] = 0.1;
		odom.twist.covariance[20] = 0.1;
		odom.twist.covariance[26] = 0.1;
		odom.twist.covariance[32] = 0.1;
		
		//publish the message
		odom_pub.publish(odom);
		
		// ******************************************************************************************
		//publish battery
		scout::Battery battery;
		battery.header.stamp = current_time;
		//battery.power_cord = roomba->power_cord_;
		//battery.dock = roomba->dock_;
		battery.level = 100.0*(sensors.batt/BATTERY_VOLTAGE);
		//if(last_charge > roomba->charge_) time_remaining = (int)(battery.level/((last_charge-roomba->charge_)/roomba->capacity_)/dt)/60;
		//last_charge = roomba->charge_;
		//battery.time_remaining = time_remaining;
		battery.current = sensors.batt;
		battery_pub.publish(battery);
		
		// ******************************************************************************************
		//publish irbumper
		scout::IR irbumper;
		irbumper.header.stamp = current_time;
		irbumper.header.frame_id = "irbumper";
		
		irbumper.ir0 = (sensors.ir & BIT_1);
		irbumper.ir1 = (sensors.ir & BIT_2);
		irbumper.ir2 = (sensors.ir & BIT_3);
		irbumper.cliff0 = (sensors.ir & BIT_4);
		irbumper.cliff1 = (sensors.ir & BIT_5);
		irbumper_pub.publish(irbumper);
		
		// ******************************************************************************************
		//publish irbumper
		sensor_msgs::Imu imu;
		imu.header.stamp = current_time;
		imu.header.frame_id = "imu";
		
		float a = (float) sensors.compass / 10.0f;
		
		//ROS_INFO("compass [%.2f]",a);
		
		// IMU orientation estimate -- fix this
		imu.orientation = tf::createQuaternionMsgFromYaw(a);
		//imu.orientation = odom.pose.pose.orientation; // reuse quaternion calculation
		//imu.orientation_covariance;
		imu.orientation_covariance[0] = 0.01; //xx;
		imu.orientation_covariance[4] = 0.01; //yy;
		imu.orientation_covariance[8] = 0.01; //zz;
		
		// gyro
		imu.angular_velocity.x = 0.0;
		imu.angular_velocity.y = 0.0;
		imu.angular_velocity.z = sensors.w;
		
		// from data sheet
		imu.angular_velocity_covariance[0] = 0.01; //xx;
		imu.angular_velocity_covariance[4] = 0.01; //yy;
		imu.angular_velocity_covariance[8] = 0.01; //zz;
		
		// accel
		imu.linear_acceleration.x = sensors.x;
		imu.linear_acceleration.y = sensors.y;
		imu.linear_acceleration.z = sensors.z;
		
		// from data sheet
		imu.linear_acceleration_covariance[0] = 0.01; //xx;
		imu.linear_acceleration_covariance[4] = 0.01; //yy;
		imu.linear_acceleration_covariance[8] = 0.01; //zz;
		
		imu_pub.publish(imu);
		
		// ******************************************************************************************
		
		ros::spinOnce();
		r.sleep();
	}
	
	closeSerialPort(serial_port);
}


