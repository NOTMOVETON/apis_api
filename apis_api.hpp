#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 



ros::Publisher local_pos_pub;
ros::Publisher global_lla_pos_pub;
ros::Publisher global_lla_pos_pub_raw;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient auto_waypoint_pull_client;
ros::ServiceClient auto_waypoint_push_client;
ros::ServiceClient auto_waypoint_set_current_client;

struct apis_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}
geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  geometry_msgs::Point current_pos_local;
  current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
  current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;
}

void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  enu_2_local(current_pose_g);
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  current_heading_g = psi*(180/M_PI) - local_offset_g;
}

geometry_msgs::Point get_current_location()
{
	geometry_msgs::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose_g);
	return current_pos_local;
}

float get_current_heading()
{
	return current_heading_g;
}

void set_heading(float heading)
{
  local_desired_heading_g = heading; 
  heading = heading + correction_heading_g + local_offset_g;
  
  ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}

void set_destination(float x, float y, float z, float psi)
{
	set_heading(psi);
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;

	local_pos_pub.publish(waypoint_g);	
}

void set_destination_lla(float lat, float lon, float alt, float heading)
{
	geographic_msgs::GeoPoseStamped lla_msg;
	// mavros_msgs::GlobalPositionTarget 
	lla_msg.pose.position.latitude = lat;
	lla_msg.pose.position.longitude = lon;
	lla_msg.pose.position.altitude = alt;
	float yaw = heading*(M_PI/180);
	float pitch = 0;
	float roll = 0;

	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);

	float qw = cy * cr * cp + sy * sr * sp;
	float qx = cy * sr * cp - sy * cr * sp;
	float qy = cy * cr * sp + sy * sr * cp;
	float qz = sy * cr * cp - cy * sr * sp;

	lla_msg.pose.orientation.w = qw;
	lla_msg.pose.orientation.x = qx;
	lla_msg.pose.orientation.y = qy;
	lla_msg.pose.orientation.z = qz;
	global_lla_pos_pub.publish(lla_msg);
}

void set_destination_lla_raw(float lat, float lon, float alt, float heading)
{
	mavros_msgs::GlobalPositionTarget lla_msg;
	lla_msg.coordinate_frame = lla_msg.FRAME_GLOBAL_TERRAIN_ALT;
	lla_msg.type_mask = lla_msg.IGNORE_VX | lla_msg.IGNORE_VY | lla_msg.IGNORE_VZ | lla_msg.IGNORE_AFX | lla_msg.IGNORE_AFY | lla_msg.IGNORE_AFZ | lla_msg.IGNORE_YAW | lla_msg.IGNORE_YAW_RATE ; 
	lla_msg.latitude = lat;
	lla_msg.longitude = lon;
	lla_msg.altitude = alt;
	// lla_msg.yaw = heading;
	global_lla_pos_pub_raw.publish(lla_msg);
}

int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	while (ros::ok() && !current_state_g.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state_g.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}

int wait4start()
{
	ROS_INFO("Waiting for user to set mode to GUIDED");
	while(ros::ok() && current_state_g.mode != "GUIDED")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state_g.mode == "GUIDED")
	{
		ROS_INFO("Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}

int initialize_local_frame()
{
	ROS_INFO("Initializing local coordinate system");
	local_offset_g = 0;
	for (int i = 1; i <= 30; i++) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		

		float q0 = current_pose_g.pose.pose.orientation.w;
		float q1 = current_pose_g.pose.pose.orientation.x;
		float q2 = current_pose_g.pose.pose.orientation.y;
		float q3 = current_pose_g.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset_g += psi*(180/M_PI);

		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
	}
	local_offset_pose_g.x = local_offset_pose_g.x/30;
	local_offset_pose_g.y = local_offset_pose_g.y/30;
	local_offset_pose_g.z = local_offset_pose_g.z/30;
	local_offset_g /= 30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X' axis is facing: %f", local_offset_g);
	return 0;
}

int arm()
{
	set_destination(0,0,0,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return 0;
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}
}

int takeoff(float takeoff_alt)
{
	set_destination(0,0,takeoff_alt,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}	
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = takeoff_alt;
	if(takeoff_client.call(srv_takeoff)){
		sleep(3);
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}else{
		ROS_ERROR("Failed Takeoff");
		return -2;
	}
	sleep(2);
	return 0; 
}

int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01)
{
	local_pos_pub.publish(waypoint_g);
	
	float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );
    if( dMag < pos_tolerance && headingErr < heading_tolerance)
	{
		return 1;
	}else{
		return 0;
	}
}

int set_mode(std::string mode)
{
	mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode.c_str();
    if(set_mode_client.call(srv_setMode)){
      ROS_INFO("setmode send ok");
	  return 0;
    }else{
      ROS_ERROR("Failed SetMode");
      return -1;
    }
}


int land()
{
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  }else{
    ROS_ERROR("Landing failed");
    return -1;
  }
}

int set_speed(float speed__mps)
{
	mavros_msgs::CommandLong speed_cmd;
	speed_cmd.request.command = 178;
	speed_cmd.request.param1 = 1; // ground speed type 
	speed_cmd.request.param2 = speed__mps;
	speed_cmd.request.param3 = -1; // no throttle change
	speed_cmd.request.param4 = 0; // absolute speed
	ROS_INFO("setting speed to %f", speed__mps);
	if(command_client.call(speed_cmd))
	{
		ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
		return 0;
	}else{
		ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
		ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
		return -1;
	}
	ROS_INFO("change speed result was %d ", speed_cmd.response.result);
	return 0;
}
int auto_set_current_waypoint(int seq)
{
	mavros_msgs::WaypointSetCurrent wp_set_cur_msg;
	wp_set_cur_msg.request.wp_seq = seq;
	ROS_INFO("setting current wp to wp # %d", seq);
	if(auto_waypoint_set_current_client.call(wp_set_cur_msg))
	{
		ROS_INFO("set current wp secceeded %d", wp_set_cur_msg.response.success);
	}else{
		ROS_ERROR("set current wp failed %d", wp_set_cur_msg.response.success);
	}
	return 0;
}

int set_yaw(float angle, float speed, float dir, float absolute_rel)
{
    mavros_msgs::CommandLong yaw_msg;
    yaw_msg.request.command = 115;
    yaw_msg.request.param1 = angle; // target angle 
    yaw_msg.request.param2 = speed; //target speed
    yaw_msg.request.param3 = dir; 
    yaw_msg.request.param4 = absolute_rel; 
    ROS_INFO("Setting the yaw angle to %f [deg]", angle);
    if(command_client.call(yaw_msg))
    {
        ROS_INFO("yaw angle set returned %d", yaw_msg.response.success);
        return 0;
    }else{
        ROS_ERROR("setting yaw angle failed %d", yaw_msg.response.success);
        return -1;
    }
	if(absolute_rel == 0 )
	{
		ROS_INFO("Yaw angle set at %d ", yaw_msg.response.result);
	}else{
		ROS_INFO("Yaw angle set at %d relative to current heading", yaw_msg.response.result);
	}
    
    return 0;
}

int takeoff_global(float lat, float lon, float alt)
{
    mavros_msgs::CommandTOL srv_takeoff_global;
    srv_takeoff_global.request.min_pitch = 0;
    srv_takeoff_global.request.latitude = lat;
    srv_takeoff_global.request.longitude = lon;
    srv_takeoff_global.request.altitude = alt;
        
    if(takeoff_client.call(srv_takeoff_global)){
        sleep(3);
        ROS_INFO("takeoff sent at the provided GPS coordinates %d", srv_takeoff_global.response.success);
    }
    else
    {
        ROS_ERROR("Failed Takeoff %d", srv_takeoff_global.response.success);
        
    }
    sleep(2);
    return 0;
}

int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>(("/mavros/setpoint_position/local"), 10);
	global_lla_pos_pub = controlnode.advertise<geographic_msgs::GeoPoseStamped>(("/mavros/setpoint_position/global"), 10);
	global_lla_pos_pub_raw = controlnode.advertise<mavros_msgs::GlobalPositionTarget>(("/mavros/setpoint_raw/global"), 10);
	currentPos = controlnode.subscribe<nav_msgs::Odometry>(("/mavros/global_position/local"), 10, pose_cb);
	state_sub = controlnode.subscribe<mavros_msgs::State>(("/mavros/state"), 10, state_cb);
	arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>(("/mavros/cmd/arming"));
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>(("/mavros/cmd/land"));
	set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>(("/mavros/set_mode"));
	takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>(("/mavros/cmd/takeoff"));
	command_client = controlnode.serviceClient<mavros_msgs::CommandLong>(("/mavros/cmd/command"));
	auto_waypoint_pull_client = controlnode.serviceClient<mavros_msgs::WaypointPull>(("/mavros/mission/pull"));
	auto_waypoint_push_client = controlnode.serviceClient<mavros_msgs::WaypointPush>(("/mavros/mission/push"));
	auto_waypoint_set_current_client = controlnode.serviceClient<mavros_msgs::WaypointSetCurrent>(("/mavros/mission/set_current"));
	return 0;
}
