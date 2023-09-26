// #include <apis_api.hpp>
// //include API 

// // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
// //     wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
// //     wp.is_current     = true;
// //     wp.autocontinue   = true;
// //     wp.x_lat          = 47.3978206;
// //     wp.y_long         = 8.543987;
// //     wp.z_alt          = 10;
// //     wp_push_srv.request.waypoints.push_back(wp);

// void fly_thr_wp(std::string file_name="/home/dmitriy/Desktop/test2.waypoints", int num=8){

// 	std::vector<std::vector<long double>> message(num, std::vector<long double>(12));
// 	int i=0, j=0;
// 	for(int i=0; i<5; i++){
// 		for(int j=0; j<12; j++){
// 			message[i][j]=0.0;
// 		}
// 	}

// 	mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
//     mavros_msgs::Waypoint wp;


// 	std::ifstream myfile;
// 	myfile.open(file_name);

// 	std::string myline;
// 	if ( myfile.is_open() ) {
// 		while (std::getline (myfile, myline)) {
// 			j=0;
			
// 			std::cout.flush();
// 			std::stringstream ss(myline);  
//     		std::string word;
// 			while (ss >> word) { // Extract word from the stream.
//         		if(isFloatNumber(word)){
// 					message[i][j]=std::stod(word);
// 				}
// 				std::cout.flush();
// 				// std::cout<<i<<" "<<message[i][j]<<" "<<myfile.good()<<std::endl;
// 				j++;
				
//     		}
// 			i++;
// 		}
// 	}
// 	myfile.close();
// 	ROS_INFO("starting mission");
// 	for(i=2; i<num; i++){
// 		ROS_INFO("0");
// 		wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
// 		if(message[i][3]==22){
// 			ROS_INFO("1");
// 			wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
// 			wp.is_current = true;
//     		wp.autocontinue = true;
// 			wp.x_lat = message[i][8];
//     		wp.y_long = message[i][9];
//     		wp.z_alt = message[i][10];
// 			wp_push_srv.request.waypoints.push_back(wp);

// 		} else if(message[i][3]==16){
// 			ROS_INFO("2");
// 			wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
// 			wp.is_current = false;
//     		wp.autocontinue = true;
// 			wp.x_lat = message[i][8];
//     		wp.y_long = message[i][9];
//     		wp.z_alt = message[i][10];
// 			wp_push_srv.request.waypoints.push_back(wp);

// 		} else if(message[i][3]==20){
// 			ROS_INFO("3");
// 			wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
// 			wp.is_current = false;
//     		wp.autocontinue = true;
// 			wp.x_lat = message[i][8];
//     		wp.y_long = message[i][9];
//     		wp.z_alt = message[i][10];
// 			wp_push_srv.request.waypoints.push_back(wp);

// 		}
// 		ROS_INFO("4");
// 	}
// 	ros::Rate rate(20.0);
// 	ROS_INFO("starting mission1");
	 
// 	while(ros::ok()){
// 	if (auto_waypoint_push_client.call(wp_push_srv)) {
//         ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
//     }
//     else
//         ROS_ERROR("Send waypoints FAILED.");

    
//         ros::spinOnce();
//         rate.sleep();
//     }
// }

// // void fly_thr_wp(std::string file_name="/home/dmitriy/Desktop/test2.waypoints", int num=8){
// // 	std::vector<std::vector<long double>> message(num, std::vector<long double>(12));
// // 	int i=0, j=0;
// // 	for(int i=0; i<5; i++){
// // 		for(int j=0; j<12; j++){
// // 			message[i][j]=0.0;
// // 		}
// // 	}


// // 	std::ifstream myfile;
// // 	myfile.open(file_name);

// // 	std::string myline;
// // 	if ( myfile.is_open() ) {
// // 		while (std::getline (myfile, myline)) {
// // 			j=0;
			
// // 			std::cout.flush();
// // 			std::stringstream ss(myline);  
// //     		std::string word;
// // 			while (ss >> word) { // Extract word from the stream.
// //         		if(isFloatNumber(word)){
// // 					message[i][j]=std::stod(word);
// // 				}
// // 				std::cout.flush();
// // 				// std::cout<<i<<" "<<message[i][j]<<" "<<myfile.good()<<std::endl;
// // 				j++;
				
// //     		}
// // 			i++;
// // 		}
// // 	}
// // 	myfile.close();
// // 	ROS_INFO("starting mission");
// // 	// for(i=2; i<num; i++){
// // 	// 	if(check_waypoint_reached(.3) == 1){
// // 	// 		if(message[i][3]==22){
// // 	// 			takeoff(message[i][10]);
// // 	// 		} else if(message[i][3]==16){
// // 	// 			set_destination_lla_raw(message[i][8],message[i][9],message[i][10],0);
// // 	// 		} else if(message[i][3]==20){
// // 	// 			set_mode("RTL");
// // 	// 		}
// // 	// 	}
// // 	// }
// // 	ros::Rate rate(2.0);
// // 	i=3;

// // 		ROS_INFO("taking off");
// // 		takeoff(message[2][10]);
	

// // 	while(ros::ok())
// // 	{
// // 		ros::spinOnce();
// // 		rate.sleep();
// // 		if(check_waypoint_reached(.3) == 1)
// // 		{
// // 			if(message[i][3]==16){
// // 				ROS_INFO("flying to next wp");
// // 				set_destination_lla_raw(message[i][8],message[i][9],message[i][10],0);
// // 				i++;
// // 			} else if(message[i][3]==20){
// // 				ROS_INFO("RTL");
// // 				set_mode("RTL");
// // 				break;
// //                 ros::shutdown();
// // 			}	

// // 		}	
		
// // 	}
// // }


// int main(int argc, char** argv)
// {
// 	//initialize ros 
// 	ros::init(argc, argv, "gnc_node");
// 	ros::NodeHandle gnc_node("~");
	
// 	//initialize control publisher/subscribers
// 	init_publisher_subscriber(gnc_node);

//   	// wait for FCU connection
// 	wait4connect();

// 	//wait for used to switch to mode GUIDED
// 	wait4start();

// 	//create local reference frame 
// 	initialize_local_frame();
// 	arm();
// 	//request takeoff
// 	// takeoff(3);

// 	// //specify some waypoints 
// 	// std::vector<gnc_api_waypoint> waypointList;
// 	// gnc_api_waypoint nextWayPoint;
// 	// nextWayPoint.x = 0;
// 	// nextWayPoint.y = 0;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = 0;
// 	// waypointList.push_back(nextWayPoint);
// 	// nextWayPoint.x = 5;
// 	// nextWayPoint.y = 0;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = -90;
// 	// waypointList.push_back(nextWayPoint);
// 	// nextWayPoint.x = 5;
// 	// nextWayPoint.y = 5;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = 0;
// 	// waypointList.push_back(nextWayPoint);
// 	// nextWayPoint.x = 0;
// 	// nextWayPoint.y = 5;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = 90;
// 	// waypointList.push_back(nextWayPoint);
// 	// nextWayPoint.x = 0;
// 	// nextWayPoint.y = 0;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = 180;
// 	// waypointList.push_back(nextWayPoint);
// 	// nextWayPoint.x = 0;
// 	// nextWayPoint.y = 0;
// 	// nextWayPoint.z = 3;
// 	// nextWayPoint.psi = 0;
// 	// waypointList.push_back(nextWayPoint);


// 	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	
// 	fly_thr_wp();
// 	return 0;
// }


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <list>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "DisArmed");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_set_mode");
    ros::NodeHandle nh;
    
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    /*
        uint8 FRAME_GLOBAL=0
        uint8 FRAME_LOCAL_NED=1
        uint8 FRAME_MISSION=2
        uint8 FRAME_GLOBAL_REL_ALT=3
        uint8 FRAME_LOCAL_ENU=4
        uint8 frame
        uint16 command
        bool is_current
        bool autocontinue
        float32 param1
        float32 param2
        float32 param3
        float32 param4
        float64 x_lat
        float64 y_long
        float64 z_alt
    */
    // WP 0
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = -35.363262;
    wp.y_long         = 149.165237;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
    // WP 1
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = -35.364462;
    wp.y_long         = 149.166437;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
    
    // WP 2
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = -35.363262;
    wp.y_long         = 149.165237;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);

    // WP 3
    wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");
    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        
    }
    else
        ROS_ERROR("Send waypoints FAILED.");

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}