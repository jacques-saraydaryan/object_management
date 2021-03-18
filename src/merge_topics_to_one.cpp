/*
 * 
 *      Author: jsaraydaryan
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <math.h> 
#include <cstdlib>
#include <string>
#include <exception>
#include "mutex"
#include <thread>

// Publisher used to send merged images
image_transport::Publisher merged_topic_pub;
image_transport::Subscriber sub_topic_1;
image_transport::Subscriber sub_topic_2;
image_transport::Subscriber sub_topic_3;

int PUBLISH_FREQUENCY=10;
float SWITCH_PERIOD = 0.1;
std::mutex current_img_mtx;
sensor_msgs::Image::ConstPtr current_img= NULL;

std::mutex current_img_src_mtx;
int current_img_src=0;
std::vector<int>  img_source_list;

//FIXME need to add a lock due to multithreading
int current_img_source_index=0;

int cameraQueueSize;

float last_time_in_sec=0;
std::map<int, std::string> topic_map; 
std::map<int, image_transport::Subscriber> subscriber_map; 


sensor_msgs::Image::ConstPtr get_current_img(){
	std::lock_guard<std::mutex> mtx2_lock(current_img_mtx);
    {
		return current_img;
	}
}

void set_current_img(sensor_msgs::Image::ConstPtr img){
	std::lock_guard<std::mutex> mtx2_lock(current_img_mtx);
    {
		current_img=img;
	}
}


int get_current_img_src(){
    std::lock_guard<std::mutex> mtx1_lock(current_img_src_mtx);
    {
		return current_img_src;
    }
}

void set_current_img_src(int value){
	std::lock_guard<std::mutex> mtx1_lock(current_img_src_mtx);
    {
			current_img_src=value;
	}
}

bool is_same_last_img(){
	try{
		if(current_img != NULL){
			float current_img_time_sec=get_current_img()->header.stamp.toSec();
			if(abs(last_time_in_sec - current_img_time_sec)< 0.001){
				ROS_DEBUG("diff time last:[%f], current[%f], diff:[%f] \n",last_time_in_sec,current_img_time_sec,last_time_in_sec - current_img_time_sec);
				return true;
			}

			last_time_in_sec= current_img_time_sec;
		}
		return false;
	}catch(...){
		return false;
	}
}

void switch_source(){
	current_img_source_index++;
	if (current_img_source_index >=img_source_list.size() ){
		current_img_source_index=0;
	}
	//ROS_DEBUG("current img index %i",current_img_source_index );
	set_current_img_src(img_source_list[current_img_source_index]);
}

void republish_image(const sensor_msgs::ImageConstPtr& msg){
	merged_topic_pub.publish(msg);
}

void img1Callback(const sensor_msgs::ImageConstPtr& msg, int i){
	if(get_current_img_src() == i){
		ROS_DEBUG("value from /image %i",i);
		set_current_img(msg);
		
	}
}

//void img2Callback(const sensor_msgs::ImageConstPtr& msg){
//	
//	if(get_current_img_src() == 1){
//		ROS_DEBUG("value from /image2");
//		set_current_img(msg);
//	}
//}
//
//void img3Callback(const sensor_msgs::ImageConstPtr& msg){
//	if(get_current_img_src() == 2){
//		ROS_DEBUG("value from /image3");
//		set_current_img(msg);
//	}
//}

void redirect_images(){

	ros::Rate rate(PUBLISH_FREQUENCY);
	ros::Time last_switch = ros::Time::now();

	ROS_INFO("before loop");

	while(ros::ok()){
		
		if(!is_same_last_img()){
			merged_topic_pub.publish(get_current_img());
		}
		//ros::spinOnce();
		rate.sleep();
		if(ros::Time::now().toSec()- last_switch.toSec() > SWITCH_PERIOD){
			switch_source();
			last_switch = ros::Time::now();
		}
	}
}




/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"merge_topics_to_one");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    
    //ROS_INFO("Param values:");
//
	//if (!ros::param::get("/convert_2d_to_3d/display_marker", display_marker))
	//	{
	//		display_marker=false;
	//	}
	//ROS_INFO("- /convert_2d_to_3d/display_marker: %i",display_marker);


	

	// Publisher
    //merged_topic_pub = it.advertise("/image_merged", 1);
	//Set of image topic sources
	//sub_topic_1= it.subscribe("/image1", 1, img1Callback);
	//sub_topic_2= it.subscribe("/image2", 1, img2Callback);
	//sub_topic_3= it.subscribe("/image3", 1, img3Callback);


	if (!ros::param::get("/object_management/publish_frequency", PUBLISH_FREQUENCY))
	    {
	      PUBLISH_FREQUENCY=10;
	    }
	ROS_INFO("- /object_management/publish_frequency: %i", PUBLISH_FREQUENCY);

		if (!ros::param::get("/object_management/switch_period", SWITCH_PERIOD))
	    {
	      SWITCH_PERIOD=0.1;
	    }

	ROS_INFO("- /object_management/switch_period: %f", SWITCH_PERIOD);


	std::vector<std::string> topic_list;
	if (!ros::param::get("/object_management/img_topic_sources", topic_list))
	    {
	      topic_list.push_back("/kinect/color/image_raw");
		  topic_list.push_back("/camera/color/image_raw");
	    }


	merged_topic_pub = it.advertise("/darknet/input", 1);


	int i=0;
	for(const auto& value: topic_list) {
		topic_map.insert(std::pair<int, std::string>(i, value)); 
		img_source_list.push_back(i);
		subscriber_map.insert((std::pair<int, image_transport::Subscriber>(i,it.subscribe(value, 1, boost::bind(img1Callback, _1, i)))));
		i++;
		ROS_INFO("- /object_management/img_topic_sources: %s", value.c_str());
	}  
	
	//sub_topic_2= it.subscribe("//camera/color/image_raw", 1, img2Callback);
	//sub_topic_3= it.subscribe("/image3", 1, img3Callback);

	
    std::thread redirect_thread (redirect_images); 

   	ROS_INFO("Ready to merge topic images sources ");

	//redirect_images();
    ros::spin();

}
