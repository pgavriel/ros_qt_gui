#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include "../include/button_controller/qnode.hpp"

namespace button_controller {

QNode::QNode(int argc, char** argv, ros::NodeHandle n) :
	init_argc(argc),
	init_argv(argv)
	{
		// Recieves NodeHandle from main
		this->node_handle = n;
		message = "";
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


bool QNode::init() {
	// Check for ros master
	if ( ! ros::master::check() ) { return false; }

	//Setup ROS parameters and publishers here.
	ros::NodeHandle n = this->node_handle;
	std::string topic = "button_control";
	loopRate = 5;
	n.getParam(ros::this_node::getName()+"/topic",topic);
	n.getParam(ros::this_node::getName()+"/loop_rate",loopRate);
	ROS_INFO("\n[FROM QNODE]\nTopic:\t\t%s\nLoop Rate:\t%d", topic.c_str(),loopRate);

	// explicitly needed since our nodehandle is going out of scope.
	ros::start();
	publisher = n.advertise<std_msgs::String>(topic, 1000);

	// Start QNode thread run()
	ROS_INFO("Starting QNode...");
	start();
	return true;
}


void QNode::run() {
	//Setup subscribers here.
	ros::Rate loop_rate(loopRate);
	std_msgs::String msg;
	while ( ros::ok() ) {
		if(newMessage){
			msg.data = message;
			publisher.publish(msg);
			if(message_type != "directional") newMessage = false;
			loop_rate.sleep();
		}
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setMessage(std::string msg, std::string type){
	message = msg;
	message_type = type;
	newMessage = true;
}
void QNode::stopPublishing(){
	newMessage = false;
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}
