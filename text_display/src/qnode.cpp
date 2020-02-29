#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/text_display/qnode.hpp"

namespace text_display {

QNode::QNode(int argc, char** argv, ros::NodeHandle n) :
	init_argc(argc),
	init_argv(argv)
	{
		// Recieves NodeHandle from main
		this->node_handle = n;
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
	if ( ! ros::master::check() ) {
		return false;
	}

	// Recieve topic parameter
	ros::NodeHandle n = this->node_handle;
	topic = "text_display";
	n.getParam(ros::this_node::getName()+"/topic",topic);
	ROS_INFO("\n[FROM QNODE]\nTopic:\t\t%s", topic.c_str());
	message = "";

	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	// Start QNode thread run()
	start();
	return true;
}


void QNode::displayCallback(const std_msgs::String::ConstPtr& msg){
	// Recieve message from Subscriber, and emit signal to send message to main_window
	message = msg->data.c_str();
	Q_EMIT message_recieved(QString(message.c_str()));
	log(Info,std::string("QNODE CALLBACK REACHED: ")+message.c_str());
}


void QNode::run() {
	// Setup Subscriber and start listening
	ros::NodeHandle n = this->node_handle;
	topic_listener = n.subscribe(topic, 1000, &QNode::displayCallback, this);
	ros::Rate loop_rate(10);
	while ( ros::ok() ) {
		ros::spin();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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

}  // namespace text_display
