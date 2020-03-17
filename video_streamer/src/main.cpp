#include <ros/ros.h>
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"


int main(int argc, char **argv) {

    //ROS
    //NOTE: This NodeHandle is sent to MainWindow, and subsequently sent to QNode
    ros::init(argc,argv,"video_streamer");
    ros::NodeHandle n;
    if ( ! ros::master::check() ) {
      ROS_ERROR("NO MASTER FOUND.");
  		return false;
  	}

    //QT
    QApplication app(argc, argv);
    video_streamer::MainWindow w(argc,argv,n);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
