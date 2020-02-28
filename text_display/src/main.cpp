#include <ros/ros.h>
#include <QtGui>
#include <QApplication>
#include "../include/text_display/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    //ROS
    ros::init(argc,argv,"text_display");
    ros::NodeHandle n;
    if ( ! ros::master::check() ) {
      ROS_ERROR("NO MASTER FOUND.");
  		return false;
  	}

    //QT
    QApplication app(argc, argv);
    text_display::MainWindow w(argc,argv,n);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
