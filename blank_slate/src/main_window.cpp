#include <ros/ros.h>
#include <ros/network.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include "../include/main_window.hpp"

namespace blank_slate_gui {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, ros::NodeHandle n, QWidget *parent)
	: QMainWindow(parent) // Instantiate QMainWindow
	, qnode(argc,argv,n)  // Instantiate QNode (with NodeHandle)
{
	ROS_INFO("Setting up GUI...");
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.png"));

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	if ( !qnode.init() ) {
		showNoMasterMessage();
	}

	// Set up ROS parameters here.
	this->node_handle = n;

}

MainWindow::~MainWindow() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();

}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  close();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}
