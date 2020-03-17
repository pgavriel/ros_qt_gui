#include <ros/ros.h>
#include <ros/network.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include "../include/main_window.hpp"

namespace video_streamer {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, ros::NodeHandle n, QWidget *parent)
	: QMainWindow(parent) // Instantiate QMainWindow
	, qnode(argc,argv,n)  // Instantiate QNode (with NodeHandle)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.png"));

	// Connect QNode's message signal the main_windows new_message SLOT
	QObject::connect(&qnode, SIGNAL(message_recieved(QString)), this, SLOT(new_message(QString)));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	if ( !qnode.init() ) {
		showNoMasterMessage();
	}

	// Set up GUI based on launch parameters.
	this->node_handle = n;

	ros::Duration(0.5).sleep();

	// Defaults
	this->fancy_draw = true;
	this->draw_delay = 20;
	this->init_message = "Welcome.";

	n.getParam(ros::this_node::getName()+"/fancy",this->fancy_draw);
	n.getParam(ros::this_node::getName()+"/draw_delay",this->draw_delay);
	n.getParam(ros::this_node::getName()+"/init_message",this->init_message);
	ROS_INFO("\n[FROM MAIN WINDOW]\nInitMessage:\t%s\nFancyDraw:\t%s\nDrawDelay:\t%d",
						this->init_message.c_str(),(this->fancy_draw?"ENABLED":"DISABLED"),this->draw_delay);

	// GUI is now set up and waiting to recieve message signals from QNode
}

MainWindow::~MainWindow() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();

}

//Function used for creating delays in ms.
void delay(int ms)
{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  close();
}

void MainWindow::new_message(QString msg) {

	return;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace text_display
