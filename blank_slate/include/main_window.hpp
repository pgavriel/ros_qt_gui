#ifndef blank_slate_gui_MAIN_WINDOW_H
#define blank_slate_gui_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <std_msgs/String.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace blank_slate_gui {

// Qt central, all operations relating to the GUI are here.
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle(), QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


public Q_SLOTS:
	//Write SLOTS here.

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	ros::NodeHandle node_handle;


};

}

#endif
