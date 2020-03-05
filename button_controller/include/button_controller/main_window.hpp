#ifndef button_controller_MAIN_WINDOW_H
#define button_controller_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <std_msgs/String.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace button_controller {

// Qt central, all operations relating to the GUI are here.
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle(), QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	void setupConnections();
	void applyWindowParams();

protected:
	void keyPressEvent(QKeyEvent *keyEvent);

public Q_SLOTS:
	void directionalButton();
	void directionalReleased();
	void gridButton();
	void sliderUpdate(int p);
	void lock();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	ros::NodeHandle node_handle;

	bool send_button_names;
	int sliderValue;


};

}

#endif
