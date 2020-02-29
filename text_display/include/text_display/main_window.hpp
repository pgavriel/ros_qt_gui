#ifndef text_display_MAIN_WINDOW_H
#define text_display_MAIN_WINDOW_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <std_msgs/String.h>

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace text_display {

// Qt central, all operations relating to the GUI are here.
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle(), QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	// Applies GUI parameters described in .yaml file
	void applyWindowParams();

public Q_SLOTS:
	// Recieves a new message from QNode Subscriber callback
  void new_message(QString msg);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	ros::NodeHandle node_handle;

	std::string init_message;
	bool fancy_draw;
	int draw_delay;

	bool drawing;
	std::string message;
	std::string last_msg;

};

}  // namespace text_display

#endif // text_display_MAIN_WINDOW_H
