#ifndef text_display_MAIN_WINDOW_H
#define text_display_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <std_msgs/String.h>

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace text_display {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle(), QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void applyWindowParams();

public Q_SLOTS:
  void new_message(QString msg);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	ros::NodeHandle node_handle;

	std::string init_message;
	bool fancy_draw;
	int draw_delay;

	std::string message;
	std::string last_msg;
	bool drawing;

};

}  // namespace text_display

#endif // text_display_MAIN_WINDOW_H
