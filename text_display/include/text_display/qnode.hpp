/**
 * @file /include/participant_display/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef text_display_QNODE_HPP_
#define text_display_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>

#include <std_msgs/String.h>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace text_display {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle());
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

   QStringListModel* loggingModel() { return &logging_model; }
   void log( const LogLevel &level, const std::string &msg);
   void displayCallback(const std_msgs::String::ConstPtr& msg);


Q_SIGNALS:
	void loggingUpdated();
  void message_recieved(QString msg);
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Subscriber topic_listener;
  QStringListModel logging_model;
  std::string message;
  std::string topic;
  ros::NodeHandle node_handle;

};

}  // namespace text_display

#endif /* text_display_QNODE_HPP_ */
