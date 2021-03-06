#ifndef blank_slate_gui_QNODE_HPP_
#define blank_slate_gui_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <std_msgs/String.h>
#include <QThread>
#include <QStringListModel>

namespace blank_slate_gui {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv, ros::NodeHandle n = ros::NodeHandle());
	virtual ~QNode();
	bool init();
	void run();

	// Logging
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

   QStringListModel* loggingModel() { return &logging_model; }
   void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();


private:
	int init_argc;
	char** init_argv;
  QStringListModel logging_model;
  ros::NodeHandle node_handle;

};

}

#endif
