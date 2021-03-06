#include <ros/ros.h>
#include <ros/network.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include "../include/text_display/main_window.hpp"

namespace text_display {

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
	applyWindowParams();
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
	ui.label->setText(this->init_message.c_str());
	last_msg = ui.label->text().toStdString();
	drawing = false;

	// GUI is now set up and waiting to recieve message signals from QNode
}

MainWindow::~MainWindow() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();

}
void MainWindow::applyWindowParams(){
	ros::NodeHandle n = this->node_handle;

	// SET WINDOW POSITION AND SIZE ----------------------------------------------
	if(n.hasParam("window_width")){
		int width;
		n.getParam("window_width",width);
		QMainWindow::resize(width,QMainWindow::height());
	}
	if(n.hasParam("window_height")){
		int height;
		n.getParam("window_height",height);
		QMainWindow::resize(QMainWindow::width(),height);
	}
	if(n.hasParam("window_x")){
		int x;
		n.getParam("window_x",x);
		QMainWindow::move(x,QMainWindow::y());
	}
	if(n.hasParam("window_y")){
		int y;
		n.getParam("window_y",y);
		QMainWindow::move(QMainWindow::x(),y);
	}

	// SET FONT SIZE -------------------------------------------------------------
	if(n.hasParam("font_size")){
		int size;
		n.getParam("font_size",size);
		QFont labelfont = ui.label->font();
		labelfont.setPointSize(size);
		ui.label->setFont(labelfont);
	}

	// SET COLORS ----------------------------------------------------------------
	// NOTE: The process for setting colors is more complicated because it's defined
	//			 in the widgets stylesheet which is a string containing multiple properties
	// 			 separated by a ';'. So we must:
	// 			 1. Recieve our color and generate our string for the parameter we want set.
	//			 2. Split the existing stylesheet into a list of the property strings.
	//			 3. In that list, find the one we want to set and replace it with the string we generated.
	// 			 4. Recombine the properties into one long string and set the stylesheet of the widget.
	// NOTE: It is likely that there is a better way. (QPallete?)
	// LABEL TEXT COLOR
	if(n.hasParam("text")){
		std::vector<int> color;
		color.resize(3);
		n.getParam("text",color);
		ROS_INFO("text color: %d %d %d",color[0],color[1],color[2]);
		std::stringstream new_color;
		new_color << "color: rgb(" << color[0] << ", " << color[1] << ", " << color[2] << ")";
		QStringList properties = ui.label->styleSheet().split(";");
	  // Find the property we want to change and replace it with our string.
		for(int i = 0 ; i < properties.size() ; i++){
			if(properties.at(i).startsWith("color:")){
				properties.replace(i,new_color.str().c_str());
				break;
			}
		}
		// Recombine properties into QString
		QString newstylesheet;
		for(int i = 0 ; i < properties.size()-1 ; i++){
			newstylesheet.append(properties.at(i));
			newstylesheet.append(";");
		}
		// Set new StyleSheet
		ui.label->setStyleSheet(newstylesheet);
	}

	// LABEL BACKGROUND COLOR
	if(n.hasParam("background")){
		std::vector<int> color;
		color.resize(3);
		n.getParam("background",color);
		ROS_INFO("text color: %d %d %d",color[0],color[1],color[2]);
		std::stringstream new_color;
		new_color << "background-color: rgb(" << color[0] << ", " << color[1] << ", " << color[2] << ")";
		QStringList properties = ui.label->styleSheet().split(";");
		// Find the property we want to change and replace it with our string.
		for(int i = 0 ; i < properties.size() ; i++){
			if(properties.at(i).startsWith("background-color:")){
				properties.replace(i,new_color.str().c_str());
				break;
			}
		}
		// Recombine properties into QString
		QString newstylesheet;
		for(int i = 0 ; i < properties.size()-1 ; i++){
			newstylesheet.append(properties.at(i));
			newstylesheet.append(";");
		}
		// Set new StyleSheet
		ui.label->setStyleSheet(newstylesheet);
	}
	// BORDER COLOR
	// In the case of border, what we're actually doing is setting the background color
	// of the centralwidget that contains our label, and luckily in this case it's
	// stylesheet only has one property so theres no need to search.
	if(n.hasParam("border")){
		std::vector<int> color;
		color.resize(3);
		n.getParam("border",color);
		std::stringstream new_color;
		new_color << "background-color: rgb(" << color[0] << ", " << color[1] << ", " << color[2] << ")";
		// Set new StyleSheet
		ui.centralwidget->setStyleSheet(QString::fromUtf8(new_color.str().c_str()));
	}
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
	// Don't allow message drawing to be interrupted.
	if (drawing){
		ROS_INFO("MESSAGE RECIEVED WHILE DRAWING, IGNORING.\n");
		return;
	}
	drawing = true;
	char cursor = '_';
	std::string fancy = "";
	std::string message = msg.toStdString(); // Message recieved

	ROS_INFO("MAIN WINDOW RECIEVED: %s",message.c_str());
 
	if(this->fancy_draw){
		//If recieved message is the same as the last one, ignore it (don't redraw).
		ROS_INFO("LAST MESSAGE: %s",last_msg.c_str());
		if(last_msg == message){
			ROS_INFO("DUPLICATE MESSAGE SENT, IGNORING.");
			drawing = false;
			return;
		}

		//If label is not currently empty, "destruct" the message.
		if(last_msg != ""){
			ROS_INFO("DESTRUCTING MESSAGE");
			std::string tmp_msg;
			tmp_msg = last_msg.substr(0,last_msg.length()-1);
			for(int i = tmp_msg.length()-2 ; i > 0 ; i--){
				ui.label->setText((tmp_msg+cursor).c_str());
				tmp_msg = last_msg.substr(0,i);
				delay(this->draw_delay);
			}
		}

		//If the new message is empty, set last_msg and return.
		if(message == ""){
			last_msg = "";
			ui.label->setText("_");
			delay(this->draw_delay*10);
			ui.label->setText("");
			drawing = false;
			return;
		}

		//Then "construct" the new message.
		ROS_INFO("CONSTRUCTING MESSAGE");
		ui.label->setText("_");
		delay(this->draw_delay*5);
		for(int i = 0 ; i < message.length()-1 ; i++){
			fancy+=message[i];
			ui.label->setText((fancy+cursor).c_str());
			delay(this->draw_delay);
		}
		ui.label->setText(message.c_str());
		last_msg = message;
		QCoreApplication::processEvents();
		ROS_INFO("LAST_MSG SET TO: %s",last_msg.c_str());
	}else{
		//Very boring
		ui.label->setText(message.c_str());
	}
	drawing = false;
	return;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace text_display
