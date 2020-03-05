#include <ros/ros.h>
#include <ros/network.h>
#include <QtGui>
#include <QMessageBox>
#include <QShortcut>
#include <iostream>
#include <sstream>
#include "../include/button_controller/main_window.hpp"

namespace button_controller {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, ros::NodeHandle n, QWidget *parent)
	: QMainWindow(parent) // Instantiate QMainWindow
	, qnode(argc,argv,n)  // Instantiate QNode (with NodeHandle)
{
	ROS_INFO("Setting up GUI...");
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	// Focus Policy is important for the function of keyPressEvents
	this->setFocusPolicy ( Qt::StrongFocus );
	setWindowIcon(QIcon(":/images/icon.png"));

	setupConnections();

	QShortcut *shortcut = new QShortcut(QKeySequence("Qt::Up"), parent);
	QObject::connect(shortcut, SIGNAL(activated()), ui.directionalUp, SIGNAL(clicked()));

	if ( !qnode.init() ) {
		showNoMasterMessage();
	}

	// Initialize slider values.
	sliderValue = ui.verticalSlider->value();
	ui.lineEdit_slider->setText(QString::number(sliderValue));

	// Set up ROS parameters here.
	this->node_handle = n;
	applyWindowParams();
	send_button_names = true;
	n.getParam(ros::this_node::getName()+"/send_button_names",send_button_names);




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

void MainWindow::setupConnections(){
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	// Connect Directional Buttons
	QObject::connect(ui.directionalUp,    SIGNAL(pressed()),this, SLOT(directionalButton()));
	QObject::connect(ui.directionalDown,  SIGNAL(pressed()),this, SLOT(directionalButton()));
	QObject::connect(ui.directionalLeft,  SIGNAL(pressed()),this, SLOT(directionalButton()));
	QObject::connect(ui.directionalRight, SIGNAL(pressed()),this, SLOT(directionalButton()));
	QObject::connect(ui.directionalUp,    SIGNAL(released()),this, SLOT(directionalReleased()));
	QObject::connect(ui.directionalDown,  SIGNAL(released()),this, SLOT(directionalReleased()));
	QObject::connect(ui.directionalLeft,  SIGNAL(released()),this, SLOT(directionalReleased()));
	QObject::connect(ui.directionalRight, SIGNAL(released()),this, SLOT(directionalReleased()));
	// Connect Grid Buttons
	QObject::connect(ui.grid1,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid2,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid3,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid4,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid5,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid6,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid7,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid8,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid9,  SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid10, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid11, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid12, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid13, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid14, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid15, SIGNAL(clicked()),this, SLOT(gridButton()));
	QObject::connect(ui.grid16, SIGNAL(clicked()),this, SLOT(gridButton()));
	// More connections
	QObject::connect(ui.pushButton_lock, SIGNAL(clicked()),this, SLOT(lock()));
	QObject::connect(ui.verticalSlider, SIGNAL(valueChanged(int)),this, SLOT(sliderUpdate(int)));
}

void MainWindow::keyPressEvent(QKeyEvent *keyEvent)
{		// Implement WASD/Arrow Key controls
		//ui.lineEdit_status->setText(QString("KEY: "+keyEvent->text()));
    switch (keyEvent->key()) {
    case Qt::Key_Down: case Qt::Key_S:
				ui.directionalDown->animateClick();
        break;
    case Qt::Key_Left: case Qt::Key_A:
				ui.directionalLeft->animateClick();
        break;
    case Qt::Key_Right: case Qt::Key_D:
				ui.directionalRight->animateClick();
        break;
    case Qt::Key_Up: case Qt::Key_W:
				ui.directionalUp->animateClick();
        break;
    default:
        break;
    }
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

}

void MainWindow::directionalButton(){
	//Save the text and object name of the button that was pressed to QStrings.
	QPushButton *clickedbutton = qobject_cast<QPushButton *>(sender());
  QString btnText = clickedbutton->text();
	QString btnName = clickedbutton->objectName();
	QString message;

	if(send_button_names){ //Send the buttons text
		message.append(btnText);
	}else{								 //Send a shorthand that is easier to parse
				 if (btnName=="directionalUp") message.append("D1");
		else if (btnName=="directionalRight") message.append("D2");
		else if (btnName=="directionalDown") message.append("D3");
		else if (btnName=="directionalLeft") message.append("D4");
		else message.append("UnknownButton?");
	}
	message.append(" ; ");
	message.append(QString::number(ui.verticalSlider->value()));
	qnode.setMessage(message.toStdString(),"directional");
	ui.lineEdit_status->setText(QString("PUBLISHING: \""+message+"\""));
}
void MainWindow::directionalReleased(){
	qnode.stopPublishing();

}

void MainWindow::gridButton(){
	//Save the text and object name of the button that was pressed to QStrings. 
	QPushButton *clickedbutton = qobject_cast<QPushButton *>(sender());
	QString btnText = clickedbutton->text();
	QString btnName = clickedbutton->objectName();
	QString message;

	if(send_button_names){ //Send the buttons text
		message.append(btnText);
	}else{								//Send a shorthand that is easier to parse
				 if (btnName=="grid1") message.append("G1");
		else if (btnName=="grid2") message.append("G2");
		else if (btnName=="grid3") message.append("G3");
		else if (btnName=="grid4") message.append("G4");
		else if (btnName=="grid5") message.append("G5");
		else if (btnName=="grid6") message.append("G6");
		else if (btnName=="grid7") message.append("G7");
		else if (btnName=="grid8") message.append("G8");
		else if (btnName=="grid9") message.append("G9");
		else if (btnName=="grid10") message.append("G10");
		else if (btnName=="grid11") message.append("G11");
		else if (btnName=="grid12") message.append("G12");
		else if (btnName=="grid13") message.append("G13");
		else if (btnName=="grid14") message.append("G14");
		else if (btnName=="grid15") message.append("G15");
		else if (btnName=="grid16") message.append("G16");
		else message.append("UnknownButton?");
	}
	message.append(" ; ");
	message.append(QString::number(ui.verticalSlider->value()));
	qnode.setMessage(message.toStdString(),"grid");
	ui.lineEdit_status->setText(QString("PUBLISHING: \""+message+"\""));
}

void MainWindow::sliderUpdate(int pos){
	sliderValue = pos;
	ui.lineEdit_slider->setText(QString::number(sliderValue));
}

void MainWindow::lock(){
	ui.directionalUp->setEnabled(!ui.directionalUp->isEnabled());
	ui.directionalDown->setEnabled(!ui.directionalDown->isEnabled());
	ui.directionalLeft->setEnabled(!ui.directionalLeft->isEnabled());
	ui.directionalRight->setEnabled(!ui.directionalRight->isEnabled());

	ui.grid1->setEnabled(!ui.grid1->isEnabled());
	ui.grid2->setEnabled(!ui.grid2->isEnabled());
	ui.grid3->setEnabled(!ui.grid3->isEnabled());
	ui.grid4->setEnabled(!ui.grid4->isEnabled());
	ui.grid5->setEnabled(!ui.grid5->isEnabled());
	ui.grid6->setEnabled(!ui.grid6->isEnabled());
	ui.grid7->setEnabled(!ui.grid7->isEnabled());
	ui.grid8->setEnabled(!ui.grid8->isEnabled());
	ui.grid9->setEnabled(!ui.grid9->isEnabled());
	ui.grid10->setEnabled(!ui.grid10->isEnabled());
	ui.grid11->setEnabled(!ui.grid11->isEnabled());
	ui.grid12->setEnabled(!ui.grid12->isEnabled());
	ui.grid13->setEnabled(!ui.grid13->isEnabled());
	ui.grid14->setEnabled(!ui.grid14->isEnabled());
	ui.grid15->setEnabled(!ui.grid15->isEnabled());
	ui.grid16->setEnabled(!ui.grid16->isEnabled());

	ui.verticalSlider->setEnabled(!ui.verticalSlider->isEnabled());

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}
