/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QWidget>
#include <QApplication>
#include "main_window.hpp"
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

    ros::init(argc,argv,"manipulation_interface_gui");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    ros::NodeHandle n;
    ros::NodeHandle nh_i("inbound_pick_server");
    ros::NodeHandle nh_o("outbound_place_server");
    ros::NodeHandle nh_g("go_to_location_server");

    manipulation_interface_gui::MainWindow w(argc,argv,n,nh_i,nh_o,nh_g);

    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    ROS_INFO("Start gui");
    int result = app.exec();
    ROS_INFO("Stop gui");

	return result;
}
