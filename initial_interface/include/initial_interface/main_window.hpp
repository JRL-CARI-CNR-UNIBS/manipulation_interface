/**
 * @file /include/initial_interface/main_window.hpp
 *
 * @brief Qt based gui for initial_interface.
 *
 * @date November 2010
 **/
#ifndef initial_interface_MAIN_WINDOW_H
#define initial_interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace initial_interface {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    void on_button_add_go_to_clicked(bool check);
    void on_button_add_place_clicked(bool check);
    void on_button_add_pick_clicked(bool check);
    void on_button_add_approach_object_clicked(bool check);
    void on_button_add_grasp_clicked(bool check);
    void on_button_add_approach_slot_clicked(bool check);
    void on_button_add_final_position_slot_clicked(bool check);
    void on_button_add_approach_box_clicked(bool check);
    void on_button_add_final_box_clicked(bool check);
    void on_button_remove_go_to_clicked(bool check);
    void on_button_remove_place_clicked(bool check);
    void on_button_remove_pick_clicked(bool check);
    void on_button_remove_object_clicked(bool check);
    void on_button_remove_box_clicked(bool check);
    void on_button_remove_group_clicked(bool check);
    void on_button_remove_slot_clicked(bool check);
    void on_button_remove_approach_object_clicked(bool check);
    void on_button_remove_grasp_clicked(bool check);
    void on_button_remove_approach_slot_clicked(bool check);
    void on_button_remove_final_position_slot_clicked(bool check);
    void on_button_remove_approach_box_clicked(bool check);
    void on_button_remove_final_box_clicked(bool check);
    void on_button_manual_guidance_clicked(bool check);
    void on_button_gripper_clicked(bool check);
    void on_button_save_clicked(bool check);
    void on_button_save_object_clicked(bool check);
    void on_button_save_slot_clicked(bool check);
    void on_button_save_box_clicked(bool check);
    void on_button_load_TF_clicked(bool chack);
    void on_button_load_clicked(bool chack);
    void on_check_robot_TF_stateChanged(int state);
    void on_check_gripper_stateChanged(int state);
    void on_check_manual_guidance_stateChanged(int state);
    void on_robot_list_currentIndexChanged(int index);
    void on_place_list_pressed(const QModelIndex &index);
    void on_pick_list_pressed(const QModelIndex &index);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    int num_grasp = 0, num_approach = 0, num_picks = 0, num_places = 0;
    std::vector<location> actual_object_grasp;
    std::vector<position> actual_object_approach;
    location actual_slot_approach;
    location actual_slot_final_position;
    location actual_box_approach;
    location actual_box_final;
    bool init_approach_object = false;
    bool init_slot_approach = false;
    bool init_slot_final = false;
    bool init_box_approach = false;
    bool init_box_final = false;
};

}  // namespace initial_interface

#endif // initial_interface_MAIN_WINDOW_H
