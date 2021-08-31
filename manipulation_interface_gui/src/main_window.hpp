/**
 * @file /include/manipulation_interface_gui/main_window.hpp
 *
 * @brief Qt based gui for manipulation_interface_gui.
 *
 * @date November 2010
 **/
#ifndef manipulation_interface_gui_MAIN_WINDOW_H
#define manipulation_interface_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <ui_main_window.h>
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace manipulation_interface_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv,
               ros::NodeHandle n,
               ros::NodeHandle nh_i,
               ros::NodeHandle nh_o,
               ros::NodeHandle nh_g,
               QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void add_go_to                      (int state);
    void add_pick                       (int state);
    void add_place                      (int state);
    void on_check_robot_TF_stateChanged (int state);

    void on_actionAbout_triggered();
    void on_button_add_action_clicked                 (bool check);
    void on_button_add_grasp_clicked                  (bool check);
    void on_button_add_approach_slot_clicked          (bool check);
    void on_button_add_final_position_slot_clicked    (bool check);
    void on_button_add_approach_box_clicked           (bool check);
    void on_button_add_final_box_clicked              (bool check);
    void on_button_add_location_clicked               (bool check);
    void on_button_add_recipe_clicked                 (bool check);
    void on_button_add_object_clicked                 (bool check);
    void on_button_add_slot_clicked                   (bool check);
    void on_button_add_box_clicked                    (bool check);
    void on_button_add_location_changes_clicked       (bool check);
    void on_button_add_slot_changes_clicked           (bool check);
    void on_button_add_box_changes_clicked            (bool check);
    void on_button_add_object_changes_clicked         (bool check);
    void on_button_copy_location_clicked              (bool check);
    void on_button_copy_object_clicked                (bool check);
    void on_button_copy_slot_clicked                  (bool check);
    void on_button_copy_box_clicked                   (bool check);
    void on_button_copy_grasp_clicked                 (bool check);
    void on_button_remove_go_to_clicked               (bool check);
    void on_button_remove_location_clicked            (bool check);
    void on_button_remove_place_clicked               (bool check);
    void on_button_remove_pick_clicked                (bool check);
    void on_button_remove_object_clicked              (bool check);
    void on_button_remove_box_clicked                 (bool check);
    void on_button_remove_group_clicked               (bool check);
    void on_button_remove_slot_clicked                (bool check);
    void on_button_remove_grasp_clicked               (bool check);
    void on_button_remove_approach_slot_clicked       (bool check);
    void on_button_remove_final_position_slot_clicked (bool check);
    void on_button_remove_approach_box_clicked        (bool check);
    void on_button_remove_final_box_clicked           (bool check);
    void on_button_remove_recipe_clicked              (bool check);
    void on_button_run_recipe_clicked                 (bool check);
    void on_button_gripper_clicked                    (bool check);
    void on_button_gripper_2_clicked                  (bool check);
    void on_button_save_components_clicked            (bool check);
    void on_button_save_actions_clicked               (bool check);
    void on_button_save_all_changes0_clicked          (bool check);
    void on_button_save_all_changes1_clicked          (bool check);
    void on_button_save_all_changes2_clicked          (bool check);
    void on_button_save_all_changes3_clicked          (bool check);
    void on_button_load_TF_clicked                    (bool check);
    void on_button_load_clicked                       (bool check);
    void on_button_write_recipe_clicked               (bool check);
    void on_button_reset_location_info_clicked        (bool check);
    void on_button_set_approach_clicked               (bool check);
    void on_button_reset_slot_info_clicked            (bool check);
    void on_button_reset_box_info_clicked             (bool check);
    void on_button_reset_object_info_clicked          (bool check);
    void on_button_remove_element_clicked             (bool check);
    void on_button_load_recipe_clicked                (bool check);
    void on_button_load_actions_clicked               (bool check);
    void on_button_save_recipe_clicked                (bool check);
    void on_button_anti_x_pressed   ();
    void on_button_anti_y_pressed   ();
    void on_button_anti_z_pressed   ();
    void on_button_anti_x_released  ();
    void on_button_anti_y_released  ();
    void on_button_anti_z_released  ();
    void on_button_back_pressed     ();
    void on_button_back_released    ();
    void on_button_clock_x_pressed  ();
    void on_button_clock_y_pressed  ();
    void on_button_clock_z_pressed  ();
    void on_button_clock_x_released ();
    void on_button_clock_y_released ();
    void on_button_clock_z_released ();
    void on_button_down_pressed     ();
    void on_button_down_released    ();
    void on_button_up_pressed       ();
    void on_button_left_pressed     ();
    void on_button_right_pressed    ();
    void on_button_front_pressed    ();
    void on_button_up_released      ();
    void on_button_left_released    ();
    void on_button_right_released   ();
    void on_button_front_released   ();
    void reset_location                              (int index);
    void reset_slot                                  (int index);
    void reset_box                                   (int index);
    void reset_object                                (int index);
    void on_robot_list_currentIndexChanged           (int index);
    void on_combo_grasp_number_currentIndexChanged   (int index);
    void on_combo_action_type_currentIndexChanged    (int index);
    void on_combo_configuration_currentIndexChanged  (int index);
    void on_combo_configuration__currentIndexChanged (int index);
    void on_combo_ref_frame_currentIndexChanged      (int index);
    void on_gripper_percentage_valueChanged   (int value);
    void on_gripper_percentage_2_valueChanged (int value);
    void on_velocity_slider_valueChanged      (int value);
    void on_list_location_modify_pressed (const QModelIndex &index);
    void on_list_box_modify_pressed      (const QModelIndex &index);
    void on_list_slot_modify_pressed     (const QModelIndex &index);
    void on_list_object_modify_pressed   (const QModelIndex &index);
    void on_list_place_pressed           (const QModelIndex &index);
    void on_list_pick_pressed            (const QModelIndex &index);
    void on_list_go_to_pressed           (const QModelIndex &index);
    void on_go_to_list_pressed           (const QModelIndex &index);
    void on_place_list_pressed           (const QModelIndex &index);
    void on_pick_list_pressed            (const QModelIndex &index);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    int num_grasp = 0, num_approach = 0, num_picks = 0, num_places = 0;
    std::vector<location>    actual_object_grasp;
    std::vector<position>    actual_object_approach;
    std::vector<std::string> actual_approach_gripper_states;
    std::vector<std::string> actual_tool_approach;
    std::vector<std::string> actual_tool_grasp;
    location                 actual_slot_approach;
    location                 actual_slot_final_position;
    location                 actual_box_approach;
    location                 actual_box_final;
    object_type              actual_object_to_modify;
    std::string              actual_gripper_state = "open";
    bool init_approach_object = false;
    bool init_slot_approach   = false;
    bool init_slot_final      = false;
    bool init_box_approach    = false;
    bool init_box_final       = false;
    bool init_objects         = false;
    float max_vel  = 0.1;
    float max_rot  = 0.7;
    int   perc_vel = 50;
};

}  // namespace manipulation_interface_gui

#endif // manipulation_interface_gui_MAIN_WINDOW_H
