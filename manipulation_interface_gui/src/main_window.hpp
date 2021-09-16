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
    void addGoTo                      (int state);
    void addPick                      (int state);
    void addPlace                     (int state);
    void on_checkRobotTF_stateChanged (int state);
    void plotMsg (std::string msg);

    void on_actionAbout_triggered();
    void on_buttonAddAction_clicked               (bool check);
    void on_buttonAddGrasp_clicked                (bool check);
    void on_buttonAddApproachSlot_clicked         (bool check);
    void on_buttonAddFinalPositionSlot_clicked    (bool check);
    void on_buttonAddApproachBox_clicked          (bool check);
    void on_buttonAddFinalBox_clicked             (bool check);
    void on_buttonAddLocation_clicked             (bool check);
    void on_buttonAddRecipe_clicked               (bool check);
    void on_buttonAddObject_clicked               (bool check);
    void on_buttonAddSlot_clicked                 (bool check);
    void on_buttonAddBox_clicked                  (bool check);
    void on_buttonAddLocationChanges_clicked      (bool check);
    void on_buttonAddSlotChanges_clicked          (bool check);
    void on_buttonAddBoxChanges_clicked          (bool check);
    void on_buttonAddObjectChanges_clicked        (bool check);
    void on_buttonAddLeavePositionSlot_clicked    (bool check);
    void on_buttonAddLeavePositionBox_clicked     (bool check);
    void on_buttonCopyLocation_clicked            (bool check);
    void on_buttonCopyObject_clicked              (bool check);
    void on_buttonCopySlot_clicked                (bool check);
    void on_buttonCopyBox_clicked                 (bool check);
    void on_buttonCopyGrasp_clicked               (bool check);
    void on_buttonRemoveGoTo_clicked              (bool check);
    void on_buttonRemoveLocation_clicked          (bool check);
    void on_buttonRemovePlace_clicked             (bool check);
    void on_buttonRemovePick_clicked              (bool check);
    void on_buttonRemoveObject_clicked            (bool check);
    void on_buttonRemoveBox_clicked               (bool check);
    void on_buttonRemoveGroup_clicked             (bool check);
    void on_buttonRemoveSlot_clicked              (bool check);
    void on_buttonRemoveGrasp_clicked             (bool check);
    void on_buttonRemoveApproachSlot_clicked      (bool check);
    void on_buttonRemoveFinalPositionSlot_clicked (bool check);
    void on_buttonRemoveApproachBox_clicked       (bool check);
    void on_buttonRemoveFinalBox_clicked          (bool check);
    void on_buttonRemoveRecipe_clicked            (bool check);
    void on_buttonRemoveLeavePositionSlot_clicked (bool check);
    void on_buttonRemoveLeavePositionBox_clicked  (bool check);
    void on_buttonRunRecipe_clicked               (bool check);
    void on_buttonGripper_clicked                 (bool check);
    void on_buttonGripper2_clicked                (bool check);
    void on_buttonLoadTF_clicked                  (bool check);
    void on_buttonLoad_clicked                    (bool check);
    void on_buttonWriteRecipe_clicked             (bool check);
    void on_buttonResetLocationInfo_clicked       (bool check);
    void on_buttonSetApproach_clicked             (bool check);
    void on_buttonSetLeave_clicked                (bool check);
    void on_buttonResetSlotInfo_clicked           (bool check);
    void on_buttonResetBoxInfo_clicked            (bool check);
    void on_buttonResetObjectInfo_clicked         (bool check);
    void on_buttonRemoveElement_clicked           (bool check);
    void on_buttonLoadRecipe_clicked              (bool check);
    void on_buttonLoadActions_clicked             (bool check);
    void on_buttonLoadObjects_clicked             (bool check);
    void on_buttonRunSelectedAction_clicked       (bool check);
    void on_buttonAntiX_pressed    ();
    void on_buttonAntiY_pressed    ();
    void on_buttonAntiZ_pressed    ();
    void on_buttonAntiX_released   ();
    void on_buttonAntiY_released   ();
    void on_buttonAntiZ_released   ();
    void on_buttonBack_pressed     ();
    void on_buttonBack_released    ();
    void on_buttonClockX_pressed   ();
    void on_buttonClockY_pressed   ();
    void on_buttonClockZ_pressed   ();
    void on_buttonClockX_released  ();
    void on_buttonClockY_released  ();
    void on_buttonClockZ_released  ();
    void on_buttonDown_pressed     ();
    void on_buttonDown_released    ();
    void on_buttonUp_pressed       ();
    void on_buttonLeft_pressed     ();
    void on_buttonRight_pressed    ();
    void on_buttonFront_pressed    ();
    void on_buttonUp_released      ();
    void on_buttonLeft_released    ();
    void on_buttonRight_released   ();
    void on_buttonFront_released   ();
    void on_lateralTab_currentChanged                (int index);
    void resetLocation                               (int index);
    void resetSlot                                   (int index);
    void resetBox                                    (int index);
    void resetObject                                 (int index);
    void on_robotList_currentIndexChanged            (int index);
    void on_comboGraspNumber_currentIndexChanged     (int index);
    void on_comboActionType_currentIndexChanged      (int index);
    void on_comboConfiguration_currentIndexChanged   (int index);
    void on_comboConfiguration2_currentIndexChanged  (int index);
    void on_comboRefFrame_currentIndexChanged        (int index);
    void on_TfList_currentIndexChanged               (int index);
    void on_gripperPercentage_valueChanged           (int value);
    void on_gripperForcePercentage_valueChanged      (int value);
    void on_gripperPercentage2_valueChanged          (int value);
    void on_gripperForcePercentage2_valueChanged     (int value);
    void on_velocitySlider_valueChanged              (int value);
    void on_listLocationModify_pressed  (const QModelIndex &index);
    void on_listBoxModify_pressed       (const QModelIndex &index);
    void on_listSlotModify_pressed      (const QModelIndex &index);
    void on_listObjectModify_pressed    (const QModelIndex &index);
    void on_listPlace_pressed           (const QModelIndex &index);
    void on_listPick_pressed            (const QModelIndex &index);
    void on_listGoTo_pressed            (const QModelIndex &index);
    void on_goToList_pressed            (const QModelIndex &index);
    void on_placeList_pressed           (const QModelIndex &index);
    void on_pickList_pressed            (const QModelIndex &index);

    void saveActions();
    void saveRecipe();

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
    std::vector<position>    actual_object_leave;
    std::vector<double>      actual_pre_gripper_position;
    std::vector<double>      actual_post_gripper_position;
    std::vector<double>      actual_approach_gripper_position;
    std::vector<double>      actual_gripper_grasp_force;
    std::vector<std::string> actual_tool_approach;
    std::vector<std::string> actual_tool_grasp;
    std::vector<std::string> actual_tool_leave;
    position                 actual_slot_approach;
    location                 actual_slot_final_position;
    position                 actual_slot_leave;
    position                 actual_box_approach;
    location                 actual_box_final;
    position                 actual_box_leave;
    object_type              actual_object_to_modify;
    double                   actual_gripper_force = 100;
    bool init_approach_object = false;
    bool init_slot_approach   = false;
    bool init_slot_final      = false;
    bool init_box_approach    = false;
    bool init_box_final       = false;
    bool init_objects         = false;
    bool init_slot_leave      = false;
    bool init_box_leave       = false;
    float max_vel  = 0.1;
    float max_rot  = 0.7;
    int   perc_vel = 50;
    double max_gripper_position = 85;
    double min_gripper_position = 0;
    double max_force_gripper    = 100;
    std::string tf_name_space = "manipulation";
    position default_approach;
    double default_x_approach_ = 0;
    double default_y_approach_ = 0;
    double default_z_approach_ = -0.1;

    int n_run_recipe_clicked_ = 0;
    int n_run_action_clicked_ = 0;

};

}  // namespace manipulation_interface_gui

#endif // manipulation_interface_gui_MAIN_WINDOW_H
