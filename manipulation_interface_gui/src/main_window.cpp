/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Geometry>
#include <csignal>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, ros::NodeHandle n, QWidget *parent)
    : QMainWindow(parent)
    , qnode_(argc,argv,n)
{
    ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setWindowIcon(QIcon(":/images/icon.png"));
    ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

    std::signal(SIGINT, SIG_DFL);

    /*********************
    ** Logging
    **********************/
    ui_.goToList              ->setModel(qnode_.loggingModelGoTo());
    ui_.placeList             ->setModel(qnode_.loggingModelPlace());
    ui_.pickList              ->setModel(qnode_.loggingModelPick());
    ui_.objectList            ->setModel(qnode_.loggingModelObject());
    ui_.slotList              ->setModel(qnode_.loggingModelSlot());
    ui_.groupList             ->setModel(qnode_.loggingModelGroup());
    ui_.boxList               ->setModel(qnode_.loggingModelBox());
    ui_.locationsList         ->setModel(qnode_.loggingModelLocation());
    ui_.listLocationModify    ->setModel(qnode_.loggingModelLocationModify());
    ui_.listBoxModify         ->setModel(qnode_.loggingModelBoxModify());
    ui_.listSlotModify        ->setModel(qnode_.loggingModelSlotModify());
    ui_.listObjectModify      ->setModel(qnode_.loggingModelObjectModify());
    ui_.componentList         ->setModel(qnode_.loggingModelComponents());
    ui_.listInfoAction        ->setModel(qnode_.loggingModelInfoAction());
    ui_.listGoTo              ->setModel(qnode_.loggingModelSecondGoto());
    ui_.listPlace             ->setModel(qnode_.loggingModelSecondPlace());
    ui_.listPick              ->setModel(qnode_.loggingModelSecondPick());
    ui_.listRecipe            ->setModel(qnode_.loggingModelRecipe());
    ui_.listActionComponents  ->setModel(qnode_.loggingModelActionComponents());
    ui_.jobPropertyList       ->setModel(qnode_.loggingModelJobProperties());

    QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui_.objectList    ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui_.slotList      ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui_.groupList     ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui_.boxList       ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui_.locationsList ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui_.componentList ->setSelectionMode(QAbstractItemView::MultiSelection);

    ui_.listRecipe ->setDragDropMode(QAbstractItemView::DragDrop);
    //    ui_.list_recipe->model()->doSetSupportedDragActions(Qt::MoveAction);

    ui_.listGoTo->setDragEnabled(true);
    ui_.listPlace->setDragEnabled(true);
    ui_.listPick ->setDragEnabled(true);

    ui_.buttonRemoveApproachSlot      ->setEnabled(false);
    ui_.buttonRemoveFinalPositionSlot ->setEnabled(false);
    ui_.buttonRemoveApproachBox       ->setEnabled(false);
    ui_.buttonRemoveFinalBox          ->setEnabled(false);
    ui_.buttonRemoveLeavePositionSlot ->setEnabled(false);
    ui_.buttonRemoveLeavePositionBox  ->setEnabled(false);
    ui_.lateralTab                    ->setEnabled(false);
    ui_.checkHumanInfo                ->setEnabled(false);
    ui_.checkRobotInfo                ->setEnabled(false);
    ui_.checkHumanInfo2               ->setEnabled(false);
    ui_.checkRobotInfo2               ->setEnabled(false);

    ui_.goToList             ->setEditTriggers(QListView::NoEditTriggers);
    ui_.placeList            ->setEditTriggers(QListView::NoEditTriggers);
    ui_.pickList             ->setEditTriggers(QListView::NoEditTriggers);
    ui_.objectList           ->setEditTriggers(QListView::NoEditTriggers);
    ui_.slotList             ->setEditTriggers(QListView::NoEditTriggers);
    ui_.groupList            ->setEditTriggers(QListView::NoEditTriggers);
    ui_.boxList              ->setEditTriggers(QListView::NoEditTriggers);
    ui_.locationsList        ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listLocationModify   ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listBoxModify        ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listSlotModify       ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listObjectModify     ->setEditTriggers(QListView::NoEditTriggers);
    ui_.componentList        ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listGoTo             ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listPlace            ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listPick             ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listRecipe           ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listInfoAction       ->setEditTriggers(QListView::NoEditTriggers);
    ui_.listActionComponents ->setEditTriggers(QListView::NoEditTriggers);
    ui_.componentList        ->setEditTriggers(QListView::NoEditTriggers);
    ui_.jobPropertyList      ->setEditTriggers(QListView::NoEditTriggers);

    ui_.editLocationFrame     ->setReadOnly(true);
    ui_.editSlotFrame         ->setReadOnly(true);
    ui_.editSlotGroup         ->setReadOnly(true);
    ui_.editBoxFrame          ->setReadOnly(true);
    ui_.editObjectTool        ->setReadOnly(true);
    ui_.editActionDescription ->setReadOnly(true);
    ui_.editGripperState      ->setReadOnly(true);
    ui_.editObjectName        ->setReadOnly(true);

    default_approach_.origin_x = default_x_approach_;
    default_approach_.origin_y = default_y_approach_;
    default_approach_.origin_z = default_z_approach_;

    /*********************
    ** Window connect to ROS
    **********************/
    if ( !qnode_.init() ) {
        showNoMasterMessage();
    }

    QString tf;
    for ( std::size_t i = 0; i < qnode_.TFs_.size(); i++ )
    {
        tf = QString::fromStdString(qnode_.TFs_.at(i));
        ui_.worldTfList->addItem(tf);
        ui_.comboRefFrame->addItem(tf);
    }
    on_buttonLoadTF_clicked(false);
    for ( std::size_t i = 0; i < qnode_.robots_.size(); i++)
    {
        tf = QString::fromStdString(qnode_.robots_.at(i));
        ui_.robotList->addItem(tf);
    }

    qnode_.frame_id_.clear();
    qnode_.frame_id_.append("/");
    qnode_.frame_id_.append( ui_.comboRefFrame->currentText().toStdString() );
    if ( !qnode_.writeParam(1) )
        plotMsg("Mongo doesn't work \n You probably need to change the Python version to mongo_connection or start mongo");
    qnode_.writeParam(2);
    qnode_.writeLocations();
    std::vector<std::string> recipes_names =qnode_.loadRecipesParam();
    if ( !recipes_names.empty() )
        for ( std::size_t i = 0; i < recipes_names.size(); i++)
            ui_.recipeBox->addItem( QString::fromStdString(recipes_names.at(i)) );
    writeJobType();

    std::vector<std::string> db_names;
    qnode_.returnDbNames(db_names);
    for ( const std::string db_name: db_names )
        if ( db_name.compare("default_data") & db_name.compare("admin") & db_name.compare("config") & db_name.compare("local") )
            ui_.comboDbNames->addItem( QString::fromStdString(db_name) );

    return;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    //    close();
}

void MainWindow::addGoTo(int state)
{
    std::string go_to_name = ui_.editActionName->text().toStdString();
    std::string description = ui_.editDescription->toPlainText().toStdString();
    std::string job_exec_name;
    if ( ui_.comboJobType->count() == 0 )
        job_exec_name = "";
    else
        job_exec_name = ui_.comboJobType->currentText().toStdString();
    std::string exec_property_id;
    if ( ui_.jobPropertyList->model()->rowCount() == 0 )
        exec_property_id = "";
    else
        exec_property_id = ui_.jobPropertyList->model()->data( ui_.jobPropertyList->model()->index( ui_.jobPropertyList->currentIndex().row(), 0 ), 0 ).toString().toStdString();
    std::string pre_exec_property_id;
    if ( ui_.comboPreExecProp->count() == 0 )
        pre_exec_property_id = "";
    else
        pre_exec_property_id= ui_.comboPreExecProp->currentText().toStdString();
    std::string post_exec_property_id;
    if ( ui_.comboPostExecProp->count() == 0 )
        post_exec_property_id = "";
    else
        post_exec_property_id = ui_.comboPostExecProp->currentText().toStdString();
    std::vector<std::string> locations;
    std::vector<std::string> agents;

    QModelIndexList indexes =  ui_.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
            agents.push_back("robot");
        else if ( state == 2 )
            agents.push_back("human");
        else if ( state == 3 )
        {
            agents.push_back("robot");
            agents.push_back("human");
        }
        if ( !indexes.empty() )
        {
            if ( ! go_to_name.empty() )
            {
                for ( int i = 0; i < indexes.size(); i++)
                    locations.push_back( ui_.componentList->model()->data(indexes.at(i)).toString().toStdString() );
                go_to_action gt_action;
                gt_action.name                  = go_to_name;
                gt_action.locations             = locations;
                gt_action.agents                = agents;
                gt_action.description           = description;
                gt_action.job_exec_name         = job_exec_name;
                gt_action.exec_property_id      = exec_property_id;
                gt_action.pre_exec_property_id  = pre_exec_property_id;
                gt_action.post_exec_property_id = post_exec_property_id;
                if ( !qnode_.addGoTo(gt_action) )
                {
                    plotMsg("There is another action with the same name or locations.");
                    ui_.editActionName->clear();
                }
                else
                {
                    ui_.editActionName->clear();
                    ui_.editDescription->clear();
                    ui_.componentList->clearSelection();
                }
            }
            else
                plotMsg("Empty name");
        }
        else
            plotMsg("There isn't any location selected.");
    }
    else
        plotMsg("Empty agent");
}

void MainWindow::addPlace(int state)
{
    std::string place_name = ui_.editActionName->text().toStdString();
    std::string description = ui_.editDescription->toPlainText().toStdString();
    std::string job_exec_name;
    if ( ui_.comboJobType->count() == 0 )
        job_exec_name = "";
    else
        job_exec_name = ui_.comboJobType->currentText().toStdString();
    std::string exec_property_id;
    if ( ui_.jobPropertyList->model()->rowCount() == 0 )
        exec_property_id = "";
    else
        exec_property_id = ui_.jobPropertyList->model()->data( ui_.jobPropertyList->model()->index( ui_.jobPropertyList->currentIndex().row(), 0 ), 0 ).toString().toStdString();
    std::string pre_exec_property_id;
    if ( ui_.comboPreExecProp->count() == 0 )
        pre_exec_property_id = "";
    else
        pre_exec_property_id= ui_.comboPreExecProp->currentText().toStdString();
    std::string post_exec_property_id;
    if ( ui_.comboPostExecProp->count() == 0 )
        post_exec_property_id = "";
    else
        post_exec_property_id = ui_.comboPostExecProp->currentText().toStdString();
    std::vector<std::string> groups;
    std::vector<std::string> agents;

    QModelIndexList indexes =  ui_.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
            agents.push_back("robot");
        else if ( state == 2 )
            agents.push_back("human");
        else if ( state == 3 )
        {
            agents.push_back("robot");
            agents.push_back("human");
        }
        if ( !indexes.empty() )
        {
            if ( !place_name.empty())
            {
                for ( int i = 0; i < indexes.size(); i++)
                    groups.push_back( ui_.componentList->model()->data(indexes.at(i)).toString().toStdString() );
                place pl_action;
                pl_action.name                  = place_name;
                pl_action.groups                = groups;
                pl_action.agents                = agents;
                pl_action.description           = description;
                pl_action.job_exec_name         = job_exec_name;
                pl_action.exec_property_id      = exec_property_id;
                pl_action.pre_exec_property_id  = pre_exec_property_id;
                pl_action.post_exec_property_id = post_exec_property_id;

                if ( !qnode_.addPlace(pl_action) )
                {
                    plotMsg("There is another action with the same name or groups.");
                    ui_.editActionName->clear();
                }
                else
                {
                    ui_.editActionName->clear();
                    ui_.editDescription->clear();
                    ui_.componentList->clearSelection();
                }
            }
            else
                plotMsg("Empty name");
        }
        else
            plotMsg("There isn't any slot group selected.");
    }
    else
        plotMsg("Empty agent");
}

void MainWindow::addPick(int state)
{
    std::string pick_name = ui_.editActionName->text().toStdString();
    std::string description = ui_.editDescription->toPlainText().toStdString();
    std::string job_exec_name;
    if ( ui_.comboJobType->count() == 0 )
        job_exec_name = "";
    else
        job_exec_name = ui_.comboJobType->currentText().toStdString();
    std::string exec_property_id;
    if ( ui_.jobPropertyList->model()->rowCount() == 0 )
        exec_property_id = "";
    else
        exec_property_id = ui_.jobPropertyList->model()->data( ui_.jobPropertyList->model()->index( ui_.jobPropertyList->currentIndex().row(), 0 ), 0 ).toString().toStdString();
    std::string pre_exec_property_id;
    if ( ui_.comboPreExecProp->count() == 0 )
        pre_exec_property_id = "";
    else
        pre_exec_property_id= ui_.comboPreExecProp->currentText().toStdString();
    std::string post_exec_property_id;
    if ( ui_.comboPostExecProp->count() == 0 )
        post_exec_property_id = "";
    else
        post_exec_property_id = ui_.comboPostExecProp->currentText().toStdString();
    std::vector<std::string> objects;
    std::vector<std::string> agents;

    QModelIndexList indexes =  ui_.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
            agents.push_back("robot");
        else if ( state == 2 )
            agents.push_back("human");
        else if ( state == 3 )
        {
            agents.push_back("robot");
            agents.push_back("human");
        }
        if ( !pick_name.empty())
        {
            if ( !indexes.empty())
            {
                for ( int i = 0; i < indexes.size(); i++)
                    objects.push_back( ui_.componentList->model()->data(indexes.at(i)).toString().toStdString() );
                pick pk_action;
                pk_action.name                  = pick_name;
                pk_action.objects               = objects;
                pk_action.agents                = agents;
                pk_action.description           = description;
                pk_action.job_exec_name         = job_exec_name;
                pk_action.exec_property_id      = exec_property_id;
                pk_action.pre_exec_property_id  = pre_exec_property_id;
                pk_action.post_exec_property_id = post_exec_property_id;

                if ( !qnode_.addPick(pk_action) )
                {
                    ui_.editActionName->clear();
                    plotMsg("There is another action with the same name or objects.");
                }
                else
                {
                    ui_.editActionName->clear();
                    ui_.editDescription->clear();
                    ui_.componentList->clearSelection();
                }
            }
            else
                plotMsg("There isn't any selected object type");
        }
        else
            plotMsg("Empty name");
    }
    else
        plotMsg("Empty agent");
}

void MainWindow::on_buttonAddAction_clicked(bool check)
{
    std::string type = ui_.comboActionType->currentText().toStdString();
    int state = 0;
    if ( ui_.checkRobot->checkState() == Qt::Checked && ui_.checkHuman->checkState() == Qt::Checked )
        state = 3;
    else if ( ui_.checkRobot->checkState() == Qt::Checked )
        state = 1;
    else if ( ui_.checkHuman->checkState() == Qt::Checked )
        state = 2;
    else
        state = 0;
    if ( !type.compare("Go to"))
        addGoTo( state );
    else if ( !type.compare("Pick"))
        addPick( state );
    else if ( !type.compare("Place") )
        addPlace( state );
    saveActions();
}

void MainWindow::on_buttonAddGrasp_clicked(bool check)
{
    qnode_.loadTF();

    std::string actual_base_frame;
    std::string frame_name = ui_.TfList->currentText().toStdString();
    double distance = std::numeric_limits<double>::infinity();
    object_loader_msgs::ListObjects objects_list;
    if ( !qnode_.returnObjectLoaderList(objects_list) )
    {
        plotMsg("Error whit return object loader list");
        return;
    }
    for ( const std::string object_name: objects_list.response.ids)
    {
        if ( object_name.find(frame_name) != std::string::npos )
        {
            location loc;
            if ( !qnode_.returnPosition(object_name, qnode_.target_frame_, loc) )
            {
                ROS_ERROR("Error with Transform");
                continue;
            }
            Eigen::Vector3d grasp_position;
            grasp_position(0) = loc.pos.origin_x;
            grasp_position(1) = loc.pos.origin_y;
            grasp_position(2) = loc.pos.origin_z;
            ROS_INFO("Grasp distance = %lf ", grasp_position.norm());
            if ( grasp_position.norm() < distance )
            {
                distance = grasp_position.norm();
                actual_base_frame = object_name;
            }
        }
    }
    ROS_INFO("Chosen object frame: %s", actual_base_frame.c_str());
    location grasp_loc;
    if ( !qnode_.returnPosition(actual_base_frame, qnode_.target_frame_, grasp_loc) )
    {
        plotMsg("Error whit Transform");
        return;
    }
    actual_object_grasp_.push_back(grasp_loc);
    actual_tool_grasp_.push_back(qnode_.target_frame_);
    num_grasp_++;
    std::string str = "grasp";
    str.append(std::to_string(num_grasp_));
    QString grasp;
    grasp.append(str.c_str());
    ui_.graspList->addItem(grasp);
    ui_.graspList->setCurrentIndex(ui_.graspList->count()-1);
    actual_object_approach_.push_back(default_approach_);
    actual_object_leave_.push_back(default_approach_);
    actual_pre_gripper_position_.push_back(max_gripper_position_);
    actual_approach_gripper_position_.push_back(std::nan("1"));
    actual_post_gripper_position_.push_back(qnode_.returnGripperPosition());
    actual_gripper_grasp_force_.push_back(actual_gripper_force_);
    actual_tool_approach_.push_back(qnode_.target_frame_);
    actual_tool_leave_.push_back(qnode_.target_frame_);
}

void MainWindow::on_buttonSetApproach_clicked(bool check)
{
    if ( ui_.graspList->count() == 0 )
        return;
    int index = ui_.graspList->currentIndex();
    std::string actual_base_frame = tf_name_space_+"/";
    actual_base_frame.append( ui_.TfList->currentText().toStdString() );
    location actual_approach;
    if ( !qnode_.returnPosition(actual_base_frame, qnode_.target_frame_, actual_approach) )
    {
        plotMsg("Error with Transfom");
        return;
    }
    double dist_x = actual_approach.pos.origin_x - actual_object_grasp_.at(index).pos.origin_x;
    double dist_y = actual_approach.pos.origin_y - actual_object_grasp_.at(index).pos.origin_y;
    double dist_z = actual_approach.pos.origin_z - actual_object_grasp_.at(index).pos.origin_z;
    Eigen::Vector3d dist_obj(dist_x,dist_y,dist_z);
    double w = actual_object_grasp_.at(index).quat.rotation_w;
    double x = actual_object_grasp_.at(index).quat.rotation_x;
    double y = actual_object_grasp_.at(index).quat.rotation_y;
    double z = actual_object_grasp_.at(index).quat.rotation_z;
    Eigen::Quaterniond quat_grasp( w, x, y, z);
    Eigen::MatrixXd matrix(quat_grasp.toRotationMatrix());
    Eigen::Vector3d dist_tool;
    dist_tool = matrix.inverse() * dist_obj;
    actual_object_approach_.at(index).origin_x = dist_tool[0];
    actual_object_approach_.at(index).origin_y = dist_tool[1];
    actual_object_approach_.at(index).origin_z = dist_tool[2];
    actual_approach_gripper_position_.at(index) = qnode_.returnGripperPosition();
    actual_pre_gripper_position_.at(index) = actual_approach_gripper_position_.at(index);
    actual_tool_approach_.at(index) = qnode_.target_frame_;
    if ( actual_object_leave_.at(index).origin_x == 0.0 & actual_object_leave_.at(index).origin_y == 0.0 & actual_object_leave_.at(index).origin_z == -10.0 )
        actual_object_leave_ = actual_object_approach_;
}

void MainWindow::on_buttonSetLeave_clicked(bool check)
{
    if ( ui_.graspList->count() == 0 )
        return;
    int index = ui_.graspList->currentIndex();
    std::string actual_base_frame = tf_name_space_+"/";
    actual_base_frame.append( ui_.TfList->currentText().toStdString() );
    location actual_leave;
    if ( !qnode_.returnPosition(actual_base_frame, qnode_.target_frame_, actual_leave) )
    {
        plotMsg("Error with Transfom");
        return;
    }
    double dist_x = actual_leave.pos.origin_x - actual_object_grasp_.at(index).pos.origin_x;
    double dist_y = actual_leave.pos.origin_y - actual_object_grasp_.at(index).pos.origin_y;
    double dist_z = actual_leave.pos.origin_z - actual_object_grasp_.at(index).pos.origin_z;
    Eigen::Vector3d dist_obj(dist_x,dist_y,dist_z);
    double w = actual_object_grasp_.at(index).quat.rotation_w;
    double x = actual_object_grasp_.at(index).quat.rotation_x;
    double y = actual_object_grasp_.at(index).quat.rotation_y;
    double z = actual_object_grasp_.at(index).quat.rotation_z;
    Eigen::Quaterniond quat_grasp( w, x, y, z);
    Eigen::MatrixXd matrix(quat_grasp.toRotationMatrix());
    Eigen::Vector3d dist_tool;
    dist_tool = matrix.inverse() * dist_obj;
    actual_object_leave_.at(index).origin_x = dist_tool[0];
    actual_object_leave_.at(index).origin_y = dist_tool[1];
    actual_object_leave_.at(index).origin_z = dist_tool[2];
    if ( std::isnan(actual_approach_gripper_position_.at(index)) )
        actual_pre_gripper_position_.at(index) = qnode_.returnGripperPosition();
    actual_tool_leave_.at(index) = qnode_.target_frame_;
}

void MainWindow::on_buttonAddApproachSlot_clicked(bool check)
{
    if ( init_slot_final_ )
    {
        location actual_approach;
        if  ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_approach) )
        {
            plotMsg("Error with Transfom");
            return;
        }
        double dist_x = actual_approach.pos.origin_x - actual_slot_final_position_.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_slot_final_position_.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_slot_final_position_.pos.origin_z;
        Eigen::Vector3d dist_slot(dist_x,dist_y,dist_z);
        double w = actual_slot_final_position_.quat.rotation_w;
        double x = actual_slot_final_position_.quat.rotation_x;
        double y = actual_slot_final_position_.quat.rotation_y;
        double z = actual_slot_final_position_.quat.rotation_z;
        Eigen::Quaterniond quat_slot( w, x, y, z);
        Eigen::MatrixXd matrix(quat_slot.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_slot;
        actual_slot_approach_.origin_x = dist_tool[0];
        actual_slot_approach_.origin_y = dist_tool[1];
        actual_slot_approach_.origin_z = dist_tool[2];
        init_slot_approach_ = true;
        ui_.buttonAddApproachSlot->setEnabled(false);
        ui_.buttonRemoveApproachSlot->setEnabled(true);
    }
    else
        plotMsg("Final position is not set.");
}

void MainWindow::on_buttonAddFinalPositionSlot_clicked(bool check)
{
    actual_slot_final_position_;
    if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_slot_final_position_) )
    {
        plotMsg("Error with Transfom");
        return;
    }
    actual_slot_approach_ = default_approach_;
    init_slot_final_ = true;
    ui_.buttonAddFinalPositionSlot->setEnabled(false);
    ui_.buttonRemoveFinalPositionSlot->setEnabled(true);
}

void MainWindow::on_buttonAddLocation_clicked(bool check)
{
    std::string location_name = ui_.editLocationName->text().toStdString();
    if ( !location_name.empty() )
    {
        for ( int i = 0; i < ui_.locationsList->model()->rowCount(); i++ )
        {
            if ( !location_name.compare( ui_.locationsList->model()->data( ui_.locationsList->model()->index( i, 0), 0).toString().toStdString() ) )
            {
                plotMsg("There is another location with the same name.");
                ui_.editLocationName->clear();
                return;
            }
        }
        location loc;
        if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, loc) )
        {
            plotMsg("Error with Transfom");
            return;
        }
        go_to_location gt;
        gt.name      = location_name;
        gt.location_ = loc;
        gt.frame     = qnode_.base_frame_;
        if ( !qnode_.loadNewLocation( gt ) )
        {
            plotMsg("Can't add the locatin to location manager.");
            return;
        }
        else
        {
            plotMsg("Location added to location manager.");
            qnode_.addLocation( gt );
            ui_.editLocationName->clear();
            if ( ui_.comboActionType->currentIndex() == 0)
                qnode_.writeLocations();
        }
    }
    else
        plotMsg("Empty name.");
    qnode_.saveComponents();
}

void MainWindow::on_buttonAddRecipe_clicked(bool check)
{
    std::string recipe_name = ui_.editRecipeName->text().toStdString();
    if ( !recipe_name.empty() )
    {
        if ( ui_.listRecipe->model()->rowCount() != 0 )
        {
            if ( !qnode_.addRecipe( recipe_name ) )
            {
                plotMsg("There is another recipe with the same name or the same actions order.");
                return;
            }
            else
            {
                ui_.editRecipeName->clear();
                ui_.listRecipe->model()->removeRows( 0, ui_.listRecipe->model()->rowCount() );
                ui_.recipeBox->addItem( QString::fromStdString( recipe_name ) );
            }
        }
        else
        {
            plotMsg("The recipe is empty.");
            return;
        }
    }
    else
    {
        plotMsg("The recipe_name is empty.");
        return;
    }
    saveRecipe();
}

void MainWindow::on_buttonAddApproachBox_clicked(bool check)
{
    if ( init_box_final_ )
    {
        location actual_approach;
        if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_approach) )
        {
            plotMsg("Error with Transfom");
            return;
        }
        double dist_x = actual_approach.pos.origin_x - actual_box_final_.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_box_final_.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_box_final_.pos.origin_z;
        Eigen::Vector3d dist_box(dist_x,dist_y,dist_z);
        double w = actual_box_final_.quat.rotation_w;
        double x = actual_box_final_.quat.rotation_x;
        double y = actual_box_final_.quat.rotation_y;
        double z = actual_box_final_.quat.rotation_z;
        Eigen::Quaterniond quat_box( w, x, y, z);
        Eigen::MatrixXd matrix(quat_box.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_box;
        actual_box_approach_.origin_x = dist_tool[0];
        actual_box_approach_.origin_y = dist_tool[1];
        actual_box_approach_.origin_z = dist_tool[2];
        init_box_approach_ = true;
        ui_.buttonAddApproachBox->setEnabled(false);
        ui_.buttonRemoveApproachBox->setEnabled(true);
    }
    else
        plotMsg("Final position is not set.");
}

void MainWindow::on_buttonAddFinalBox_clicked(bool check)
{
    if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_box_final_) )
    {
        plotMsg("Error with Transfom");
        return;
    }
    actual_box_approach_ = default_approach_;
    init_box_final_ = true;
    ui_.buttonAddFinalBox->setEnabled(false);
    ui_.buttonRemoveFinalBox->setEnabled(true);
}

void MainWindow::on_buttonRemoveGoTo_clicked(bool check)
{
    QModelIndexList indexes =  ui_.goToList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.goToList->model();
        QAbstractItemModel* model_2 = ui_.listGoTo->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode_.removeGoTo(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.goToList->selectionModel()->selectedIndexes();
        }
        ui_.editActionDescription->clear();
        ui_.listInfoAction->model()->removeRows( 0, ui_.listInfoAction->model()->rowCount() );
        ui_.goToList->clearSelection();

        return;
    }

    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemoveLocation_clicked(bool check)
{
    QModelIndexList indexes =  ui_.locationsList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;

        QAbstractItemModel* model   = ui_.locationsList->model();
        QAbstractItemModel* model_2 = ui_.listLocationModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode_.removeLocation(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.locationsList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemovePlace_clicked(bool check)
{
    QModelIndexList indexes =  ui_.placeList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.placeList->model();;
        QAbstractItemModel* model_2 = ui_.listPlace->model();;
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode_.removePlace(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.placeList->selectionModel()->selectedIndexes();
        }
        ui_.editActionDescription->clear();
        ui_.listInfoAction->model()->removeRows( 0, ui_.listInfoAction->model()->rowCount() );
        ui_.placeList->clearSelection();

        return;
    }
    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemovePick_clicked(bool check)
{
    QModelIndexList indexes =  ui_.pickList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.pickList->model();
        QAbstractItemModel* model_2 = ui_.listPick->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode_.removePick(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.pickList->selectionModel()->selectedIndexes();
        }
        ui_.editActionDescription->clear();
        ui_.listInfoAction->model()->removeRows( 0, ui_.listInfoAction->model()->rowCount() );
        ui_.pickList->clearSelection();

        return;
    }
    plotMsg("There isn't a selected Pick");
}

void MainWindow::on_buttonRemoveObject_clicked(bool check)
{
    QModelIndexList indexes =  ui_.objectList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.objectList->model();
        QAbstractItemModel* model_2 = ui_.listObjectModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode_.removeObject(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.objectList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected object");
}

void MainWindow::on_buttonRemoveBox_clicked(bool check)
{
    QModelIndexList indexes =  ui_.boxList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.boxList->model();
        QAbstractItemModel* model_2 = ui_.listBoxModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode_.removeBox(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.boxList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected box");
}

void MainWindow::on_buttonRemoveGroup_clicked(bool check)
{
    QModelIndexList indexes =  ui_.groupList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui_.groupList->model();
        QAbstractItemModel* model_2 = ui_.slotList->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            std::vector<std::string> slot_names = qnode_.removeGroup(model->data(index).toString().toStdString());
            for ( std::size_t j = 0; j < slot_names.size(); j++ )
                for ( int k = 0; k < model_2->rowCount(); k++)
                    if ( !slot_names.at(j).compare(model_2->data(model_2->index(k,0)).toString().toStdString()) )
                        model_2->removeRow(model_2->index(k,0).row());
            model->removeRow(index.row());
            indexes =  ui_.groupList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected group");
}

void MainWindow::on_buttonRemoveSlot_clicked(bool check)
{
    QModelIndexList indexes =  ui_.slotList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model   = ui_.slotList->model();
        QAbstractItemModel* model_2 = ui_.listSlotModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode_.removeSlot(model->data(index).toString().toStdString());
            model  ->removeRow(index.row());
            model_2->removeRow(index.row());
            indexes =  ui_.slotList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected slot");
}

void MainWindow::on_buttonRemoveGrasp_clicked(bool check)
{
    int index = ui_.graspList->currentIndex();

    if ( ui_.graspList->count() > 0 )
    {
        ui_.graspList->removeItem(index);
        actual_object_grasp_.erase(actual_object_grasp_.begin()+index);
        actual_tool_grasp_.erase(actual_tool_grasp_.begin()+index);
        actual_object_approach_.erase(actual_object_approach_.begin()+index);
        actual_pre_gripper_position_.erase(actual_pre_gripper_position_.begin()+index);
        actual_tool_approach_.erase(actual_tool_approach_.begin()+index);
        actual_tool_leave_.erase(actual_tool_leave_.begin()+index);
        return;
    }
    plotMsg("There isn't a selected grasp");
}

void MainWindow::on_buttonRemoveApproachSlot_clicked(bool check)
{
    init_slot_approach_ = false;
    ui_.buttonAddApproachSlot->setEnabled(true);
    ui_.buttonRemoveApproachSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveFinalPositionSlot_clicked(bool check)
{
    init_slot_final_ = false;
    ui_.buttonAddFinalPositionSlot->setEnabled(true);
    ui_.buttonRemoveFinalPositionSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveApproachBox_clicked(bool check)
{
    init_box_approach_ = false;
    ui_.buttonAddApproachBox->setEnabled(true);
    ui_.buttonRemoveApproachBox->setEnabled(false);
}

void MainWindow::on_buttonRemoveFinalBox_clicked(bool check)
{
    init_box_final_ = false;
    ui_.buttonAddFinalBox->setEnabled(true);
    ui_.buttonRemoveFinalBox->setEnabled(false);
}

void MainWindow::on_buttonRemoveRecipe_clicked(bool check)
{
    int index = ui_.recipeBox->currentIndex();
    std::string name = ui_.recipeBox->currentText().toStdString();
    qnode_.removeRecipe( name );
    ui_.recipeBox->removeItem( index );
}

void MainWindow::on_buttonRunRecipe_clicked(bool check)
{
    if ( ui_.listRecipe->model()->rowCount() == 0 )
    {
        plotMsg("Empty recipe");
        return;
    }
    loadObjects();
    qnode_.activeConfiguration("watch");
    std::string risp = qnode_.runRecipe();
    qnode_.activeConfiguration("watch");
    plotMsg(risp);
    ui_.comboConfiguration->setCurrentIndex(0);
}

void MainWindow::on_buttonGripper_clicked(bool check)
{
    double actual_gripper_position_target_ = (max_gripper_position_/100) * ui_.gripperPercentage->value();
    actual_gripper_force_    = ui_.gripperForcePercentage->value();

    std::string name_action = "pos_";
    name_action.append( std::to_string(actual_gripper_position_target_) );
    name_action.append("_force_");
    name_action.append( std::to_string(actual_gripper_force_) );
    name_action.append("_vel_");
    name_action.append( std::to_string(actual_gripper_force_) );
    ROS_INFO("Gripper command: %s", name_action.c_str());
    qnode_.moveGripper(name_action);
}

void MainWindow::on_buttonGripper2_clicked(bool check)
{
    on_buttonGripper_clicked(false);
}

void MainWindow::on_buttonAddObject_clicked(bool check)
{
    QString object_name = ui_.editObjectName->text();
    std::string obj_name = object_name.toStdString();

    if ( !obj_name.empty() )
    {
        if ( !actual_object_grasp_.empty() )
        {
            if ( actual_object_grasp_.size() == actual_object_approach_.size() )
            {
                if ( actual_object_approach_.size() == actual_pre_gripper_position_.size() )
                {
                    if ( actual_tool_approach_ == actual_tool_grasp_ )
                    {
                        if ( actual_tool_leave_ == actual_tool_grasp_ )
                        {
                            for ( int i = 0; i < ui_.objectList->model()->rowCount(); i++ )
                            {
                                if ( !obj_name.compare( ui_.objectList->model()->data( ui_.objectList->model()->index( i, 0), 0).toString().toStdString() ) )
                                {
                                    ROS_ERROR("Object just set");
                                    plotMsg("There is another object with the same name.");
                                    return;
                                }
                            }
                            object_type obj;
                            obj.type     = obj_name;
                            obj.tool     = actual_tool_grasp_;
                            obj.approach = actual_object_approach_;
                            obj.grasp    = actual_object_grasp_;
                            obj.leave    = actual_object_leave_;
                            obj.pre_gripper_position  = actual_pre_gripper_position_;
                            obj.post_gripper_position = actual_post_gripper_position_;
                            obj.gripper_force         = actual_gripper_grasp_force_;
                            qnode_.addObject( obj );
                            num_grasp_ = 0;
                            ui_.graspList->clear();
                            actual_object_grasp_.clear();
                            actual_object_approach_.clear();
                            actual_object_leave_.clear();
                            actual_pre_gripper_position_.clear();
                            actual_post_gripper_position_.clear();
                            actual_approach_gripper_position_.clear();
                            actual_gripper_grasp_force_.clear();
                            actual_tool_approach_.clear();
                            actual_tool_leave_.clear();
                            actual_tool_grasp_.clear();
                            if ( ui_.comboActionType->currentIndex() == 1 )
                                qnode_.writeObjects();
                            plotMsg("Added object description");
                        }
                        else
                        {
                            plotMsg("Leave and grasp have different tool");
                            for ( std::size_t i = 0; i < actual_tool_grasp_.size(); i++)
                            {
                                ROS_ERROR("Grasp_tool: %s",actual_tool_grasp_.at(i).c_str());
                                ROS_ERROR("Leave_tool: %s",actual_tool_leave_.at(i).c_str());
                            }

                        }
                    }
                    else
                    {
                        plotMsg("Apporach and grasp have different tool");
                        for ( std::size_t i = 0; i < actual_tool_grasp_.size(); i++)
                        {
                            ROS_ERROR("Grasp_tool: %s",actual_tool_grasp_.at(i).c_str());
                            ROS_ERROR("Approach_tool: %s",actual_tool_approach_.at(i).c_str());
                        }
                    }
                }
                else {
                    ROS_ERROR("Apporach and gripper state have different sizes: %zu, %zu", actual_object_approach_.size(), actual_pre_gripper_position_.size() );
                    plotMsg("Apporach and gripper have different sizes");
                }
            }
            else {
                ROS_ERROR("Apporach and grasp have different sizes: %zu, %zu", actual_object_grasp_.size(), actual_object_approach_.size() );
                plotMsg("Apporach and grasp have different sizes");
            }
        }
        else {
            ROS_ERROR("Empty grasp");
            plotMsg("Empty grasp");
        }
    }
    else {
        ROS_ERROR("Empty name");
        plotMsg("Empty name");
    }
    qnode_.saveComponents();
}

void MainWindow::on_buttonAddSlot_clicked(bool check)
{
    bool ok;

    std::string slot_name = ui_.editSlotName->text().toStdString();
    std::string max_obj   = ui_.editMaxObject->text().toStdString();
    int num_max_obj       = ui_.editMaxObject->text().toInt(&ok);
    std::string group_name= ui_.editGroupName->text().toStdString();

    if ( !slot_name.empty() )
    {
        if ( !group_name.empty())
        {
            if ( !max_obj.empty() )
            {
                if (init_slot_final_)
                {
                    if ( ok )
                    {
                        if ( !init_slot_leave_ )
                            actual_slot_leave_ = actual_slot_approach_;
                        for ( int i = 0; i < ui_.slotList->model()->rowCount(); i++ )
                        {
                            if ( !slot_name.compare( ui_.slotList->model()->data( ui_.slotList->model()->index( i, 0 ), 0).toString().toStdString() ) )
                            {
                                ROS_ERROR("Slot just set");
                                plotMsg("There is another slot with this name");
                                ui_.editSlotName->clear();
                                return;
                            }
                        }

                        init_slot_approach_ = false;
                        init_slot_final_ = false;
                        init_slot_leave_ = false;
                        ui_.buttonAddApproachSlot->setEnabled(true);
                        ui_.buttonRemoveApproachSlot->setEnabled(false);
                        ui_.buttonAddFinalPositionSlot->setEnabled(true);
                        ui_.buttonRemoveFinalPositionSlot->setEnabled(false);
                        ui_.buttonAddLeavePositionSlot->setEnabled(true);
                        ui_.buttonRemoveLeavePositionSlot->setEnabled(false);

                        if ( !qnode_.loadNewGroup( group_name ) )
                        {
                            plotMsg("Can't add the group to location manager");
                            return;
                        }
                        manipulation_slot slt;
                        slt.name           = slot_name;
                        slt.group          = group_name;
                        slt.approach       = actual_slot_approach_;
                        slt.leave          = actual_slot_leave_;
                        slt.location_      = actual_slot_final_position_;
                        slt.max_objects    = num_max_obj;
                        slt.frame          = qnode_.base_frame_;
                        if ( !qnode_.loadNewSlot( slt ) )
                        {
                            plotMsg("Can't add the slot to location manager");
                            return;
                        }
                        else
                        {
                            qnode_.addSlot(slt);
                            ui_.editSlotName->clear();
                            ui_.editMaxObject->clear();
                            ui_.editGroupName->clear();
                            if ( ui_.comboActionType->currentIndex() == 2 )
                                qnode_.writeGroups();

                            plotMsg("Added slot and group to location manager");
                        }
                    }
                    else
                    {
                        ROS_ERROR("Max number isn't a number");
                        plotMsg("Max number isn't a number");
                        ui_.editMaxObject->clear();
                    }
                }
                else
                {
                    ROS_ERROR("Empty final position");
                    plotMsg("Empty final position");
                }
            }
            else
            {
                ROS_ERROR("Empty max number");
                plotMsg("Empty max number");
            }
        }
        else {
            ROS_ERROR("Empty group");
            plotMsg("Empty group");
        }
    }
    else {
        ROS_ERROR("Empty name");
        plotMsg("Empty name");
    }
    qnode_.saveComponents();
}

void MainWindow::on_buttonAddBox_clicked(bool check)
{
    std::string box_name = ui_.editBoxName->text().toStdString();
    if ( init_box_final_ )
    {
        if ( !box_name.empty() )
        {
            if ( !init_box_leave_ )
                actual_box_leave_ = actual_box_approach_;
            for ( int i = 0; i < ui_.boxList->model()->rowCount(); i++ )
            {
                if ( !box_name.compare( ui_.boxList->model()->data( ui_.boxList->model()->index( i, 0 ),0).toString().toStdString() ) )
                {
                    ROS_ERROR("Box just set");
                    plotMsg("There is another box with this name");
                    ui_.editBoxName->clear();
                    return;
                }
            }
            box bx;
            bx.name      = box_name;
            bx.location_ = actual_box_final_;
            bx.approach  = actual_box_approach_;
            bx.leave     = actual_box_leave_;
            bx.frame     = qnode_.base_frame_;

            init_box_approach_ = false;
            init_box_final_ = false;
            init_box_leave_ = false;
            ui_.buttonAddApproachBox->setEnabled(true);
            ui_.buttonRemoveApproachBox->setEnabled(false);
            ui_.buttonAddFinalBox->setEnabled(true);
            ui_.buttonRemoveFinalBox->setEnabled(false);
            ui_.buttonAddLeavePositionBox->setEnabled(true);
            ui_.buttonRemoveLeavePositionBox->setEnabled(false);

            if ( !qnode_.loadNewBox( bx ) )
            {
                plotMsg("Can't add the box to location manager");
                return;
            }
            else
            {
                qnode_.addBox( bx );
                ui_.editBoxName->clear();
                plotMsg("Added box to the location manager");
            }
        }
        else
        {
            ROS_ERROR("Empty name");
            plotMsg("Empty name");
        }
    }
    else
    {
        ROS_ERROR("Empty final position");
        plotMsg("Empty final position");
    }
    qnode_.saveComponents();
}

void MainWindow::on_buttonAddLocationChanges_clicked(bool check)
{
    go_to_location loc;

    QModelIndex index = ui_.listLocationModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6;
    loc.location_.pos.origin_x    = ui_.editLocationPositionX   ->text().toDouble(&ok0);
    loc.location_.pos.origin_y    = ui_.editLocationPositionY   ->text().toDouble(&ok1);
    loc.location_.pos.origin_z    = ui_.editLocationPositionZ   ->text().toDouble(&ok2);
    loc.location_.quat.rotation_x = ui_.editLocationQuaternionX ->text().toDouble(&ok3);
    loc.location_.quat.rotation_y = ui_.editLocationQuaternionY ->text().toDouble(&ok4);
    loc.location_.quat.rotation_z = ui_.editLocationQuaternionZ ->text().toDouble(&ok5);
    loc.location_.quat.rotation_w = ui_.editLocationQuaternionW ->text().toDouble(&ok6);
    loc.frame                     = ui_.editLocationFrame       ->text().toStdString();
    loc.name                      = ui_.listLocationModify      ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6)
    {
        plotMsg("One or more number aren't double");
        return;
    }

    if ( qnode_.addLocationChanges(loc) )
        plotMsg("Location added");
    else
        plotMsg("Can't add the location to location manager");

    qnode_.saveComponents();
}

void MainWindow::on_buttonAddSlotChanges_clicked(bool check)
{
    manipulation_slot slt;

    QModelIndex index = ui_.listSlotModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10, ok11, ok12, ok13;
    slt.location_.pos.origin_x    = ui_.editSlotPositionX   ->text().toDouble(&ok0);
    slt.location_.pos.origin_y    = ui_.editSlotPositionY   ->text().toDouble(&ok1);
    slt.location_.pos.origin_z    = ui_.editSlotPositionZ   ->text().toDouble(&ok2);
    slt.location_.quat.rotation_x = ui_.editSlotQuaternionX ->text().toDouble(&ok3);
    slt.location_.quat.rotation_y = ui_.editSlotQuaternionY ->text().toDouble(&ok4);
    slt.location_.quat.rotation_z = ui_.editSlotQuaternionZ ->text().toDouble(&ok5);
    slt.location_.quat.rotation_w = ui_.editSlotQuaternionW ->text().toDouble(&ok6);
    slt.approach.origin_x         = ui_.editSlotApproachX   ->text().toDouble(&ok7);
    slt.approach.origin_y         = ui_.editSlotApproachY   ->text().toDouble(&ok8);
    slt.approach.origin_z         = ui_.editSlotApproachZ   ->text().toDouble(&ok9);
    slt.leave.origin_x            = ui_.editSlotLeaveX      ->text().toDouble(&ok10);
    slt.leave.origin_y            = ui_.editSlotLeaveY      ->text().toDouble(&ok11);
    slt.leave.origin_z            = ui_.editSlotLeaveZ      ->text().toDouble(&ok12);

    slt.max_objects               = ui_.editSlotMaxObjects  ->text().toInt(&ok13);
    slt.frame                     = ui_.editSlotFrame       ->text().toStdString();
    slt.group                     = ui_.editSlotGroup       ->text().toStdString();
    slt.name                      = ui_.listSlotModify      ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 || !ok10 || !ok11 || !ok12 || !ok13 )
    {
        plotMsg("One or more number aren't numers");
        return;
    }

    if ( qnode_.addSlotChanges(slt) )
        plotMsg("Slot added");
    else
        plotMsg("Can't add the slot to location manager");

    qnode_.saveComponents();
}

void MainWindow::on_buttonAddBoxChanges_clicked(bool check)
{
    box bx;

    QModelIndex index = ui_.listBoxModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10, ok11, ok12;
    bx.location_.pos.origin_x    = ui_.editBoxPositionX   ->text().toDouble(&ok0);
    bx.location_.pos.origin_y    = ui_.editBoxPositionY   ->text().toDouble(&ok1);
    bx.location_.pos.origin_z    = ui_.editBoxPositionZ   ->text().toDouble(&ok2);
    bx.location_.quat.rotation_x = ui_.editBoxQuaternionX ->text().toDouble(&ok3);
    bx.location_.quat.rotation_y = ui_.editBoxQuaternionY ->text().toDouble(&ok4);
    bx.location_.quat.rotation_z = ui_.editBoxQuaternionZ ->text().toDouble(&ok5);
    bx.location_.quat.rotation_w = ui_.editBoxQuaternionW ->text().toDouble(&ok6);
    bx.approach.origin_x         = ui_.editBoxApproachX   ->text().toDouble(&ok7);
    bx.approach.origin_y         = ui_.editBoxApproachY   ->text().toDouble(&ok8);
    bx.approach.origin_z         = ui_.editBoxApproachZ   ->text().toDouble(&ok9);
    bx.leave.origin_x            = ui_.editBoxLeaveX      ->text().toDouble(&ok10);
    bx.leave.origin_y            = ui_.editBoxLeaveY      ->text().toDouble(&ok11);
    bx.leave.origin_z            = ui_.editBoxLeaveZ      ->text().toDouble(&ok12);
    bx.frame                     = ui_.editBoxFrame        ->text().toStdString();
    bx.name                      = ui_.listBoxModify       ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 || !ok10 || !ok11 || !ok12 )
    {
        plotMsg("One or more number aren't numers");
        return;
    }

    if ( qnode_.addBoxChanges(bx) )
        plotMsg("Box added");
    else
        plotMsg("Can't add the box to location manager");

    qnode_.saveComponents();
}

void MainWindow::on_buttonAddObjectChanges_clicked(bool check)
{
    QModelIndex index = ui_.listObjectModify->currentIndex();
    int index2 = ui_.comboGraspNumber->currentIndex();

    object_type obj = qnode_.returnObjectInfo( ui_.listObjectModify->model()->data( index ).toString().toStdString() );

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10, ok11, ok12;

    obj.approach.at(index2).origin_x     = ui_.editObjectApproachX   ->text().toDouble(&ok7);
    obj.approach.at(index2).origin_y     = ui_.editObjectApproachY   ->text().toDouble(&ok8);
    obj.approach.at(index2).origin_z     = ui_.editObjectApproachZ   ->text().toDouble(&ok9);
    obj.leave.at(index2).origin_x        = ui_.editObjectLeaveX      ->text().toDouble(&ok10);
    obj.leave.at(index2).origin_y        = ui_.editObjectLeaveY      ->text().toDouble(&ok11);
    obj.leave.at(index2).origin_z        = ui_.editObjectLeaveZ      ->text().toDouble(&ok12);
    obj.grasp.at(index2).pos.origin_x    = ui_.editObjectPositionX   ->text().toDouble(&ok0);
    obj.grasp.at(index2).pos.origin_y    = ui_.editObjectPositionY   ->text().toDouble(&ok1);
    obj.grasp.at(index2).pos.origin_z    = ui_.editObjectPositionZ   ->text().toDouble(&ok2);
    obj.grasp.at(index2).quat.rotation_x = ui_.editObjectQuaternionX ->text().toDouble(&ok3);
    obj.grasp.at(index2).quat.rotation_y = ui_.editObjectQuaternionY ->text().toDouble(&ok4);
    obj.grasp.at(index2).quat.rotation_z = ui_.editObjectQuaternionZ ->text().toDouble(&ok5);
    obj.grasp.at(index2).quat.rotation_w = ui_.editObjectQuaternionW ->text().toDouble(&ok6);
    obj.tool.at(index2)                  = ui_.editObjectTool        ->text().toStdString();
    obj.type                             = ui_.listObjectModify      ->model()->data( index ).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
        plotMsg("One or more number aren't double");
    else
        qnode_.addObjectChanges(obj);
    qnode_.saveComponents();
}

void MainWindow::saveActions()
{
    if ( !qnode_.saveActions() )
        plotMsg("There is some problem with the save");
}

void MainWindow::on_buttonLoadTF_clicked(bool chack)
{
    qnode_.loadTF();

    QString tf;
    ui_.TfList->clear();
    std::string tf_name;
    for ( std::size_t i = 0; i < qnode_.TFs_.size(); i++ )
    {
        std::size_t found = qnode_.TFs_.at(i).find( tf_name_space_.c_str() );
        if ( found != std::string::npos )
        {
            tf_name = qnode_.TFs_.at(i);
            found = tf_name.find("/",2);
            tf_name.erase( 0, found+1 );
            found = tf_name.find("/n_",0);
            tf_name.erase(tf_name.begin()+found,tf_name.end());
            tf = QString::fromStdString(tf_name);
            bool presence = false;
            for ( std::size_t j = 0; j < ui_.TfList->count(); j++ )
                if ( ui_.TfList->findText(tf) != -1 )
                    presence = true;
            if ( !presence )
                ui_.TfList->addItem(tf);
        }
    }
}

void MainWindow::on_buttonLoad_clicked(bool chack)
{
    qnode_.writeParam(1);
}

void MainWindow::on_buttonLoadObjects_clicked(bool chack)
{
    loadObjects();
}

void MainWindow::on_buttonCopyGrasp_clicked(bool chack)
{
    QModelIndex index  = ui_.listObjectModify->currentIndex();
    std::string name = ui_.listObjectModify->model()->data(index).toString().toStdString();
    int index2 = ui_.comboGraspNumber->currentIndex();

    qnode_.addObjectCopyGrasp( name, index2);
    QString qstr = QString::fromStdString( std::to_string( ui_.comboGraspNumber->count() ) );
    ui_.comboGraspNumber->addItem( qstr );
    actual_object_to_modify_ = qnode_.returnObjectInfo(name);
}

void MainWindow::on_buttonResetLocationInfo_clicked(bool chack)
{
    if ( !ui_.listLocationModify->selectionModel()->selectedIndexes().empty() )
        resetLocation( ui_.listLocationModify->model()->data(ui_.listLocationModify->currentIndex()).toString().toStdString() );
}

void MainWindow::resetLocation(const std::string &name)
{
    ui_.editLocationFrame       ->clear();
    ui_.editLocationPositionX   ->clear();
    ui_.editLocationPositionY   ->clear();
    ui_.editLocationPositionZ   ->clear();
    ui_.editLocationQuaternionW ->clear();
    ui_.editLocationQuaternionX ->clear();
    ui_.editLocationQuaternionY ->clear();
    ui_.editLocationQuaternionZ ->clear();

    go_to_location loc = qnode_.returnLocationInfo(name);
    QString frame_ = QString::fromStdString( loc.frame);
    QString pos_x  = QString::fromStdString( std::to_string(loc.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(loc.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(loc.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(loc.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(loc.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(loc.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(loc.location_.quat.rotation_w) );

    ui_.editLocationFrame       ->insert(frame_);
    ui_.editLocationPositionX   ->insert(pos_x);
    ui_.editLocationPositionY   ->insert(pos_y);
    ui_.editLocationPositionZ   ->insert(pos_z);
    ui_.editLocationQuaternionW ->insert(quat_w);
    ui_.editLocationQuaternionX ->insert(quat_x);
    ui_.editLocationQuaternionY ->insert(quat_y);
    ui_.editLocationQuaternionZ ->insert(quat_z);
}

void MainWindow::on_buttonResetSlotInfo_clicked(bool chack)
{
    if ( !ui_.listSlotModify->selectionModel()->selectedIndexes().empty() )
        resetSlot( ui_.listSlotModify->model()->data(ui_.listSlotModify->currentIndex()).toString().toStdString() );
}

void MainWindow::resetSlot(const std::string &name)
{
    ui_.editSlotFrame       ->clear();
    ui_.editSlotGroup       ->clear();
    ui_.editSlotPositionX   ->clear();
    ui_.editSlotPositionY   ->clear();
    ui_.editSlotPositionZ   ->clear();
    ui_.editSlotQuaternionW ->clear();
    ui_.editSlotQuaternionX ->clear();
    ui_.editSlotQuaternionY ->clear();
    ui_.editSlotQuaternionZ ->clear();
    ui_.editSlotApproachX   ->clear();
    ui_.editSlotApproachY   ->clear();
    ui_.editSlotApproachZ   ->clear();
    ui_.editSlotMaxObjects  ->clear();
    ui_.editSlotLeaveX      ->clear();
    ui_.editSlotLeaveY      ->clear();
    ui_.editSlotLeaveZ      ->clear();

    manipulation_slot sl = qnode_.returnSlotInfo(name);
    QString frame_ = QString::fromStdString( sl.frame);
    QString group_ = QString::fromStdString( sl.group);
    QString pos_x  = QString::fromStdString( std::to_string(sl.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(sl.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(sl.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(sl.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(sl.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(sl.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(sl.location_.quat.rotation_w) );
    QString appr_x = QString::fromStdString( std::to_string(sl.approach.origin_x) );
    QString appr_y = QString::fromStdString( std::to_string(sl.approach.origin_y) );
    QString appr_z = QString::fromStdString( std::to_string(sl.approach.origin_z) );
    QString max_ob = QString::fromStdString( std::to_string(sl.max_objects) );
    QString leav_x = QString::fromStdString( std::to_string(sl.leave.origin_x) );
    QString leav_y = QString::fromStdString( std::to_string(sl.leave.origin_y) );
    QString leav_z = QString::fromStdString( std::to_string(sl.leave.origin_z) );

    ui_.editSlotFrame       ->insert(frame_);
    ui_.editSlotGroup       ->insert(group_);
    ui_.editSlotPositionX   ->insert(pos_x);
    ui_.editSlotPositionY   ->insert(pos_y);
    ui_.editSlotPositionZ   ->insert(pos_z);
    ui_.editSlotQuaternionW ->insert(quat_w);
    ui_.editSlotQuaternionX ->insert(quat_x);
    ui_.editSlotQuaternionY ->insert(quat_y);
    ui_.editSlotQuaternionZ ->insert(quat_z);
    ui_.editSlotApproachX   ->insert(appr_x);
    ui_.editSlotApproachY   ->insert(appr_y);
    ui_.editSlotApproachZ   ->insert(appr_z);
    ui_.editSlotMaxObjects  ->insert(max_ob);
    ui_.editSlotLeaveX      ->insert(leav_x);
    ui_.editSlotLeaveY      ->insert(leav_y);
    ui_.editSlotLeaveZ      ->insert(leav_z);

}

void MainWindow::on_buttonResetBoxInfo_clicked(bool chack)
{
    if ( !ui_.listBoxModify->selectionModel()->selectedIndexes().empty() )
        resetBox( ui_.listBoxModify->model()->data(ui_.listBoxModify->currentIndex()).toString().toStdString() );
}

void MainWindow::on_buttonUp_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { 0, 0, vel, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonDown_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { 0, 0, -vel, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonLeft_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { 0, -vel, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonRight_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { 0, vel, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonFront_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { -vel, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonBack_pressed()
{
    std::vector<float> twist_move;
    float vel = max_vel_*perc_vel_/100;

    twist_move = { vel, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockX_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, rot, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockY_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, 0, rot, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockZ_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, 0, 0, rot };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiX_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, -rot, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiY_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, 0, -rot, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiZ_pressed()
{
    std::vector<float> twist_move;
    float rot = max_rot_*perc_vel_/100;

    twist_move = { 0, 0, 0, 0, 0, -rot };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonUp_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonDown_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonLeft_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonRight_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonFront_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonBack_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockX_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockY_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonClockZ_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiX_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiY_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiZ_released()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode_.cartMove ( twist_move );
}

void MainWindow::addDefaultBoxes()
{
    object_loader_msgs::ListObjects obj_list;
    if ( !qnode_.returnObjectLoaderList(obj_list) )
    {
        plotMsg("Error with return object loader list");
        return;
    }
    std::string msg = "Added the boxes: \n";
    if ( ui_.checkDefaultBoxes->checkState() == Qt::Checked )
    {
        for ( const std::string obj_name: obj_list.response.ids )
        {
            location internal_loc;
            qnode_.returnPosition(default_base_frame_, obj_name, internal_loc);

            internal_loc.pos.origin_z = internal_loc.pos.origin_z + default_box_high_;
            internal_loc.quat.rotation_x = 0.707;
            internal_loc.quat.rotation_y = 0.707;
            internal_loc.quat.rotation_z = 0.0;
            internal_loc.quat.rotation_w = 0.0;
            box bx;
            std::string str = obj_name;
            str.append("/box");
            bx.name = str;
            bx.location_ = internal_loc;
            bx.approach.origin_x = 0.0;
            bx.approach.origin_y = 0.0;
            bx.approach.origin_z = 0.0;
            bx.leave.origin_x = 0.0;
            bx.leave.origin_y = 0.0;
            bx.leave.origin_z = 0.0;
            bx.frame = default_base_frame_;

            if ( qnode_.loadNewBox( bx ) )
            {
                msg.append(bx.name);
                msg.append("\n");
            }
        }
    }
    if ( obj_list.response.ids.empty() )
        plotMsg("No boxes added");
    else
        plotMsg(msg);
}

void MainWindow::removeDefaultBoxes()
{
    manipulation_msgs::ListOfObjects manipulation_object_list;
    qnode_.returnManipulationObjectList(manipulation_object_list);
    for ( const std::string box_name: manipulation_object_list.response.box_names )
        if ( box_name.find("manipulation/") != std::string::npos )
            qnode_.removeBox(box_name);
}

void MainWindow::loadObjects()
{
    std::vector<std::string> object_list = qnode_.loadObjectsInManipulation();
    if ( object_list.empty() )
        plotMsg("No objects have been added ");
    else
    {
        std::string str = "Added the objects:\n";
        for ( std::size_t i = 0; i < object_list.size(); i++)
        {
            std::string object_name = object_list.at(i).erase( 0, object_list.at(i).find("/")+1 );
            str.append( object_name.c_str() );
            if ( i != object_list.size()-1 )
                str.append(",\n");
            else
                str.append(".");
        }
        plotMsg(str);
    }
}

void MainWindow::on_checkDefaultBoxes_stateChanged (int state)
{
    if ( state == Qt::Unchecked)
        removeDefaultBoxes();
    else if ( state == Qt::Checked )
        addDefaultBoxes();
}

void MainWindow::on_buttonRunSelectedAction_clicked(bool check)
{
    QModelIndexList indexes =  ui_.listRecipe->selectionModel()->selectedIndexes();
    if ( !indexes.empty() )
    {
        loadObjects();
        qnode_.activeConfiguration("watch");
        int index = indexes.at(0).row();
        std::string risp = qnode_.runSelectedAction(index);
        qnode_.activeConfiguration("watch");
        plotMsg(risp);
        ui_.comboConfiguration->setCurrentIndex(0);
    }
    else
    {
        plotMsg("No selected action");
        ui_.editActionName->clear();
    }
    ui_.comboConfiguration->setCurrentIndex(0);
}

void MainWindow::on_buttonCopyLocation_clicked(bool chack)
{
    if ( !ui_.listLocationModify->selectionModel()->selectedIndexes().empty() )
    {
        std::string name = ui_.listLocationModify->model()->data(ui_.listLocationModify->currentIndex()).toString().toStdString();
        go_to_location loc = qnode_.returnLocationInfo( name );
        loc.name = ui_.editNewLocation->text().toStdString();

        if ( !loc.name.empty() )
        {
            if ( !qnode_.addLocationCopy( loc ) )
            {
                plotMsg("There is another location whit the same name.");
                ui_.editNewLocation->clear();
                return;
            }
            else
            {
                ui_.editNewLocation->clear();
                if ( ui_.comboActionType->currentIndex() == 0)
                    qnode_.writeLocations();
            }
        }
        else
            plotMsg("Empty name.");
    }
    else
        plotMsg("No selected location.");

    qnode_.saveComponents();
}

void MainWindow::on_buttonCopyObject_clicked(bool chack)
{
    if ( !ui_.listObjectModify->selectionModel()->selectedIndexes().empty() )
    {
        std::string name = ui_.listObjectModify->model()->data(ui_.listObjectModify->currentIndex()).toString().toStdString();
        object_type obj = qnode_.returnObjectInfo( name );
        obj.type = ui_.editNewObject->text().toStdString();

        if ( !obj.type.empty() )
        {
            if ( !qnode_.addObjectCopy( obj ) )
            {
                plotMsg("There is another object whit the same name.");
                ui_.editNewObject->clear();
                return;
            }
            else
            {
                ui_.editNewObject->clear();
                if ( ui_.comboActionType->currentIndex() == 1 )
                    qnode_.writeObjects();
            }
        }
        else
            plotMsg("Empty name.");
    }
    else
        plotMsg("No selected object.");

    qnode_.saveComponents();
}

void MainWindow::on_buttonCopySlot_clicked(bool chack)
{
    if ( !ui_.listSlotModify->selectionModel()->selectedIndexes().empty() )
    {
        std::string name  = ui_.listSlotModify->model()->data(ui_.listSlotModify->currentIndex()).toString().toStdString();
        manipulation_slot slt = qnode_.returnSlotInfo( name );
        slt.name = ui_.editNewSlot->text().toStdString();

        if ( !slt.name.empty() )
        {
            if ( !qnode_.addSlotCopy( slt ) )
            {
                plotMsg("There is another slot whit the same name.");
                ui_.editNewSlot->clear();
                return;
            }
            else
            {
                ui_.editNewSlot->clear();
                if ( ui_.comboActionType->currentIndex() == 2 )
                    qnode_.writeGroups();
            }
        }
        else
            plotMsg("Empty name.");
    }
    else
        plotMsg("No selected slot.");

    qnode_.saveComponents();
}

void MainWindow::on_buttonCopyBox_clicked(bool chack)
{
    if ( !ui_.listBoxModify->selectionModel()->selectedIndexes().empty() )
    {
        std::string name = ui_.listBoxModify->model()->data(ui_.listBoxModify->currentIndex()).toString().toStdString();
        box bx = qnode_.returnBoxInfo( name );
        bx.name = ui_.editNewBox->text().toStdString();

        if ( !bx.name.empty() )
        {
            if ( !qnode_.addBoxCopy( bx ) )
            {
                plotMsg("There is another box whit the same name.");
                ui_.editNewBox->clear();
                return;
            }
            else
                ui_.editNewBox->clear();
        }
        else
            plotMsg("Empty name.");
    }
    else
        plotMsg("No selected box.");

    qnode_.saveComponents();
}

void MainWindow::resetBox(const std::string &name)
{
    ui_.editBoxFrame       ->clear();
    ui_.editBoxPositionX   ->clear();
    ui_.editBoxPositionY   ->clear();
    ui_.editBoxPositionZ   ->clear();
    ui_.editBoxQuaternionW ->clear();
    ui_.editBoxQuaternionX ->clear();
    ui_.editBoxQuaternionY ->clear();
    ui_.editBoxQuaternionZ ->clear();
    ui_.editBoxApproachX   ->clear();
    ui_.editBoxApproachY   ->clear();
    ui_.editBoxApproachZ   ->clear();
    ui_.editBoxLeaveX      ->clear();
    ui_.editBoxLeaveY      ->clear();
    ui_.editBoxLeaveZ      ->clear();

    box bx = qnode_.returnBoxInfo(name);
    QString frame_ = QString::fromStdString( bx.frame);
    QString pos_x  = QString::fromStdString( std::to_string(bx.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(bx.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(bx.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(bx.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(bx.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(bx.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(bx.location_.quat.rotation_w) );
    QString appr_x = QString::fromStdString( std::to_string(bx.approach.origin_x) );
    QString appr_y = QString::fromStdString( std::to_string(bx.approach.origin_y) );
    QString appr_z = QString::fromStdString( std::to_string(bx.approach.origin_z) );
    QString leav_x = QString::fromStdString( std::to_string(bx.leave.origin_x) );
    QString leav_y = QString::fromStdString( std::to_string(bx.leave.origin_y) );
    QString leav_z = QString::fromStdString( std::to_string(bx.leave.origin_z) );

    ui_.editBoxFrame       ->insert(frame_);
    ui_.editBoxPositionX   ->insert(pos_x);
    ui_.editBoxPositionY   ->insert(pos_y);
    ui_.editBoxPositionZ   ->insert(pos_z);
    ui_.editBoxQuaternionW ->insert(quat_w);
    ui_.editBoxQuaternionX ->insert(quat_x);
    ui_.editBoxQuaternionY ->insert(quat_y);
    ui_.editBoxQuaternionZ ->insert(quat_z);
    ui_.editBoxApproachX   ->insert(appr_x);
    ui_.editBoxApproachY   ->insert(appr_y);
    ui_.editBoxApproachZ   ->insert(appr_z);
    ui_.editBoxLeaveX      ->insert(leav_x);
    ui_.editBoxLeaveY      ->insert(leav_y);
    ui_.editBoxLeaveZ      ->insert(leav_z);

}

void MainWindow::on_buttonResetObjectInfo_clicked(bool chack)
{
    if ( !ui_.listObjectModify->selectionModel()->selectedIndexes().empty() )
        resetObject( ui_.listObjectModify->model()->data(ui_.listObjectModify->currentIndex()).toString().toStdString() );
}

void MainWindow::resetObject(const std::string &name)
{
    init_objects_ = false;
    ui_.comboGraspNumber->clear();

    ui_.editObjectPositionX   ->clear();
    ui_.editObjectPositionY   ->clear();
    ui_.editObjectPositionZ   ->clear();
    ui_.editObjectQuaternionW ->clear();
    ui_.editObjectQuaternionX ->clear();
    ui_.editObjectQuaternionY ->clear();
    ui_.editObjectQuaternionZ ->clear();
    ui_.editObjectApproachX   ->clear();
    ui_.editObjectApproachY   ->clear();
    ui_.editObjectApproachZ   ->clear();
    ui_.editObjectLeaveX      ->clear();
    ui_.editObjectLeaveY      ->clear();
    ui_.editObjectLeaveZ      ->clear();
    ui_.editObjectTool        ->clear();
    ui_.editGripperState      ->clear();
    ui_.editPreGripperState   ->clear();
    ui_.editPostGripperState  ->clear();

    actual_object_to_modify_ = qnode_.returnObjectInfo(name);

    for ( std::size_t i = 0; i < actual_object_to_modify_.grasp.size(); i++ )
        ui_.comboGraspNumber->addItem( QString::fromStdString( std::to_string(i) ) );
    init_objects_ = true;

    int i = ui_.comboGraspNumber->currentIndex();
    QString pos_x  = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(i).quat.rotation_w) );
    QString appr_x = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(i).origin_x) );
    QString appr_y = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(i).origin_y) );
    QString appr_z = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(i).origin_z) );
    QString leav_x = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(i).origin_x) );
    QString leav_y = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(i).origin_y) );
    QString leav_z = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(i).origin_z) );
    QString tool   = QString::fromStdString( actual_object_to_modify_.tool.at(i) );
    QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify_.gripper_force.at(i) ) );
    QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify_.pre_gripper_position.at(i) ) );
    QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify_.post_gripper_position.at(i) ) );

    ui_.editObjectPositionX   ->insert(pos_x);
    ui_.editObjectPositionY   ->insert(pos_y);
    ui_.editObjectPositionZ   ->insert(pos_z);
    ui_.editObjectQuaternionW ->insert(quat_w);
    ui_.editObjectQuaternionX ->insert(quat_x);
    ui_.editObjectQuaternionY ->insert(quat_y);
    ui_.editObjectQuaternionZ ->insert(quat_z);
    ui_.editObjectApproachX   ->insert(appr_x);
    ui_.editObjectApproachY   ->insert(appr_y);
    ui_.editObjectApproachZ   ->insert(appr_z);
    ui_.editObjectLeaveX      ->insert(leav_x);
    ui_.editObjectLeaveY      ->insert(leav_y);
    ui_.editObjectLeaveZ      ->insert(leav_z);
    ui_.editObjectTool        ->insert(tool);
    ui_.editGripperState      ->insert(gripper_force);
    ui_.editPreGripperState   ->insert(pre_position);
    ui_.editPostGripperState  ->insert(post_position);
}

void MainWindow::on_checkRobotTF_stateChanged(int state)
{
    if ( state == Qt::Checked )
    {
        ui_.lateralTab   ->setEnabled(true);

        ui_.worldTfList  ->setEnabled(false);

        qnode_.base_frame_ = ui_.worldTfList->currentText().toStdString();
    }
    else if ( state == Qt::Unchecked )
    {
        ui_.lateralTab   ->setEnabled(false);
        ui_.worldTfList  ->setEnabled(true);

        qnode_.base_frame_.clear();
    }
}

void MainWindow::on_robotList_currentIndexChanged(int index)
{
    qnode_.setTargetFrame( index );
}

void MainWindow::on_comboGraspNumber_currentIndexChanged(int index)
{
    ui_.editObjectPositionX   ->clear();
    ui_.editObjectPositionY   ->clear();
    ui_.editObjectPositionZ   ->clear();
    ui_.editObjectQuaternionW ->clear();
    ui_.editObjectQuaternionX ->clear();
    ui_.editObjectQuaternionY ->clear();
    ui_.editObjectQuaternionZ ->clear();
    ui_.editObjectApproachX   ->clear();
    ui_.editObjectApproachY   ->clear();
    ui_.editObjectApproachZ   ->clear();
    ui_.editObjectLeaveX      ->clear();
    ui_.editObjectLeaveY      ->clear();
    ui_.editObjectLeaveZ      ->clear();
    ui_.editObjectTool        ->clear();
    ui_.editGripperState      ->clear();
    ui_.editPreGripperState   ->clear();
    ui_.editPostGripperState  ->clear();

    if ( init_objects_)
    {
        QString pos_x          = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).pos.origin_x) );
        QString pos_y          = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).pos.origin_y) );
        QString pos_z          = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).pos.origin_z) );
        QString quat_x         = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).quat.rotation_x) );
        QString quat_y         = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).quat.rotation_y) );
        QString quat_z         = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).quat.rotation_z) );
        QString quat_w         = QString::fromStdString( std::to_string(actual_object_to_modify_.grasp.at(index).quat.rotation_w) );
        QString appr_x         = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(index).origin_x) );
        QString appr_y         = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(index).origin_y) );
        QString appr_z         = QString::fromStdString( std::to_string(actual_object_to_modify_.approach.at(index).origin_z) );
        QString tool           = QString::fromStdString( actual_object_to_modify_.tool.at(index) );
        QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify_.gripper_force.at(index) ) );
        QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify_.pre_gripper_position.at(index) ) );
        QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify_.post_gripper_position.at(index) ) );
        QString leav_x         = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(index).origin_x) );
        QString leav_y         = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(index).origin_y) );
        QString leav_z         = QString::fromStdString( std::to_string(actual_object_to_modify_.leave.at(index).origin_z) );

        ui_.editObjectPositionX   ->insert(pos_x);
        ui_.editObjectPositionY   ->insert(pos_y);
        ui_.editObjectPositionZ   ->insert(pos_z);
        ui_.editObjectQuaternionW ->insert(quat_w);
        ui_.editObjectQuaternionX ->insert(quat_x);
        ui_.editObjectQuaternionY ->insert(quat_y);
        ui_.editObjectQuaternionZ ->insert(quat_z);
        ui_.editObjectApproachX   ->insert(appr_x);
        ui_.editObjectApproachY   ->insert(appr_y);
        ui_.editObjectApproachZ   ->insert(appr_z);
        ui_.editObjectLeaveX      ->insert(leav_x);
        ui_.editObjectLeaveY      ->insert(leav_y);
        ui_.editObjectLeaveZ      ->insert(leav_z);
        ui_.editObjectTool        ->insert(tool);
        ui_.editGripperState      ->insert(gripper_force);
        ui_.editPreGripperState   ->insert(pre_position);
        ui_.editPostGripperState  ->insert(post_position);
    }
}

void MainWindow::on_comboActionType_currentIndexChanged(int index)
{
    if ( index == 0 )
        qnode_.writeLocations();
    else if ( index == 1)
        qnode_.writeObjects();
    else if ( index == 2 )
        qnode_.writeGroups();
    writeJobType();
    writePreExecProp();
    writeJobProperty();
    writePostExecProp();
}

void MainWindow::on_comboConfiguration_currentIndexChanged(int index)
{
    qnode_.activeConfiguration("watch");
    if ( index == 0 )
        qnode_.activeConfiguration("watch");
    else if ( index == 1 )
        qnode_.activeConfiguration("manual_guidance");
    else if ( index == 2 )
        qnode_.activeConfiguration("lin_xyz_manual_guidance");
    else if ( index == 3 )
        qnode_.activeConfiguration("rot_xyz_manual_guidance");
    else if ( index == 4 )
        qnode_.activeConfiguration("cart_teleop");
    else
        plotMsg("There is a problem with configuration");
    ui_.comboConfiguration2->setCurrentIndex( index );
}

void MainWindow::on_comboConfiguration2_currentIndexChanged(int index)
{
    ui_.comboConfiguration->setCurrentIndex( index );
}

void MainWindow::on_comboRefFrame_currentIndexChanged(int index)
{
    qnode_.frame_id_.clear();
    qnode_.frame_id_.append("/");
    qnode_.frame_id_.append( ui_.comboRefFrame->currentText().toStdString() );
}

void MainWindow::on_TfList_currentIndexChanged(int index)
{
    std::string name = ui_.TfList->currentText().toStdString();
    std::size_t found  = name.find( "/" );
    if ( found != std::string::npos)
        name.erase( found, name.size() );
    ui_.editObjectName->setText(QString::fromStdString(name));
}

void MainWindow::on_listLocationModify_pressed(const QModelIndex &index)
{
    std::string name = ui_.listLocationModify->model()->data(index).toString().toStdString();
    resetLocation( name );
}

void MainWindow::on_listBoxModify_pressed(const QModelIndex &index)
{
    std::string name = ui_.listBoxModify->model()->data(index).toString().toStdString();
    resetBox( name );
}

void MainWindow::on_listSlotModify_pressed(const QModelIndex &index)
{
    std::string name = ui_.listSlotModify->model()->data(index).toString().toStdString();
    resetSlot( name );
}

void MainWindow::on_listObjectModify_pressed(const QModelIndex &index)
{
    std::string name = ui_.listObjectModify->model()->data(index).toString().toStdString();
    resetObject( name );
}

void MainWindow::on_gripperPercentage_valueChanged(int value)
{
    ui_.gripperPercentage->setValue(value);
    ui_.gripperPercentage2->setValue(value);
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui_.openGripperLabel->setText(qstr);
    ui_.openGripperLabel2->setText(qstr);
}

void MainWindow::on_gripperForcePercentage_valueChanged(int value)
{
    ui_.gripperForcePercentage->setValue(value);
    ui_.gripperForcePercentage2->setValue(value);
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui_.gripperForceLabel->setText(qstr);
    ui_.gripperForceLabel2->setText(qstr);
}

void MainWindow::on_gripperPercentage2_valueChanged(int value)
{
    ui_.gripperPercentage->setValue(value);
}

void MainWindow::on_gripperForcePercentage2_valueChanged(int value)
{
    ui_.gripperForcePercentage->setValue(value);
}

void MainWindow::on_velocitySlider_valueChanged(int value)
{
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui_.velocityLabel->setText(qstr);
    perc_vel_ = value;
}

void MainWindow::on_lateralTab_currentChanged(int index)
{
    on_buttonLoadTF_clicked(false);
}

void MainWindow::on_tab_manager_currentChanged(int index)
{
    on_comboActionType_currentIndexChanged(ui_.comboActionType->currentIndex());
}

void MainWindow::on_buttonRemoveElement_clicked(bool check )
{
    QModelIndexList indexes =  ui_.listRecipe->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model = ui_.listRecipe->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            model->removeRow(index.row());
            indexes =  ui_.listRecipe->selectionModel()->selectedIndexes();
        }
        return;
    }

    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonLoadRecipe_clicked(bool check)
{
    std::vector<std::string> recipes_names = qnode_.loadRecipesParam();
    for ( std::size_t i = 0; i < recipes_names.size(); i++)
    {
        bool presence = false;
        for ( int j = 0; j < ui_.recipeBox->count(); j++)
            if ( !recipes_names.at(i).compare( ui_.recipeBox->itemText(j).toStdString() ))
                presence = true;
        if ( !presence )
            ui_.recipeBox->addItem( QString::fromStdString( recipes_names.at(i) ));
    }
}

void MainWindow::on_buttonWriteRecipe_clicked(bool check)
{
    std::string name = ui_.recipeBox->currentText().toStdString();
    if ( ui_.recipeBox->count() != 0 )
        qnode_.writeRecipe( name );
}

void MainWindow::on_buttonLoadActions_clicked( bool check)
{
    qnode_.writeParam(2);
}

void MainWindow::saveRecipe()
{

    if ( qnode_.saveRecipe() )
    {
        plotMsg("The recipe is saved.");
        ui_.editRecipeName->clear();
    }
    else
    {
        plotMsg("There is some problem with the save");
        return;
    }
}

void MainWindow::on_listPlace_pressed(const QModelIndex &index)
{
    qnode_.addSlotGroups( ui_.listPlace->model()->data(index).toString().toStdString() );
    place pl = qnode_.returnPlaceInfo( ui_.listPlace->model()->data(index).toString().toStdString() );
    ui_.editActionDescription2->setText( QString::fromStdString( pl.description ) );
    ui_.labelJobType2->setText( QString::fromStdString( pl.job_exec_name ) );
    ui_.labelPreExec2->setText( QString::fromStdString( pl.pre_exec_property_id ) );
    ui_.labelExecProp2->setText( QString::fromStdString( pl.exec_property_id ) );
    ui_.labelPostExec2->setText( QString::fromStdString( pl.post_exec_property_id ) );
    ui_.listGoTo->clearSelection();
    ui_.listPick->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else if ( pl.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( pl.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_listPick_pressed(const QModelIndex &index)
{
    qnode_.addObjectType( ui_.listPick->model()->data(index).toString().toStdString() );
    pick pk = qnode_.returnPickInfo( ui_.listPick->model()->data(index).toString().toStdString() );
    ui_.editActionDescription2->setText( QString::fromStdString( pk.description ) );
    ui_.labelJobType2->setText( QString::fromStdString( pk.job_exec_name ) );
    ui_.labelPreExec2->setText( QString::fromStdString( pk.pre_exec_property_id ) );
    ui_.labelExecProp2->setText( QString::fromStdString( pk.exec_property_id ) );
    ui_.labelPostExec2->setText( QString::fromStdString( pk.post_exec_property_id ) );
    ui_.listPlace->clearSelection();
    ui_.listGoTo->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else if ( pk.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( pk.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_listGoTo_pressed(const QModelIndex &index)
{
    qnode_.addLocationInfo( ui_.listGoTo->model()->data(index).toString().toStdString() );
    go_to_action gt = qnode_.returnGoToInfo( ui_.listGoTo->model()->data(index).toString().toStdString() );
    ui_.editActionDescription2->setText( QString::fromStdString( gt.description ) );
    ui_.labelJobType2->setText( QString::fromStdString( gt.job_exec_name ) );
    ui_.labelPreExec2->setText( QString::fromStdString( gt.pre_exec_property_id ) );
    ui_.labelExecProp2->setText( QString::fromStdString( gt.exec_property_id ) );
    ui_.labelPostExec2->setText( QString::fromStdString( gt.post_exec_property_id ) );
    ui_.listPlace->clearSelection();
    ui_.listPick->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else if ( gt.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(true);
    }
    else if ( gt.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo2->setChecked(true);
        ui_.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo2->setChecked(false);
        ui_.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_goToList_pressed(const QModelIndex &index)
{
    qnode_.addSecondLocationInfo( ui_.goToList->model()->data(index).toString().toStdString() );
    go_to_action gt = qnode_.returnGoToInfo( ui_.goToList->model()->data(index).toString().toStdString() );
    ui_.editActionDescription->setText( QString::fromStdString( gt.description ) );
    ui_.labelJobType->setText( QString::fromStdString( gt.job_exec_name ) );
    ui_.labelPreExec->setText( QString::fromStdString( gt.pre_exec_property_id ) );
    ui_.labelExecProp->setText( QString::fromStdString( gt.exec_property_id ) );
    ui_.labelPostExec->setText( QString::fromStdString( gt.post_exec_property_id ) );
    ui_.placeList->clearSelection();
    ui_.pickList->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
    else if ( gt.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( gt.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_placeList_pressed(const QModelIndex &index)
{
    qnode_.addSecondSlotGroups( ui_.placeList->model()->data(index).toString().toStdString() );
    place pl = qnode_.returnPlaceInfo( ui_.placeList->model()->data(index).toString().toStdString() );
    ui_.editActionDescription->setText( QString::fromStdString( pl.description ) );
    ui_.labelJobType->setText( QString::fromStdString( pl.job_exec_name ) );
    ui_.labelPreExec->setText( QString::fromStdString( pl.pre_exec_property_id ) );
    ui_.labelExecProp->setText( QString::fromStdString( pl.exec_property_id ) );
    ui_.labelPostExec->setText( QString::fromStdString( pl.post_exec_property_id ) );
    ui_.goToList->clearSelection();
    ui_.pickList->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
    else if ( pl.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( pl.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_pickList_pressed (const QModelIndex &index)
{
    qnode_.addSecondObjectType( ui_.pickList->model()->data(index).toString().toStdString() );
    pick pk = qnode_.returnPickInfo( ui_.pickList->model()->data(index).toString().toStdString() );
    ui_.editActionDescription->setText( QString::fromStdString( pk.description ) );
    ui_.labelJobType->setText( QString::fromStdString( pk.job_exec_name ) );
    ui_.labelPreExec->setText( QString::fromStdString( pk.pre_exec_property_id ) );
    ui_.labelExecProp->setText( QString::fromStdString( pk.exec_property_id ) );
    ui_.labelPostExec->setText( QString::fromStdString( pk.post_exec_property_id ) );
    ui_.placeList->clearSelection();
    ui_.goToList->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
    else if ( pk.agents.at(0) == "robot" )
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(true);
    }
    else if ( pk.agents.at(0) == "human" )
    {
        ui_.checkHumanInfo->setChecked(true);
        ui_.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui_.checkHumanInfo->setChecked(false);
        ui_.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_buttonAddLeavePositionSlot_clicked(bool check)
{
    if ( init_slot_final_ )
    {
        location actual_approach;
        if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_approach) )
        {
            plotMsg("Error with Transfom");
            return;
        }
        double dist_x = actual_approach.pos.origin_x - actual_slot_final_position_.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_slot_final_position_.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_slot_final_position_.pos.origin_z;
        Eigen::Vector3d dist_slot(dist_x,dist_y,dist_z);
        double w = actual_slot_final_position_.quat.rotation_w;
        double x = actual_slot_final_position_.quat.rotation_x;
        double y = actual_slot_final_position_.quat.rotation_y;
        double z = actual_slot_final_position_.quat.rotation_z;
        Eigen::Quaterniond quat_slot( w, x, y, z);
        Eigen::MatrixXd matrix(quat_slot.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_slot;
        actual_slot_leave_.origin_x = dist_tool[0];
        actual_slot_leave_.origin_y = dist_tool[1];
        actual_slot_leave_.origin_z = dist_tool[2];
        init_slot_leave_ = true;
        ui_.buttonAddLeavePositionSlot->setEnabled(false);
        ui_.buttonRemoveLeavePositionSlot->setEnabled(true);
    }
    else
        plotMsg("Final position is not set.");
}

void MainWindow::on_buttonAddLeavePositionBox_clicked(bool check)
{
    if ( init_box_final_ )
    {
        location actual_approach;
        if ( !qnode_.returnPosition(qnode_.base_frame_, qnode_.target_frame_, actual_approach))
        {
            plotMsg("Error with Transfom");
            return;
        }
        double dist_x = actual_approach.pos.origin_x - actual_box_final_.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_box_final_.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_box_final_.pos.origin_z;
        Eigen::Vector3d dist_box(dist_x,dist_y,dist_z);
        double w = actual_box_final_.quat.rotation_w;
        double x = actual_box_final_.quat.rotation_x;
        double y = actual_box_final_.quat.rotation_y;
        double z = actual_box_final_.quat.rotation_z;
        Eigen::Quaterniond quat_box( w, x, y, z);
        Eigen::MatrixXd matrix(quat_box.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_box;
        actual_box_leave_.origin_x = dist_tool[0];
        actual_box_leave_.origin_y = dist_tool[1];
        actual_box_leave_.origin_z = dist_tool[2];
        init_box_leave_ = true;
        ui_.buttonAddLeavePositionBox->setEnabled(false);
        ui_.buttonRemoveLeavePositionBox->setEnabled(true);
    }
    else
        plotMsg("Final position is not set.");
}

void MainWindow::on_buttonRemoveLeavePositionSlot_clicked(bool check)
{
    init_slot_leave_ = false;
    ui_.buttonAddLeavePositionSlot->setEnabled(true);
    ui_.buttonRemoveLeavePositionSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveLeavePositionBox_clicked(bool check)
{
    init_box_leave_ = false;
    ui_.buttonAddLeavePositionBox->setEnabled(true);
    ui_.buttonRemoveLeavePositionBox->setEnabled(false);
}

void MainWindow::plotMsg(std::string msg)
{
    QMessageBox msgBox;
    msgBox.setText(msg.c_str());
    msgBox.exec();
}

void MainWindow::on_comboJobType_currentIndexChanged(int index)
{
    writeJobProperty();
    writePreExecProp();
    writePostExecProp();
}

void MainWindow::writeJobType()
{
    std::vector<std::string> job_list;
    ui_.comboJobType->clear();
    int action_index = ui_.comboActionType->currentIndex();
    if ( action_index == 0)
    {
        if ( !qnode_.getGoToJobList( job_list ) )
        {
            ROS_ERROR("Unable to get goto job list");
            return;
        }
        else
        {
            if ( job_list.size() == 0 )
                ROS_WARN("Job list is empty");

            for ( std::size_t i = 0; i < job_list.size(); i++ )
                ui_.comboJobType->addItem( QString::fromStdString( job_list.at(i) ) );
        }
    }
    else if ( action_index == 1 )
    {
        if ( !qnode_.getPickJobList( job_list ) )
        {
            ROS_ERROR("Unable to get pick job list");
            return;
        }
        else
        {
            if ( job_list.size() == 0 )
                ROS_WARN("Job list is empty");

            for ( std::size_t i = 0; i < job_list.size(); i++ )
                ui_.comboJobType->addItem( QString::fromStdString( job_list.at(i) ) );
        }
    }
    else if ( action_index == 2 )
    {
        if ( !qnode_.getPlaceJobList( job_list ) )
        {
            ROS_ERROR("Unable to get place job list");
            return;
        }
        else
        {
            if ( job_list.size() == 0 )
                ROS_WARN("Job list is empty");

            for ( std::size_t i = 0; i < job_list.size(); i++ )
                ui_.comboJobType->addItem( QString::fromStdString( job_list.at(i) ) );
        }
    }
}

void MainWindow::writePreExecProp()
{
    ui_.comboPreExecProp->clear();
    std::string job_type = ui_.comboJobType->currentText().toStdString();
    std::vector<std::string> pre_exec_prop_list;
    if ( !qnode_.getPreExecProp( job_type, pre_exec_prop_list ))
    {
        ROS_ERROR("Unable to get pre executer property list");
        return;
    }
    else
        for ( std::size_t i = 0; i < pre_exec_prop_list.size(); i++ )
            ui_.comboPreExecProp->addItem( QString::fromStdString( pre_exec_prop_list.at(i) ) );
}

void MainWindow::writePostExecProp()
{
    ui_.comboPostExecProp->clear();
    std::string job_type = ui_.comboJobType->currentText().toStdString();
    std::vector<std::string> post_exec_prop_list;
    if ( !qnode_.getPostExecProp( job_type, post_exec_prop_list ) )
    {
        ROS_ERROR("Unable to get post executer property list");
        return;
    }
    else
        for ( std::size_t i = 0; i < post_exec_prop_list.size(); i++ )
            ui_.comboPostExecProp->addItem( QString::fromStdString( post_exec_prop_list.at(i) ) );
}

void MainWindow::writeJobProperty()
{
    if ( ui_.jobPropertyList->model()->rowCount() > 0 )
        ui_.jobPropertyList->model()->removeRows( 0, ui_.jobPropertyList->model()->rowCount()-1 );
    std::string job_type = ui_.comboJobType->currentText().toStdString();
    std::vector<std::string> exec_prop_list;
    if ( !qnode_.getExecProp( job_type, exec_prop_list ) )
    {
        ROS_ERROR("Unable to get executer property list");
        return;
    }
    else
    {
        for ( std::size_t i = 0; i < exec_prop_list.size(); i++ )
        {
            ui_.jobPropertyList->model()->insertRow( ui_.jobPropertyList->model()->rowCount() );
            QVariant new_row( QString( exec_prop_list.at(i).c_str() ) );
            ui_.jobPropertyList->model()->setData( ui_.jobPropertyList->model()->index( ui_.jobPropertyList->model()->rowCount()-1, 0 ), new_row );
        }
    }
}

void MainWindow::on_buttonAddDbName_clicked(bool check)
{
    QString db_name = ui_.editDbName->text();
    bool presence = false;
    for ( int i = 0; i < ui_.comboDbNames->count(); i++ )
        if ( !ui_.comboDbNames->itemText(i).compare(db_name) )
            presence = true;
    if ( !presence )
        ui_.comboDbNames->addItem(db_name);
    else
        plotMsg("There is another db with the same name");
    ui_.editDbName->clear();
}

void MainWindow::on_comboDbNames_currentIndexChanged(int index)
{
    clearAll();
    qnode_.clearAll();
    qnode_.changeDbName(ui_.comboDbNames->currentText().toStdString());
    if ( !qnode_.writeParam(1) )
        plotMsg("Mongo doesn't work \n You probably need to change the Python version to mongo_connection or start mongo");
    qnode_.writeParam(2);
}

void MainWindow::clearAll()
{
    ui_.locationsList        ->model()->removeRows(0, ui_.locationsList        ->model()->rowCount());
    ui_.slotList             ->model()->removeRows(0, ui_.slotList             ->model()->rowCount());
    ui_.groupList            ->model()->removeRows(0, ui_.groupList            ->model()->rowCount());
    ui_.boxList              ->model()->removeRows(0, ui_.boxList              ->model()->rowCount());
    ui_.objectList           ->model()->removeRows(0, ui_.objectList           ->model()->rowCount());
    ui_.goToList             ->model()->removeRows(0, ui_.goToList             ->model()->rowCount());
    ui_.placeList            ->model()->removeRows(0, ui_.placeList            ->model()->rowCount());
    ui_.pickList             ->model()->removeRows(0, ui_.pickList             ->model()->rowCount());
    ui_.listLocationModify   ->model()->removeRows(0, ui_.listLocationModify   ->model()->rowCount());
    ui_.listObjectModify     ->model()->removeRows(0, ui_.listObjectModify     ->model()->rowCount());
    ui_.listBoxModify        ->model()->removeRows(0, ui_.listBoxModify        ->model()->rowCount());
    ui_.listSlotModify       ->model()->removeRows(0, ui_.listSlotModify       ->model()->rowCount());
    ui_.listGoTo             ->model()->removeRows(0, ui_.listGoTo             ->model()->rowCount());
    ui_.listPlace            ->model()->removeRows(0, ui_.listPlace            ->model()->rowCount());
    ui_.listPick             ->model()->removeRows(0, ui_.listPick             ->model()->rowCount());
    ui_.listActionComponents ->model()->removeRows(0, ui_.listActionComponents ->model()->rowCount());
    ui_.listRecipe           ->model()->removeRows(0, ui_.listRecipe           ->model()->rowCount());
    ui_.listInfoAction       ->model()->removeRows(0, ui_.listInfoAction       ->model()->rowCount());
    ui_.componentList        ->model()->removeRows(0, ui_.locationsList        ->model()->rowCount());
    ui_.jobPropertyList      ->model()->removeRows(0, ui_.locationsList        ->model()->rowCount());

    ui_.comboJobType->clear();
    ui_.comboPostExecProp->clear();
    ui_.comboPreExecProp->clear();
}
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui_.goToList -> scrollToBottom();
    ui_.placeList -> scrollToBottom();
    ui_.pickList  -> scrollToBottom();
    ui_.listGoTo -> scrollToBottom();
    ui_.listPlace -> scrollToBottom();
    ui_.listPick  -> scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/


void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace manipulation_interface_gui

