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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, ros::NodeHandle n, ros::NodeHandle nh_i, ros::NodeHandle nh_o, ros::NodeHandle nh_g, QWidget *parent)
	: QMainWindow(parent)
    , qnode(argc,argv,n,nh_i,nh_o,nh_g)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
	** Logging
	**********************/
    ui.goToList              ->setModel(qnode.loggingModelGoTo());
    ui.placeList             ->setModel(qnode.loggingModelPlace());
    ui.pickList              ->setModel(qnode.loggingModelPick());
    ui.objectList            ->setModel(qnode.loggingModelObject());
    ui.slotList              ->setModel(qnode.loggingModelSlot());
    ui.groupList             ->setModel(qnode.loggingModelGroup());
    ui.boxList               ->setModel(qnode.loggingModelBox());
    ui.locationsList         ->setModel(qnode.loggingModelLocation());
    ui.listLocationModify    ->setModel(qnode.loggingModelLocationModify());
    ui.listBoxModify         ->setModel(qnode.loggingModelBoxModify());
    ui.listSlotModify        ->setModel(qnode.loggingModelSlotModify());
    ui.listObjectModify      ->setModel(qnode.loggingModelObjectModify());
    ui.componentList         ->setModel(qnode.loggingModelComponents());
    ui.listInfoAction        ->setModel(qnode.loggingModelInfoAction());
    ui.listGoTo              ->setModel(qnode.loggingModelSecondGoto());
    ui.listPlace             ->setModel(qnode.loggingModelSecondPlace());
    ui.listPick              ->setModel(qnode.loggingModelSecondPick());
    ui.listRecipe            ->setModel(qnode.loggingModelRecipe());

    ui.listActionComponents ->setModel(qnode.loggingModelActionComponents());

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.objectList    ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.slotList      ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.groupList     ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.boxList       ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.locationsList ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.componentList ->setSelectionMode(QAbstractItemView::MultiSelection);

    ui.listRecipe ->setDragDropMode(QAbstractItemView::DragDrop);
//    ui.list_recipe->model()->doSetSupportedDragActions(Qt::MoveAction);

    ui.listGoTo->setDragEnabled(true);
    ui.listPlace->setDragEnabled(true);
    ui.listPick ->setDragEnabled(true);

    ui.buttonRemoveApproachSlot      ->setEnabled(false);
    ui.buttonRemoveFinalPositionSlot ->setEnabled(false);
    ui.buttonRemoveApproachBox       ->setEnabled(false);
    ui.buttonRemoveFinalBox          ->setEnabled(false);
    ui.buttonRemoveLeavePositionSlot ->setEnabled(false);
    ui.buttonRemoveLeavePositionBox  ->setEnabled(false);
    ui.lateralTab                    ->setEnabled(false);
    ui.checkHumanInfo                ->setEnabled(false);
    ui.checkRobotInfo                ->setEnabled(false);
    ui.checkHumanInfo2               ->setEnabled(false);
    ui.checkRobotInfo2               ->setEnabled(false);

    ui.goToList             ->setEditTriggers(QListView::NoEditTriggers);
    ui.placeList            ->setEditTriggers(QListView::NoEditTriggers);
    ui.pickList             ->setEditTriggers(QListView::NoEditTriggers);
    ui.objectList           ->setEditTriggers(QListView::NoEditTriggers);
    ui.slotList             ->setEditTriggers(QListView::NoEditTriggers);
    ui.groupList            ->setEditTriggers(QListView::NoEditTriggers);
    ui.boxList              ->setEditTriggers(QListView::NoEditTriggers);
    ui.locationsList        ->setEditTriggers(QListView::NoEditTriggers);
    ui.listLocationModify   ->setEditTriggers(QListView::NoEditTriggers);
    ui.listBoxModify        ->setEditTriggers(QListView::NoEditTriggers);
    ui.listSlotModify       ->setEditTriggers(QListView::NoEditTriggers);
    ui.listObjectModify     ->setEditTriggers(QListView::NoEditTriggers);
    ui.componentList        ->setEditTriggers(QListView::NoEditTriggers);
    ui.listGoTo             ->setEditTriggers(QListView::NoEditTriggers);
    ui.listPlace            ->setEditTriggers(QListView::NoEditTriggers);
    ui.listPick             ->setEditTriggers(QListView::NoEditTriggers);
    ui.listRecipe           ->setEditTriggers(QListView::NoEditTriggers);
    ui.listInfoAction       ->setEditTriggers(QListView::NoEditTriggers);
    ui.listActionComponents ->setEditTriggers(QListView::NoEditTriggers);

    ui.editLocationFrame     ->setReadOnly(true);
    ui.editSlotFrame         ->setReadOnly(true);
    ui.editSlotGroup         ->setReadOnly(true);
    ui.editBoxFrame          ->setReadOnly(true);
    ui.editObjectTool        ->setReadOnly(true);
    ui.editActionDescription ->setReadOnly(true);
    ui.editGripperState      ->setReadOnly(true);
    ui.editObjectName        ->setReadOnly(true);

    default_approach.origin_x = default_x_approach_;
    default_approach.origin_y = default_y_approach_;
    default_approach.origin_z = default_z_approach_;

    /*********************
    ** Window connect to ROS
    **********************/
    if ( !qnode.init() ) {
        showNoMasterMessage();
    }

    QString tf;
    for ( int i = 0; i < qnode.TFs.size(); i++ )
    {
        tf = QString::fromStdString(qnode.TFs[i]);
        ui.worldTfList->addItem(tf);
        ui.comboRefFrame->addItem(tf);
    }
    on_buttonLoadTF_clicked(false);
    for ( int i = 0; i < qnode.robots.size(); i++)
    {
        tf = QString::fromStdString(qnode.robots[i]);
        ui.robotList->addItem(tf);
    }

    qnode.frame_id.clear();
    qnode.frame_id.append("/");
    qnode.frame_id.append( ui.comboRefFrame->currentText().toStdString() );

    qnode.writeParam(1);
//    qnode.initialAddComponentsInManipulation();
    qnode.writeParam(2);
    qnode.writeLocations();
    std::vector<std::string> recipes_names =qnode.loadRecipesParam();
    if ( !recipes_names.empty() )
    {
        for ( int i = 0; i < recipes_names.size(); i++)
        {
            ui.recipeBox->addItem( QString::fromStdString(recipes_names[i]) );
        }
    }
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

void MainWindow::addGoTo( int state )
{
    std::string go_to_name = ui.editActionName->text().toStdString();
    std::string description = ui.editDescription->toPlainText().toStdString();
    std::vector<std::string> locations;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
        {
            agents_.push_back("robot");
           }
        else if ( state == 2 )
        {
            agents_.push_back("human");
        }
        else if ( state == 3 )
        {
            agents_.push_back("robot");
            agents_.push_back("human");
        }
        if ( !indexes.empty() )
        {
            if ( ! go_to_name.empty() )
            {
                for ( int i = 0; i < indexes.size(); i++)
                {
                    locations.push_back( qnode.returnLocationListText( indexes.at(i).row() ) );
                }
                if ( !qnode.addGoTo(go_to_name,locations,description,agents_) )
                {
                    plotMsg("There is another action with the same name or locations.");
                    ui.editActionName->clear();
                }
                else
                {
                    ui.editActionName->clear();
                    ui.editDescription->clear();
                    ui.componentList->clearSelection();
                }
            }
            else
            {
                plotMsg("Empty name");
            }
        }
        else
        {
            plotMsg("There isn't any location selected.");
        }
    }
    else
    {
        plotMsg("Empty agent");
    }
}

void MainWindow::addPlace( int state )
{
    std::string place_name = ui.editActionName->text().toStdString();
    std::string description = ui.editDescription->toPlainText().toStdString();
    std::vector<std::string> groups;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
        {
            agents_.push_back("robot");
        }
        else if ( state == 2 )
        {
            agents_.push_back("human");
        }
        else if ( state == 3 )
        {
            agents_.push_back("robot");
            agents_.push_back("human");
        }
        if ( !indexes.empty() )
        {
            if ( !place_name.empty())
            {
                for ( int i = 0; i < indexes.size(); i++)
                {
                    groups.push_back( qnode.returnGroupListText( indexes.at(i).row() ) );
                }
                if ( !qnode.addPlace(place_name,groups,description,agents_) )
                {
                    plotMsg("There is another action with the same name or groups.");
                    ui.editActionName->clear();
                }
                else
                {
                    ui.editActionName->clear();
                    ui.editDescription->clear();
                    ui.componentList->clearSelection();
                }
            }
            else
            {
                plotMsg("Empty name");
            }
        }
        else
        {
            plotMsg("There isn't any slot group selected.");
        }
    }
    else
    {
        plotMsg("Empty agent");
    }
}

void MainWindow::addPick( int state )
{
    std::string pick_name = ui.editActionName->text().toStdString();
    std::string description = ui.editDescription->toPlainText().toStdString();
    std::vector<std::string> objects;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.componentList->selectionModel()->selectedIndexes();
    if ( state != 0 )
    {
        if ( state == 1 )
        {
            agents_.push_back("robot");
        }
        else if ( state == 2 )
        {
            agents_.push_back("human");
        }
        else if ( state == 3 )
        {
            agents_.push_back("robot");
            agents_.push_back("human");
        }
        if ( !pick_name.empty())
        {
            if ( !indexes.empty())
            {
                for ( int i = 0; i < indexes.size(); i++)
                {
                    objects.push_back(qnode.returnObjectListText( indexes.at(i).row() ) );
                }
                if ( !qnode.addPick(pick_name,objects,description,agents_) )
                {
                    ui.editActionName->clear();
                    plotMsg("There is another action with the same name or objects.");
                }
                else
                {
                    ui.editActionName->clear();
                    ui.editDescription->clear();
                    ui.componentList->clearSelection();
                }
            }
            else
            {
                plotMsg("There isn't any selected object type");
            }
        }
        else
        {
            plotMsg("Empty name");
        }
    }
    else
    {
        plotMsg("Empty agent");
    }
}

void MainWindow::on_buttonAddAction_clicked(bool check)
{
    std::string type = ui.comboActionType->currentText().toStdString();
    int state = 0;
    if ( ui.checkRobot->checkState() == Qt::Checked && ui.checkHuman->checkState() == Qt::Checked )
    {
        state = 3;
    }
    else if ( ui.checkRobot->checkState() == Qt::Checked )
    {
        state = 1;
    }
    else if ( ui.checkHuman->checkState() == Qt::Checked )
    {
        state = 2;
    }
    else
    {
        state = 0;
    }
    if ( !type.compare("Go to"))
    {
        addGoTo( state );
    }
    else if ( !type.compare("Pick"))
    {
        addPick( state );
    }
    else if ( !type.compare("Place") )
    {
        addPlace( state );
    }
    saveActions();
}

void MainWindow::on_buttonAddGrasp_clicked(bool check)
{
  std::string actual_base_frame = tf_name_space+"/";
  actual_base_frame.append(ui.TfList->currentText().toStdString());
  actual_object_grasp.push_back(qnode.returnPosition(actual_base_frame, qnode.target_frame));
    actual_tool_grasp.push_back(qnode.target_frame);
    num_grasp++;
    std::string str = "grasp";
    str.append(std::to_string(num_grasp));
    QString grasp;
    grasp.append(str.c_str());
    ui.graspList->addItem(grasp);
    ui.graspList->setCurrentIndex(ui.graspList->count()-1);
    actual_object_approach.push_back(default_approach);
    actual_object_leave.push_back(default_approach);
    actual_pre_gripper_position.push_back(max_gripper_position);
    actual_approach_gripper_position.push_back(std::nan("1"));
    actual_post_gripper_position.push_back(qnode.returnGripperPosition());
    actual_gripper_grasp_force.push_back(actual_gripper_force);
    actual_tool_approach.push_back(qnode.target_frame);
    actual_tool_leave.push_back(qnode.target_frame);
}

void MainWindow::on_buttonSetApproach_clicked(bool check)
{
    if ( ui.graspList->count() == 0 )
    {
        return;
    }
    int index = ui.graspList->currentIndex();
    std::string actual_base_frame = tf_name_space+"/";
    actual_base_frame.append( ui.TfList->currentText().toStdString() );
    location actual_approach = qnode.returnPosition(actual_base_frame, qnode.target_frame);
    double dist_x = actual_approach.pos.origin_x - actual_object_grasp[index].pos.origin_x;
    double dist_y = actual_approach.pos.origin_y - actual_object_grasp[index].pos.origin_y;
    double dist_z = actual_approach.pos.origin_z - actual_object_grasp[index].pos.origin_z;
    Eigen::Vector3d dist_obj(dist_x,dist_y,dist_z);
    double w = actual_object_grasp[index].quat.rotation_w;
    double x = actual_object_grasp[index].quat.rotation_x;
    double y = actual_object_grasp[index].quat.rotation_y;
    double z = actual_object_grasp[index].quat.rotation_z;
    Eigen::Quaterniond quat_grasp( w, x, y, z);
    Eigen::MatrixXd matrix(quat_grasp.toRotationMatrix());
    Eigen::Vector3d dist_tool;
    dist_tool = matrix.inverse() * dist_obj;
    actual_object_approach[index].origin_x = dist_tool[0];
    actual_object_approach[index].origin_y = dist_tool[1];
    actual_object_approach[index].origin_z = dist_tool[2];
    actual_approach_gripper_position[index] = qnode.returnGripperPosition();
    actual_pre_gripper_position[index] = actual_approach_gripper_position[index];
    actual_tool_approach[index] = qnode.target_frame;
}

void MainWindow::on_buttonSetLeave_clicked(bool check)
{
    if ( ui.graspList->count() == 0 )
    {
        return;
    }
    int index = ui.graspList->currentIndex();
    std::string actual_base_frame = tf_name_space+"/";
    actual_base_frame.append( ui.TfList->currentText().toStdString() );
    location actual_leave = qnode.returnPosition(actual_base_frame, qnode.target_frame);
    double dist_x = actual_leave.pos.origin_x - actual_object_grasp[index].pos.origin_x;
    double dist_y = actual_leave.pos.origin_y - actual_object_grasp[index].pos.origin_y;
    double dist_z = actual_leave.pos.origin_z - actual_object_grasp[index].pos.origin_z;
    Eigen::Vector3d dist_obj(dist_x,dist_y,dist_z);
    double w = actual_object_grasp[index].quat.rotation_w;
    double x = actual_object_grasp[index].quat.rotation_x;
    double y = actual_object_grasp[index].quat.rotation_y;
    double z = actual_object_grasp[index].quat.rotation_z;
    Eigen::Quaterniond quat_grasp( w, x, y, z);
    Eigen::MatrixXd matrix(quat_grasp.toRotationMatrix());
    Eigen::Vector3d dist_tool;
    dist_tool = matrix.inverse() * dist_obj;
    actual_object_leave[index].origin_x = dist_tool[0];
    actual_object_leave[index].origin_y = dist_tool[1];
    actual_object_leave[index].origin_z = dist_tool[2];
    if ( std::isnan(actual_approach_gripper_position[index]) )
    {
      actual_pre_gripper_position[index] = qnode.returnGripperPosition();
    }
    actual_tool_leave[index] = qnode.target_frame;
}

void MainWindow::on_buttonAddApproachSlot_clicked(bool check)
{
    if ( init_slot_final )
    {
        location actual_approach = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
        double dist_x = actual_approach.pos.origin_x - actual_slot_final_position.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_slot_final_position.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_slot_final_position.pos.origin_z;
        Eigen::Vector3d dist_slot(dist_x,dist_y,dist_z);
        double w = actual_slot_final_position.quat.rotation_w;
        double x = actual_slot_final_position.quat.rotation_x;
        double y = actual_slot_final_position.quat.rotation_y;
        double z = actual_slot_final_position.quat.rotation_z;
        Eigen::Quaterniond quat_slot( w, x, y, z);
        Eigen::MatrixXd matrix(quat_slot.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_slot;
        actual_slot_approach.origin_x = dist_tool[0];
        actual_slot_approach.origin_y = dist_tool[1];
        actual_slot_approach.origin_z = dist_tool[2];
        init_slot_approach = true;
        ui.buttonAddApproachSlot->setEnabled(false);
        ui.buttonRemoveApproachSlot->setEnabled(true);
    }
    else
    {
        plotMsg("Final position is not set.");
    }
}

void MainWindow::on_buttonAddFinalPositionSlot_clicked(bool check)
{
    actual_slot_final_position = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
    actual_slot_approach = default_approach;
    init_slot_final = true;
    ui.buttonAddFinalPositionSlot->setEnabled(false);
    ui.buttonRemoveFinalPositionSlot->setEnabled(true);
}

void MainWindow::on_buttonAddLocation_clicked(bool check)
{
    std::string location_name = ui.editLocationName->text().toStdString();
    if ( !location_name.empty() )
    {
        for ( int i = 0; i < ui.locationsList->model()->rowCount(); i++ )
        {
            if ( !location_name.compare( ui.locationsList->model()->data( ui.locationsList->model()->index( i, 0), 0).toString().toStdString() ) )
            {
                plotMsg("There is another location with the same name.");
                ui.editLocationName->clear();
                return;
            }
        }
        location loc = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
        go_to_location gt;
        gt.name      = location_name;
        gt.location_ = loc;
        gt.frame     = qnode.base_frame;
        if ( !qnode.loadNewLocation( gt ) )
        {
            plotMsg("Can't add the locatin to location manager.");
            return;
        }
        else
        {
            plotMsg("Location added to location manager.");
            qnode.addLocation( gt );
            ui.editLocationName->clear();
            if ( ui.comboActionType->currentIndex() == 0)
            {
                qnode.writeLocations();
            }
        }
    }
    else
    {
        plotMsg("Empty name.");
    }
    qnode.saveComponents();
}

void MainWindow::on_buttonAddRecipe_clicked(bool check)
{
    std::string recipe_name = ui.editRecipeName->text().toStdString();
    if ( !recipe_name.empty() )
    {
        if ( ui.listRecipe->model()->rowCount() != 0 )
        {
            if ( !qnode.addRecipe( recipe_name ) )
            {
                plotMsg("There is another recipe with the same name or the same actions order.");
                return;
            }
            else
            {
                ui.editRecipeName->clear();
                ui.listRecipe->model()->removeRows( 0, ui.listRecipe->model()->rowCount() );
                ui.recipeBox->addItem( QString::fromStdString( recipe_name ) );
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
    if ( init_box_final )
    {
        location actual_approach = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
        double dist_x = actual_approach.pos.origin_x - actual_box_final.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_box_final.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_box_final.pos.origin_z;
        Eigen::Vector3d dist_box(dist_x,dist_y,dist_z);
        double w = actual_box_final.quat.rotation_w;
        double x = actual_box_final.quat.rotation_x;
        double y = actual_box_final.quat.rotation_y;
        double z = actual_box_final.quat.rotation_z;
        Eigen::Quaterniond quat_box( w, x, y, z);
        Eigen::MatrixXd matrix(quat_box.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_box;
        actual_box_approach.origin_x = dist_tool[0];
        actual_box_approach.origin_y = dist_tool[1];
        actual_box_approach.origin_z = dist_tool[2];
        init_box_approach = true;
        ui.buttonAddApproachBox->setEnabled(false);
        ui.buttonRemoveApproachBox->setEnabled(true);
    }
    else
    {
        plotMsg("Final position is not set.");
    }
}

void MainWindow::on_buttonAddFinalBox_clicked(bool check)
{
    actual_box_final = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
    actual_box_approach = default_approach;
    init_box_final = true;
    ui.buttonAddFinalBox->setEnabled(false);
    ui.buttonRemoveFinalBox->setEnabled(true);
}

void MainWindow::on_buttonRemoveGoTo_clicked(bool check)
{
    QModelIndexList indexes =  ui.goToList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.goToList->model();
        QAbstractItemModel* model_ = ui.listGoTo->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode.removeGoTo(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.goToList->selectionModel()->selectedIndexes();
        }
        ui.editActionDescription->clear();
        ui.listInfoAction->model()->removeRows( 0, ui.listInfoAction->model()->rowCount() );
        ui.goToList->clearSelection();

        return;
    }

    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemoveLocation_clicked(bool check)
{
    QModelIndexList indexes =  ui.locationsList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;

        QAbstractItemModel* model  = ui.locationsList->model();
        QAbstractItemModel* model_ = ui.listLocationModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode.removeLocation(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.locationsList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemovePlace_clicked(bool check)
{
    QModelIndexList indexes =  ui.placeList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.placeList->model();;
        QAbstractItemModel* model_ = ui.listPlace->model();;
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.removePlace(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.placeList->selectionModel()->selectedIndexes();
        }
        ui.editActionDescription->clear();
        ui.listInfoAction->model()->removeRows( 0, ui.listInfoAction->model()->rowCount() );
        ui.placeList->clearSelection();

        return;
    }
    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonRemovePick_clicked(bool check)
{
    QModelIndexList indexes =  ui.pickList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.pickList->model();
        QAbstractItemModel* model_ = ui.listPick->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.removePick(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.pickList->selectionModel()->selectedIndexes();
        }
        ui.editActionDescription->clear();
        ui.listInfoAction->model()->removeRows( 0, ui.listInfoAction->model()->rowCount() );
        ui.pickList->clearSelection();

        return;
    }
    plotMsg("There isn't a selected Pick");
}

void MainWindow::on_buttonRemoveObject_clicked(bool check)
{
    QModelIndexList indexes =  ui.objectList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.objectList->model();
        QAbstractItemModel* model_ = ui.listObjectModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.removeObject(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.objectList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected object");
}

void MainWindow::on_buttonRemoveBox_clicked(bool check)
{
    QModelIndexList indexes =  ui.boxList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.boxList->model();
        QAbstractItemModel* model_ = ui.listBoxModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.removeBox(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.boxList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected box");
}

void MainWindow::on_buttonRemoveGroup_clicked(bool check)
{
    QModelIndexList indexes =  ui.groupList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.groupList->model();
        QAbstractItemModel* model2 = ui.slotList->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            std::vector<int> indexes2 = qnode.removeGroup(index.row());
            for ( int i = indexes2.size()-1; i >= 0; i--)
            {
                model2->removeRow(indexes2[i]);
            }
            model->removeRow(index.row());
            indexes =  ui.groupList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected group");
}

void MainWindow::on_buttonRemoveSlot_clicked(bool check)
{
    QModelIndexList indexes =  ui.slotList->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.slotList->model();
        QAbstractItemModel* model_ = ui.listSlotModify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.removeSlot(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.slotList->selectionModel()->selectedIndexes();
        }
        return;
    }
    plotMsg("There isn't a selected slot");
}

void MainWindow::on_buttonRemoveGrasp_clicked(bool check)
{
    int index = ui.graspList->currentIndex();

    if ( ui.graspList->count() > 0 )
    {
        ui.graspList->removeItem(index);
        actual_object_grasp.erase(actual_object_grasp.begin()+index);
        actual_tool_grasp.erase(actual_tool_grasp.begin()+index);
        actual_object_approach.erase(actual_object_approach.begin()+index);
        actual_pre_gripper_position.erase(actual_pre_gripper_position.begin()+index);
        actual_tool_approach.erase(actual_tool_approach.begin()+index);
        actual_tool_leave.erase(actual_tool_leave.begin()+index);
        return;
    }
    plotMsg("There isn't a selected grasp");
}

void MainWindow::on_buttonRemoveApproachSlot_clicked(bool check)
{
    init_slot_approach = false;
    ui.buttonAddApproachSlot->setEnabled(true);
    ui.buttonRemoveApproachSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveFinalPositionSlot_clicked(bool check)
{
    init_slot_final = false;
    ui.buttonAddFinalPositionSlot->setEnabled(true);
    ui.buttonRemoveFinalPositionSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveApproachBox_clicked(bool check)
{
    init_box_approach = false;
    ui.buttonAddApproachBox->setEnabled(true);
    ui.buttonRemoveApproachBox->setEnabled(false);
}

void MainWindow::on_buttonRemoveFinalBox_clicked(bool check)
{
    init_box_final = false;
    ui.buttonAddFinalBox->setEnabled(true);
    ui.buttonRemoveFinalBox->setEnabled(false);
}

void MainWindow::on_buttonRemoveRecipe_clicked(bool check)
{
    int index = ui.recipeBox->currentIndex();
    qnode.removeRecipe( index );
    ui.recipeBox->removeItem( index );
}

void MainWindow::on_buttonRunRecipe_clicked(bool check)
{
    if ( ui.listRecipe->model()->rowCount() == 0 )
    {
        plotMsg("Empty recipe");
        return;
    }
    n_run_recipe_clicked_++;
    if ( n_run_recipe_clicked_ % 2 )
    {
        plotMsg("Did you load the objects?");
    }
    else
    {
        std::string risp = qnode.runRecipe();
        plotMsg(risp);
    }
}

void MainWindow::on_buttonGripper_clicked(bool check)
{
    double actual_gripper_position_target = (max_gripper_position/100) * ui.gripperPercentage->value();
    actual_gripper_force    = ui.gripperForcePercentage->value();

    std::string name_action = "pos_";
    name_action.append( std::to_string(actual_gripper_position_target) );
    name_action.append("_force_");
    name_action.append( std::to_string(actual_gripper_force) );
    ROS_INFO("Gripper command: %s", name_action.c_str());
    qnode.moveGripper("close");
}

void MainWindow::on_buttonGripper2_clicked(bool check)
{
    on_buttonGripper_clicked(false);
}

void MainWindow::on_buttonAddObject_clicked(bool check)
{ 
    QString object_name = ui.editObjectName->text();
    std::string obj_name = object_name.toStdString();

    if ( !obj_name.empty() )
    {
        if ( !actual_object_grasp.empty() )
        {
            if ( actual_object_grasp.size() == actual_object_approach.size() )
            {
                if ( actual_object_approach.size() == actual_pre_gripper_position.size() )
                {
                    if ( actual_tool_approach == actual_tool_grasp )
                    {
                        if ( actual_tool_leave == actual_tool_grasp )
                        {
                            object_type obj;
                            obj.type     = obj_name;
                            obj.tool     = actual_tool_grasp;
                            obj.approach = actual_object_approach;
                            obj.grasp    = actual_object_grasp;
                            obj.leave    = actual_object_leave;
                            obj.pre_gripper_position  = actual_pre_gripper_position;
                            obj.post_gripper_position = actual_post_gripper_position;
                            obj.gripper_force         = actual_gripper_grasp_force;
                            if( !qnode.add_object( obj ) )
                            {
                                ROS_ERROR("Object just set");
                                plotMsg("There is another object with this name");
                                ui.editObjectName->clear();
                            }
                            else
                            {
                                num_grasp = 0;
                                ui.graspList->clear();
                                init_approach_object = false;
                                actual_object_grasp.clear();
                                actual_object_approach.clear();
                                actual_object_leave.clear();
                                actual_pre_gripper_position.clear();
                                actual_post_gripper_position.clear();
                                actual_approach_gripper_position.clear();
                                actual_gripper_grasp_force.clear();
                                actual_tool_approach.clear();
                                actual_tool_leave.clear();
                                actual_tool_grasp.clear();
                                if ( ui.comboActionType->currentIndex() == 1 )
                                {
                                    qnode.writeObjects();
                                }
                                plotMsg("Added object description");
                            }
                        }
                        else
                        {
                            plotMsg("Leave and grasp have different tool");
                            for ( int i = 0; i < actual_tool_grasp.size(); i++)
                            {
                              ROS_ERROR("Grasp_tool: %s",actual_tool_grasp[i].c_str());
                              ROS_ERROR("Leave_tool: %s",actual_tool_leave[i].c_str());
                            }

                        }
                    }
                    else
                    {
                        plotMsg("Apporach and grasp have different tool");
                        for ( int i = 0; i < actual_tool_grasp.size(); i++)
                        {
                          ROS_ERROR("Grasp_tool: %s",actual_tool_grasp[i].c_str());
                          ROS_ERROR("Approach_tool: %s",actual_tool_approach[i].c_str());
                        }
                    }
                }
                else {
                    ROS_ERROR("Apporach and gripper state have different sizes: %zu, %zu", actual_object_approach.size(), actual_pre_gripper_position.size() );
                    plotMsg("Apporach and gripper have different sizes");
                }
            }
            else {
                ROS_ERROR("Apporach and grasp have different sizes: %zu, %zu", actual_object_grasp.size(), actual_object_approach.size() );
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
    qnode.saveComponents();
}

void MainWindow::on_buttonAddSlot_clicked(bool check)
{
    bool ok;

    std::string slot_name = ui.editSlotName->text().toStdString();
    std::string max_obj   = ui.editMaxObject->text().toStdString();
    int num_max_obj       = ui.editMaxObject->text().toInt(&ok);
    std::string group_name= ui.editGroupName->text().toStdString();

    if ( !slot_name.empty() )
    {
        if ( !group_name.empty())
        {
            if ( !max_obj.empty() )
            {
                if (init_slot_final)
                {
                    if ( ok )
                    {
                        if ( !init_slot_leave )
                        {
                            actual_slot_leave = actual_slot_approach;
                        }
                        for ( int i = 0; i < ui.slotList->model()->rowCount(); i++ )
                        {
                            if ( !slot_name.compare( ui.slotList->model()->data( ui.slotList->model()->index( i, 0 ), 0).toString().toStdString() ) )
                            {
                                ROS_ERROR("Slot just set");
                                plotMsg("There is another slot with this name");
                                ui.editSlotName->clear();
                                return;
                            }
                        }
                        if ( !qnode.loadNewGroup( group_name ) )
                        {
                            plotMsg("Can't add the group to location manager");
                            init_slot_approach = false;
                            init_slot_final = false;
                            ui.buttonAddApproachSlot->setEnabled(true);
                            ui.buttonRemoveApproachSlot->setEnabled(false);
                            ui.buttonAddFinalPositionSlot->setEnabled(true);
                            ui.buttonRemoveFinalPositionSlot->setEnabled(false);
                            ui.buttonAddLeavePositionSlot->setEnabled(true);
                            ui.buttonRemoveLeavePositionSlot->setEnabled(false);
                            return;
                        }
                        manipulation_slot slt;
                        slt.name           = slot_name;
                        slt.group          = group_name;
                        slt.approach       = actual_slot_approach;
                        slt.leave          = actual_slot_leave;
                        slt.location_      = actual_slot_final_position;
                        slt.max_objects    = num_max_obj;
                        slt.frame          = qnode.base_frame;
                        if ( !qnode.loadNewSlot( slt ) )
                        {
                            plotMsg("Can't add the slot to location manager");
                            init_slot_approach = false;
                            init_slot_final = false;
                            ui.buttonAddApproachSlot->setEnabled(true);
                            ui.buttonRemoveApproachSlot->setEnabled(false);
                            ui.buttonAddFinalPositionSlot->setEnabled(true);
                            ui.buttonRemoveFinalPositionSlot->setEnabled(false);
                            ui.buttonAddLeavePositionSlot->setEnabled(true);
                            ui.buttonRemoveLeavePositionSlot->setEnabled(false);
                            return;
                        }
                        else
                        {
                            qnode.addSlot(slt);
                            ui.editSlotName->clear();
                            ui.editMaxObject->clear();
                            ui.editGroupName->clear();
                            init_slot_approach = false;
                            init_slot_final = false;
                            ui.buttonAddApproachSlot->setEnabled(true);
                            ui.buttonRemoveApproachSlot->setEnabled(false);
                            ui.buttonAddFinalPositionSlot->setEnabled(true);
                            ui.buttonRemoveFinalPositionSlot->setEnabled(false);
                            ui.buttonAddLeavePositionSlot->setEnabled(true);
                            ui.buttonRemoveLeavePositionSlot->setEnabled(false);
                            if ( ui.comboActionType->currentIndex() == 2 )
                            {
                                qnode.writeGroups();
                            }
                            plotMsg("Added slot and group to location manager");
                        }
                    }
                    else
                    {
                        ROS_ERROR("Max number isn't a number");
                        plotMsg("Max number isn't a number");
                        ui.editMaxObject->clear();
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
    qnode.saveComponents();
}

void MainWindow::on_buttonAddBox_clicked(bool check)
{
    std::string box_name = ui.editBoxName->text().toStdString();
    if ( init_box_final )
    {
        if ( !box_name.empty() )
        {
            if ( !init_box_leave )
            {
                actual_box_leave = actual_box_approach;
            }
            for ( int i = 0; i < ui.boxList->model()->rowCount(); i++ )
            {
                if ( !box_name.compare( ui.boxList->model()->data( ui.boxList->model()->index( i, 0 ),0).toString().toStdString() ) )
                {
                    ROS_ERROR("Box just set");
                    plotMsg("There is another box with this name");
                    ui.editBoxName->clear();
                    return;
                }
            }
            box bx;
            bx.name      = box_name;
            bx.location_ = actual_box_final;
            bx.approach  = actual_box_approach;
            bx.leave     = actual_box_leave;
            bx.frame     = qnode.base_frame;
            if ( !qnode.loadNewBox( bx ) )
            {
                plotMsg("Can't add the box to location manager");
                init_box_approach = false;
                init_box_final = false;
                ui.buttonAddApproachBox->setEnabled(true);
                ui.buttonRemoveApproachBox->setEnabled(false);
                ui.buttonAddFinalBox->setEnabled(true);
                ui.buttonRemoveFinalBox->setEnabled(false);
                ui.buttonAddLeavePositionBox->setEnabled(true);
                ui.buttonRemoveLeavePositionBox->setEnabled(false);
                return;
            }
            else
            {
                qnode.addBox( bx );
                ui.editBoxName->clear();
                init_box_approach = false;
                init_box_final = false;
                ui.buttonAddApproachBox->setEnabled(true);
                ui.buttonRemoveApproachBox->setEnabled(false);
                ui.buttonAddFinalBox->setEnabled(true);
                ui.buttonRemoveFinalBox->setEnabled(false);
                ui.buttonAddLeavePositionBox->setEnabled(true);
                ui.buttonRemoveLeavePositionBox->setEnabled(false);
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
    qnode.saveComponents();
}

void MainWindow::on_buttonAddLocationChanges_clicked(bool check)
{
    go_to_location loc;

    QModelIndex index = ui.listLocationModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6;
    loc.location_.pos.origin_x    = ui.editLocationPositionX   ->text().toDouble(&ok0);
    loc.location_.pos.origin_y    = ui.editLocationPositionY   ->text().toDouble(&ok1);
    loc.location_.pos.origin_z    = ui.editLocationPositionZ   ->text().toDouble(&ok2);
    loc.location_.quat.rotation_x = ui.editLocationQuaternionX ->text().toDouble(&ok3);
    loc.location_.quat.rotation_y = ui.editLocationQuaternionY ->text().toDouble(&ok4);
    loc.location_.quat.rotation_z = ui.editLocationQuaternionZ ->text().toDouble(&ok5);
    loc.location_.quat.rotation_w = ui.editLocationQuaternionW ->text().toDouble(&ok6);
    loc.frame                     = ui.editLocationFrame       ->text().toStdString();
    loc.name                      = ui.listLocationModify      ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6)
    {
        plotMsg("One or more number aren't double");
    }
    else
    {
        qnode.addLocationChanges( index.row(), loc);
    }
    qnode.saveComponents();
}

void MainWindow::on_buttonAddSlotChanges_clicked(bool check)
{
    manipulation_slot slt;

    QModelIndex index = ui.listSlotModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10;
    slt.location_.pos.origin_x    = ui.editSlotPositionX   ->text().toDouble(&ok0);
    slt.location_.pos.origin_y    = ui.editSlotPositionY   ->text().toDouble(&ok1);
    slt.location_.pos.origin_z    = ui.editSlotPositionZ   ->text().toDouble(&ok2);
    slt.location_.quat.rotation_x = ui.editSlotQuaternionX ->text().toDouble(&ok3);
    slt.location_.quat.rotation_y = ui.editSlotQuaternionY ->text().toDouble(&ok4);
    slt.location_.quat.rotation_z = ui.editSlotQuaternionZ ->text().toDouble(&ok5);
    slt.location_.quat.rotation_w = ui.editSlotQuaternionW ->text().toDouble(&ok6);
    slt.approach.origin_x         = ui.editSlotApproachX   ->text().toDouble(&ok7);
    slt.approach.origin_y         = ui.editSlotApproachY   ->text().toDouble(&ok8);
    slt.approach.origin_z         = ui.editSlotApproachZ   ->text().toDouble(&ok9);
    slt.max_objects               = ui.editSlotMaxObjects  ->text().toInt(&ok10);
    slt.frame                     = ui.editSlotFrame       ->text().toStdString();
    slt.group                     = ui.editSlotGroup       ->text().toStdString();
    slt.name                      = ui.listSlotModify      ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 || !ok10 )
    {
        plotMsg("One or more number aren't numers");
    }
    else
    {
        qnode.addSlotChanges( index.row(), slt);
    }
    qnode.saveComponents();
}

void MainWindow::on_buttonAddBoxChanges_clicked(bool check)
{
    box bx;

    QModelIndex index = ui.listBoxModify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9;
    bx.location_.pos.origin_x    = ui.editBoxPositionX   ->text().toDouble(&ok0);
    bx.location_.pos.origin_y    = ui.editBoxPositionY   ->text().toDouble(&ok1);
    bx.location_.pos.origin_z    = ui.editBoxPositionZ   ->text().toDouble(&ok2);
    bx.location_.quat.rotation_x = ui.editBoxQuaternionX ->text().toDouble(&ok3);
    bx.location_.quat.rotation_y = ui.editBoxQuaternionY ->text().toDouble(&ok4);
    bx.location_.quat.rotation_z = ui.editBoxQuaternionZ ->text().toDouble(&ok5);
    bx.location_.quat.rotation_w = ui.editBoxQuaternionW ->text().toDouble(&ok6);
    bx.approach.origin_x         = ui.editBoxApproachX   ->text().toDouble(&ok7);
    bx.approach.origin_y         = ui.editBoxApproachY   ->text().toDouble(&ok8);
    bx.approach.origin_z         = ui.editBoxApproachZ   ->text().toDouble(&ok9);
    bx.frame                     = ui.editBoxFrame        ->text().toStdString();
    bx.name                      = ui.listBoxModify       ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        plotMsg("One or more number aren't numers");
    }
    else
    {
        qnode.addBoxChanges( index.row(), bx);
    }
    qnode.saveComponents();
}

void MainWindow::on_buttonAddObjectChanges_clicked(bool check)
{
    QModelIndex index = ui.listObjectModify->currentIndex();
    int index2 = ui.comboGraspNumber->currentIndex();

    object_type obj = qnode.returnObjectInfo( index.row() );

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9;

    obj.approach[index2].origin_x     = ui.editObjectApproachX   ->text().toDouble(&ok7);
    obj.approach[index2].origin_y     = ui.editObjectApproachY   ->text().toDouble(&ok8);
    obj.approach[index2].origin_z     = ui.editObjectApproachZ   ->text().toDouble(&ok9);
    obj.grasp[index2].pos.origin_x    = ui.editObjectPositionX   ->text().toDouble(&ok0);
    obj.grasp[index2].pos.origin_y    = ui.editObjectPositionY   ->text().toDouble(&ok1);
    obj.grasp[index2].pos.origin_z    = ui.editObjectPositionZ   ->text().toDouble(&ok2);
    obj.grasp[index2].quat.rotation_x = ui.editObjectQuaternionX ->text().toDouble(&ok3);
    obj.grasp[index2].quat.rotation_y = ui.editObjectQuaternionY ->text().toDouble(&ok4);
    obj.grasp[index2].quat.rotation_z = ui.editObjectQuaternionZ ->text().toDouble(&ok5);
    obj.grasp[index2].quat.rotation_w = ui.editObjectQuaternionW ->text().toDouble(&ok6);
    obj.tool[index2]                  = ui.editObjectTool         ->text().toStdString();
    obj.type                          = ui.listObjectModify       ->model()->data( index ).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        plotMsg("One or more number aren't double");
    }
    else
    {
        qnode.addObjectChanges( index.row(), obj);
    }
    qnode.saveComponents();
}

void MainWindow::saveActions()
{
    if ( qnode.saveActions() )
    {
        plotMsg("The save are done.");
    }
    else
    {
        plotMsg("There is some problem with the save");
    }
}

void MainWindow::on_buttonLoadTF_clicked(bool chack)
{
    qnode.loadTF();

    QString tf;
    ui.TfList->clear();
    std::string tf_name;
    for ( int i = 0; i < qnode.TFs.size(); i++ )
    {
        std::size_t found = qnode.TFs[i].find( tf_name_space.c_str() );
        if ( found != std::string::npos )
        {
            tf_name = qnode.TFs[i];
            found = tf_name.find("/",2);
            tf_name.erase( 0, found+1 );
            tf = QString::fromStdString(tf_name);
            ui.TfList->addItem(tf);
        }
    }
}

void MainWindow::on_buttonLoad_clicked(bool chack)
{
    qnode.writeParam(1);
}

void MainWindow::on_buttonCopyGrasp_clicked(bool chack)
{
    int index  = ui.listObjectModify->currentIndex().row();
    int index2 = ui.comboGraspNumber->currentIndex();

    qnode.addObjectCopyGrasp( index, index2);
    QString qstr = QString::fromStdString( std::to_string( ui.comboGraspNumber->count() ) );
    ui.comboGraspNumber->addItem( qstr );
    actual_object_to_modify = qnode.returnObjectInfo(index);
}

void MainWindow::on_buttonResetLocationInfo_clicked(bool chack)
{
    if ( !ui.listLocationModify->selectionModel()->selectedIndexes().empty() )
    {
        resetLocation( ui.listLocationModify->currentIndex().row() );
    }
}

void MainWindow::resetLocation(int index)
{
    ui.editLocationFrame       ->clear();
    ui.editLocationPositionX   ->clear();
    ui.editLocationPositionY   ->clear();
    ui.editLocationPositionZ   ->clear();
    ui.editLocationQuaternionW ->clear();
    ui.editLocationQuaternionX ->clear();
    ui.editLocationQuaternionY ->clear();
    ui.editLocationQuaternionZ ->clear();

    go_to_location loc = qnode.returnLocationInfo(index);
    QString frame_ = QString::fromStdString( loc.frame);
    QString pos_x  = QString::fromStdString( std::to_string(loc.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(loc.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(loc.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(loc.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(loc.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(loc.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(loc.location_.quat.rotation_w) );
    QString name_  = QString::fromStdString( loc.name );

    ui.editLocationFrame       ->insert(frame_);
    ui.editLocationPositionX   ->insert(pos_x);
    ui.editLocationPositionY   ->insert(pos_y);
    ui.editLocationPositionZ   ->insert(pos_z);
    ui.editLocationQuaternionW ->insert(quat_w);
    ui.editLocationQuaternionX ->insert(quat_x);
    ui.editLocationQuaternionY ->insert(quat_y);
    ui.editLocationQuaternionZ ->insert(quat_z);
}

void MainWindow::on_buttonResetSlotInfo_clicked(bool chack)
{
    if ( !ui.listSlotModify->selectionModel()->selectedIndexes().empty() )
    {
        resetSlot( ui.listSlotModify->currentIndex().row() );
    }
}

void MainWindow::resetSlot(int index)
{
    ui.editSlotFrame       ->clear();
    ui.editSlotGroup       ->clear();
    ui.editSlotPositionX   ->clear();
    ui.editSlotPositionY   ->clear();
    ui.editSlotPositionZ   ->clear();
    ui.editSlotQuaternionW ->clear();
    ui.editSlotQuaternionX ->clear();
    ui.editSlotQuaternionY ->clear();
    ui.editSlotQuaternionZ ->clear();
    ui.editSlotApproachX   ->clear();
    ui.editSlotApproachY   ->clear();
    ui.editSlotApproachZ   ->clear();
    ui.editSlotMaxObjects  ->clear();
    ui.editSlotLeaveX      ->clear();
    ui.editSlotLeaveY      ->clear();
    ui.editSlotLeaveZ      ->clear();

    manipulation_slot sl = qnode.returnSlotInfo(index);
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
    QString name_  = QString::fromStdString( sl.name );
    QString leav_x = QString::fromStdString( std::to_string(sl.leave.origin_x) );
    QString leav_y = QString::fromStdString( std::to_string(sl.leave.origin_y) );
    QString leav_z = QString::fromStdString( std::to_string(sl.leave.origin_z) );

    ui.editSlotFrame       ->insert(frame_);
    ui.editSlotGroup       ->insert(group_);
    ui.editSlotPositionX   ->insert(pos_x);
    ui.editSlotPositionY   ->insert(pos_y);
    ui.editSlotPositionZ   ->insert(pos_z);
    ui.editSlotQuaternionW ->insert(quat_w);
    ui.editSlotQuaternionX ->insert(quat_x);
    ui.editSlotQuaternionY ->insert(quat_y);
    ui.editSlotQuaternionZ ->insert(quat_z);
    ui.editSlotApproachX   ->insert(appr_x);
    ui.editSlotApproachY   ->insert(appr_y);
    ui.editSlotApproachZ   ->insert(appr_z);
    ui.editSlotMaxObjects  ->insert(max_ob);
    ui.editSlotLeaveX      ->insert(leav_x);
    ui.editSlotLeaveY      ->insert(leav_y);
    ui.editSlotLeaveZ      ->insert(leav_z);

}

void MainWindow::on_buttonResetBoxInfo_clicked(bool chack)
{
    if ( !ui.listBoxModify->selectionModel()->selectedIndexes().empty() )
    {
        resetBox( ui.listBoxModify->currentIndex().row() );
    }
}

void MainWindow::on_buttonUp_pressed      ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, 0, vel, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonDown_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, 0, -vel, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonLeft_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, -vel, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonRight_pressed   ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, vel, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonFront_pressed   ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { -vel, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonBack_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { vel, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockX_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, rot, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockY_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, rot, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockZ_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, 0, rot };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiX_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, -rot, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiY_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, -rot, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiZ_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, 0, -rot };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonUp_released      ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonDown_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonLeft_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonRight_released   ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonFront_released   ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonBack_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockX_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockY_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonClockZ_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiX_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiY_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonAntiZ_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_buttonLoadObjects_clicked(bool check)
{
    std::vector<std::string> object_list = qnode.loadObjectsInManipulation();
    if ( object_list.empty() )
    {
        plotMsg("No objects have been added ");
    }
    else
    {
        std::string str = "Added the objects:\n";
        for ( std::size_t i = 0; i < object_list.size(); i++)
        {
            std::string object_name = object_list[i].erase( 0, object_list[i].find("/")+1 );
            str.append( object_name.c_str() );
            if ( i != object_list.size()-1 )
            {
                str.append(",\n");
            }
            else
            {
                str.append(".");
            }
        }
        plotMsg(str);
    }
}

void MainWindow::on_buttonRunSelectedAction_clicked(bool check)
{
    QModelIndexList indexes =  ui.listRecipe->selectionModel()->selectedIndexes();
    if ( !indexes.empty() )
    {
        n_run_action_clicked_++;
        if ( n_run_action_clicked_ % 2 )
        {
            plotMsg("Did you load the objects?");
        }
        else
        {
            int index = indexes.at(0).row();
            std::string risp = qnode.runSelectedAction(index);
            plotMsg(risp);
        }
    }
    else
    {
        plotMsg("No selected action");
        ui.editActionName->clear();
    }
}

void MainWindow::on_buttonCopyLocation_clicked(bool chack)
{
    if ( !ui.listLocationModify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.listLocationModify->currentIndex().row();
        go_to_location loc = qnode.returnLocationInfo( index );
        loc.name = ui.editNewLocation->text().toStdString();

        if ( !loc.name.empty() )
        {
            if ( !qnode.addLocationCopy( loc ) )
            {
                plotMsg("There is another location whit the same name.");
                ui.editNewLocation->clear();
            }
            else
            {
                ui.editNewLocation->clear();
                if ( ui.comboActionType->currentIndex() == 0)
                {
                    qnode.writeLocations();
                }
            }
        }
        else
        {
            plotMsg("Empty name.");
        }
    }
    else
    {
        plotMsg("No selected location.");
    }
}

void MainWindow::on_buttonCopyObject_clicked(bool chack)
{
    if ( !ui.listObjectModify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.listObjectModify->currentIndex().row();
        object_type obj = qnode.returnObjectInfo( index );
        obj.type = ui.editNewObject->text().toStdString();

        if ( !obj.type.empty() )
        {
            if ( !qnode.addObjectCopy( obj ) )
            {
                plotMsg("There is another object whit the same name.");
                ui.editNewObject->clear();
            }
            else
            {
                ui.editNewObject->clear();
                if ( ui.comboActionType->currentIndex() == 1 )
                {
                    qnode.writeObjects();
                }
            }
        }
        else
        {
            plotMsg("Empty name.");
        }
    }
    else
    {
        plotMsg("No selected object.");
    }
}

void MainWindow::on_buttonCopySlot_clicked(bool chack)
{
    if ( !ui.listSlotModify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.listSlotModify->currentIndex().row();
        manipulation_slot slt = qnode.returnSlotInfo( index );
        slt.name = ui.editNewSlot->text().toStdString();

        if ( !slt.name.empty() )
        {
            if ( !qnode.addSlotCopy( slt ) )
            {
                plotMsg("There is another slot whit the same name.");
                ui.editNewSlot->clear();
            }
            else
            {
                ui.editNewSlot->clear();
                if ( ui.comboActionType->currentIndex() == 2 )
                {
                    qnode.writeGroups();
                }
            }
        }
        else
        {
            plotMsg("Empty name.");
        }
    }
    else
    {
        plotMsg("No selected slot.");
    }
}

void MainWindow::on_buttonCopyBox_clicked(bool chack)
{
    if ( !ui.listBoxModify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.listBoxModify->currentIndex().row();
        box bx = qnode.returnBoxInfo( index );
        bx.name = ui.editNewBox->text().toStdString();

        if ( !bx.name.empty() )
        {
            if ( !qnode.addBoxCopy( bx ) )
            {
                plotMsg("There is another box whit the same name.");
                ui.editNewBox->clear();
            }
            else
            {
                ui.editNewBox->clear();
            }
        }
        else
        {
            plotMsg("Empty name.");
        }
    }
    else
    {
        plotMsg("No selected box.");
    }
}

void MainWindow::resetBox(int index)
{
    ui.editBoxFrame       ->clear();
    ui.editBoxPositionX   ->clear();
    ui.editBoxPositionY   ->clear();
    ui.editBoxPositionZ   ->clear();
    ui.editBoxQuaternionW ->clear();
    ui.editBoxQuaternionX ->clear();
    ui.editBoxQuaternionY ->clear();
    ui.editBoxQuaternionZ ->clear();
    ui.editBoxApproachX   ->clear();
    ui.editBoxApproachY   ->clear();
    ui.editBoxApproachZ   ->clear();
    ui.editBoxLeaveX      ->clear();
    ui.editBoxLeaveY      ->clear();
    ui.editBoxLeaveZ      ->clear();

    box bx = qnode.returnBoxInfo(index);
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
    QString name_  = QString::fromStdString( bx.name );

    ui.editBoxFrame       ->insert(frame_);
    ui.editBoxPositionX   ->insert(pos_x);
    ui.editBoxPositionY   ->insert(pos_y);
    ui.editBoxPositionZ   ->insert(pos_z);
    ui.editBoxQuaternionW ->insert(quat_w);
    ui.editBoxQuaternionX ->insert(quat_x);
    ui.editBoxQuaternionY ->insert(quat_y);
    ui.editBoxQuaternionZ ->insert(quat_z);
    ui.editBoxApproachX   ->insert(appr_x);
    ui.editBoxApproachY   ->insert(appr_y);
    ui.editBoxApproachZ   ->insert(appr_z);
    ui.editBoxLeaveX      ->insert(leav_x);
    ui.editBoxLeaveY      ->insert(leav_y);
    ui.editBoxLeaveZ      ->insert(leav_z);

}

void MainWindow::on_buttonResetObjectInfo_clicked(bool chack)
{
    if ( !ui.listObjectModify->selectionModel()->selectedIndexes().empty() )
    {
        resetObject( ui.listObjectModify->currentIndex().row() );
    }
}

void MainWindow::resetObject(int index)
{
    init_objects = false;
    ui.comboGraspNumber->clear();

    ui.editObjectPositionX   ->clear();
    ui.editObjectPositionY   ->clear();
    ui.editObjectPositionZ   ->clear();
    ui.editObjectQuaternionW ->clear();
    ui.editObjectQuaternionX ->clear();
    ui.editObjectQuaternionY ->clear();
    ui.editObjectQuaternionZ ->clear();
    ui.editObjectApproachX   ->clear();
    ui.editObjectApproachY   ->clear();
    ui.editObjectApproachZ   ->clear();
    ui.editObjectLeaveX      ->clear();
    ui.editObjectLeaveY      ->clear();
    ui.editObjectLeaveZ      ->clear();
    ui.editObjectTool        ->clear();
    ui.editGripperState      ->clear();
    ui.editPreGripperState   ->clear();
    ui.editPostGripperState  ->clear();

    actual_object_to_modify = qnode.returnObjectInfo(index);

    for ( int i = 0; i < actual_object_to_modify.grasp.size(); i++ )
    {
        ui.comboGraspNumber->addItem( QString::fromStdString( std::to_string(i) ) );
    }
    init_objects = true;

    int i = ui.comboGraspNumber->currentIndex();
    QString pos_x  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[i].quat.rotation_w) );
    QString appr_x = QString::fromStdString( std::to_string(actual_object_to_modify.approach[i].origin_x) );
    QString appr_y = QString::fromStdString( std::to_string(actual_object_to_modify.approach[i].origin_y) );
    QString appr_z = QString::fromStdString( std::to_string(actual_object_to_modify.approach[i].origin_z) );
    QString leav_x = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_x) );
    QString leav_y = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_y) );
    QString leav_z = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_z) );
    QString tool   = QString::fromStdString( actual_object_to_modify.tool[i] );
    QString type_  = QString::fromStdString( actual_object_to_modify.type );
    QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify.gripper_force[i] ) );
    QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify.pre_gripper_position[i] ) );
    QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify.post_gripper_position[i] ) );

    ui.editObjectPositionX   ->insert(pos_x);
    ui.editObjectPositionY   ->insert(pos_y);
    ui.editObjectPositionZ   ->insert(pos_z);
    ui.editObjectQuaternionW ->insert(quat_w);
    ui.editObjectQuaternionX ->insert(quat_x);
    ui.editObjectQuaternionY ->insert(quat_y);
    ui.editObjectQuaternionZ ->insert(quat_z);
    ui.editObjectApproachX   ->insert(appr_x);
    ui.editObjectApproachY   ->insert(appr_y);
    ui.editObjectApproachZ   ->insert(appr_z);
    ui.editObjectLeaveX      ->insert(leav_x);
    ui.editObjectLeaveY      ->insert(leav_y);
    ui.editObjectLeaveZ      ->insert(leav_z);
    ui.editObjectTool        ->insert(tool);
    ui.editGripperState      ->insert(gripper_force);
    ui.editPreGripperState   ->insert(pre_position);
    ui.editPostGripperState  ->insert(post_position);
}

void MainWindow::on_checkRobotTF_stateChanged(int state)
{
    if ( state != 0 )
    {
        ui.lateralTab                    ->setEnabled(true);

        ui.checkRobotTF ->setEnabled(false);
        ui.worldTfList  ->setEnabled(false);

        qnode.base_frame = ui.worldTfList->currentText().toStdString();
    }
}

void MainWindow::on_robotList_currentIndexChanged(int index)
{
    qnode.setTargetFrame( index );
}

void MainWindow::on_comboGraspNumber_currentIndexChanged(int index)
{
    ui.editObjectPositionX   ->clear();
    ui.editObjectPositionY   ->clear();
    ui.editObjectPositionZ   ->clear();
    ui.editObjectQuaternionW ->clear();
    ui.editObjectQuaternionX ->clear();
    ui.editObjectQuaternionY ->clear();
    ui.editObjectQuaternionZ ->clear();
    ui.editObjectApproachX   ->clear();
    ui.editObjectApproachY   ->clear();
    ui.editObjectApproachZ   ->clear();
    ui.editObjectLeaveX      ->clear();
    ui.editObjectLeaveY      ->clear();
    ui.editObjectLeaveZ      ->clear();
    ui.editObjectTool        ->clear();
    ui.editGripperState      ->clear();
    ui.editPreGripperState   ->clear();
    ui.editPostGripperState  ->clear();

    if ( init_objects)
    {
        QString pos_x          = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_x) );
        QString pos_y          = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_y) );
        QString pos_z          = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_z) );
        QString quat_x         = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_x) );
        QString quat_y         = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_y) );
        QString quat_z         = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_z) );
        QString quat_w         = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_w) );
        QString appr_x         = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_x) );
        QString appr_y         = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_y) );
        QString appr_z         = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_z) );
        QString tool           = QString::fromStdString( actual_object_to_modify.tool[index] );
        QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify.gripper_force[index] ) );
        QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify.pre_gripper_position[index] ) );
        QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify.post_gripper_position[index] ) );
        QString leav_x         = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_x) );
        QString leav_y         = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_y) );
        QString leav_z         = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_z) );

        ui.editObjectPositionX   ->insert(pos_x);
        ui.editObjectPositionY   ->insert(pos_y);
        ui.editObjectPositionZ   ->insert(pos_z);
        ui.editObjectQuaternionW ->insert(quat_w);
        ui.editObjectQuaternionX ->insert(quat_x);
        ui.editObjectQuaternionY ->insert(quat_y);
        ui.editObjectQuaternionZ ->insert(quat_z);
        ui.editObjectApproachX   ->insert(appr_x);
        ui.editObjectApproachY   ->insert(appr_y);
        ui.editObjectApproachZ   ->insert(appr_z);
        ui.editObjectLeaveX      ->insert(leav_x);
        ui.editObjectLeaveY      ->insert(leav_y);
        ui.editObjectLeaveZ      ->insert(leav_z);
        ui.editObjectTool        ->insert(tool);
        ui.editGripperState      ->insert(gripper_force);
        ui.editPreGripperState   ->insert(pre_position);
        ui.editPostGripperState  ->insert(post_position);
    }
}

void MainWindow::on_comboActionType_currentIndexChanged(int index)
{
    if ( index == 0 )
    {
        qnode.writeLocations();
    }
    else if ( index == 1)
    {
        qnode.writeObjects();
    }
    else if ( index == 2 )
    {
        qnode.writeGroups();
    }
}

void MainWindow::on_comboConfiguration_currentIndexChanged(int index)
{
    if ( index == 0 )
    {
        qnode.activeConfiguration("watch");
    }
    else if ( index == 1 )
    {
        qnode.activeConfiguration("manual_guidance");
    }
    else if ( index == 2 )
    {
        qnode.activeConfiguration("lin_xyz_manual_guidance");
    }
    else if ( index == 3 )
    {
        qnode.activeConfiguration("rot_xyz_manual_guidance");
    }
    else if ( index == 4 )
    {
        qnode.activeConfiguration("cart_teleop");
    }
    else
    {
        plotMsg("There is a problem with configuration");
    }
    ui.comboConfiguration2->setCurrentIndex( index );
}

void MainWindow::on_comboConfiguration2_currentIndexChanged(int index)
{
    ui.comboConfiguration->setCurrentIndex( index );
}

void MainWindow::on_comboRefFrame_currentIndexChanged(int index)
{
    qnode.frame_id.clear();
    qnode.frame_id.append("/");
    qnode.frame_id.append( ui.comboRefFrame->currentText().toStdString() );
}

void MainWindow::on_TfList_currentIndexChanged(int index)
{
  std::string name = ui.TfList->currentText().toStdString();
  std::size_t found  = name.find( "/" );
  if ( found != std::string::npos)
  {
    name.erase( found, name.size() );
  }
  ui.editObjectName->setText(QString::fromStdString(name));
}

void MainWindow::on_listLocationModify_pressed(const QModelIndex &index)
{
    resetLocation( index.row() );
}

void MainWindow::on_listBoxModify_pressed(const QModelIndex &index)
{
    resetBox( index.row() );
}

void MainWindow::on_listSlotModify_pressed(const QModelIndex &index)
{
    resetSlot( index.row() );
}

void MainWindow::on_listObjectModify_pressed(const QModelIndex &index)
{
    resetObject( index.row() );
}

void MainWindow::on_gripperPercentage_valueChanged (int value)
{
  ui.gripperPercentage->setValue(value);
  ui.gripperPercentage2->setValue(value);
  std::string str = std::to_string(value);
  QString qstr = QString::fromStdString(str);
  ui.openGripperLabel->setText(qstr);
  ui.openGripperLabel2->setText(qstr);
}

void MainWindow::on_gripperForcePercentage_valueChanged (int value)
{
    ui.gripperForcePercentage->setValue(value);
    ui.gripperForcePercentage2->setValue(value);
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui.gripperForceLabel->setText(qstr);
    ui.gripperForceLabel2->setText(qstr);
}

void MainWindow::on_gripperPercentage2_valueChanged (int value)
{
    ui.gripperPercentage->setValue(value);
}

void MainWindow::on_gripperForcePercentage2_valueChanged (int value)
{
    ui.gripperForcePercentage->setValue(value);
}

void MainWindow::on_velocitySlider_valueChanged(int value)
{
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui.velocityLabel->setText(qstr);
    perc_vel = value;
}

void MainWindow::on_lateralTab_currentChanged(int index)
{
    on_buttonLoadTF_clicked(false);
}

void MainWindow::on_buttonRemoveElement_clicked(bool check )
{
    QModelIndexList indexes =  ui.listRecipe->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model = ui.listRecipe->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            model->removeRow(index.row());
            indexes =  ui.listRecipe->selectionModel()->selectedIndexes();
        }
        return;
    }

    plotMsg("There isn't a selected position");
}

void MainWindow::on_buttonLoadRecipe_clicked(bool check)
{
    std::vector<std::string> recipes_names = qnode.loadRecipesParam();
    for ( int i = 0; i < recipes_names.size(); i++)
    {
        bool presence = false;
        for ( int j = 0; j < ui.recipeBox->count(); j++)
        {
            if ( !recipes_names[i].compare( ui.recipeBox->itemText(j).toStdString() ))
            {
                presence = true;
            }
        }
        if ( !presence )
        {
            ui.recipeBox->addItem( QString::fromStdString( recipes_names[i] ));
        }
    }
}

void MainWindow::on_buttonWriteRecipe_clicked(bool check)
{
    int ind = ui.recipeBox->currentIndex();
    if ( ui.recipeBox->count() != 0 )
    {
        qnode.writeRecipe( ind );
    }
}

void MainWindow::on_buttonLoadActions_clicked  ( bool check )
{
    qnode.writeParam(2);
}

void MainWindow::saveRecipe()
{

    if ( qnode.saveRecipe() )
    {
        plotMsg("The recipe is saved.");
        ui.editRecipeName->clear();
    }
    else
    {
        plotMsg("There is some problem with the save");
        return;
    }
}

void MainWindow::on_listPlace_pressed(const QModelIndex &index)
{
    qnode.addSlotGroups( index.row() );
    place pl = qnode.returnPlaceInfo( index.row() );
    QString qstr = QString::fromStdString( pl.description );
    ui.editActionDescription->setText( qstr );
    ui.listGoTo->clearSelection();
    ui.listPick->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
    else if ( pl.agents[0] == "robot" )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( pl.agents[0] == "human" )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_listPick_pressed(const QModelIndex &index)
{
    qnode.addObjectType( index.row() );
    pick pk = qnode.returnPickInfo( index.row() );
    QString qstr = QString::fromStdString( pk.description );
    ui.editActionDescription2->setText( qstr );
    ui.listPlace->clearSelection();
    ui.listGoTo->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
    else if ( pk.agents[0] == "robot" )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( pk.agents[0] == "human" )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_listGoTo_pressed(const QModelIndex &index)
{
    qnode.addLocationInfo( index.row() );
    go_to_action gt = qnode.returnGoToInfo( index.row() );
    QString qstr = QString::fromStdString( gt.description );
    ui.editActionDescription2->setText( qstr );
    ui.listPlace->clearSelection();
    ui.listPick->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
    else if ( gt.agents[0] == "robot" )
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(true);
    }
    else if ( gt.agents[0] == "human" )
    {
        ui.checkHumanInfo2->setChecked(true);
        ui.checkRobotInfo2->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo2->setChecked(false);
        ui.checkRobotInfo2->setChecked(false);
    }
}

void MainWindow::on_goToList_pressed(const QModelIndex &index)
{
    qnode.addSecondLocationInfo( index.row() );
    go_to_action gt = qnode.returnGoToInfo( index.row() );
    QString qstr = QString::fromStdString( gt.description );
    ui.editActionDescription->setText( qstr );
    ui.placeList->clearSelection();
    ui.pickList->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
    else if ( gt.agents[0] == "robot" )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( gt.agents[0] == "human" )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_placeList_pressed(const QModelIndex &index)
{
    qnode.addSecondSlotGroups( index.row() );
    place pl = qnode.returnPlaceInfo( index.row() );
    QString qstr = QString::fromStdString( pl.description );
    ui.editActionDescription->setText( qstr );
    ui.goToList->clearSelection();
    ui.pickList->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
    else if ( pl.agents[0] == "robot" )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( pl.agents[0] == "human" )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_pickList_pressed (const QModelIndex &index)
{
    qnode.addSecondObjectType( index.row() );
    pick pk = qnode.returnPickInfo( index.row() );
    QString qstr = QString::fromStdString( pk.description );
    ui.editActionDescription->setText( qstr );
    ui.placeList->clearSelection();
    ui.goToList->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
    else if ( pk.agents[0] == "robot" )
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(true);
    }
    else if ( pk.agents[0] == "human" )
    {
        ui.checkHumanInfo->setChecked(true);
        ui.checkRobotInfo->setChecked(false);
    }
    else
    {
        ui.checkHumanInfo->setChecked(false);
        ui.checkRobotInfo->setChecked(false);
    }
}

void MainWindow::on_buttonAddLeavePositionSlot_clicked(bool check)
{
    if ( init_slot_final )
    {
        location actual_approach = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
        double dist_x = actual_approach.pos.origin_x - actual_slot_final_position.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_slot_final_position.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_slot_final_position.pos.origin_z;
        Eigen::Vector3d dist_slot(dist_x,dist_y,dist_z);
        double w = actual_slot_final_position.quat.rotation_w;
        double x = actual_slot_final_position.quat.rotation_x;
        double y = actual_slot_final_position.quat.rotation_y;
        double z = actual_slot_final_position.quat.rotation_z;
        Eigen::Quaterniond quat_slot( w, x, y, z);
        Eigen::MatrixXd matrix(quat_slot.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_slot;
        actual_slot_leave.origin_x = dist_tool[0];
        actual_slot_leave.origin_y = dist_tool[1];
        actual_slot_leave.origin_z = dist_tool[2];
        init_slot_leave = true;
        ui.buttonAddLeavePositionSlot->setEnabled(false);
        ui.buttonRemoveLeavePositionSlot->setEnabled(true);
    }
    else
    {
        plotMsg("Final position is not set.");
    }
}

void MainWindow::on_buttonAddLeavePositionBox_clicked(bool check)
{
    if ( init_box_final )
    {
        location actual_approach = qnode.returnPosition(qnode.base_frame, qnode.target_frame);
        double dist_x = actual_approach.pos.origin_x - actual_box_final.pos.origin_x;
        double dist_y = actual_approach.pos.origin_y - actual_box_final.pos.origin_y;
        double dist_z = actual_approach.pos.origin_z - actual_box_final.pos.origin_z;
        Eigen::Vector3d dist_box(dist_x,dist_y,dist_z);
        double w = actual_box_final.quat.rotation_w;
        double x = actual_box_final.quat.rotation_x;
        double y = actual_box_final.quat.rotation_y;
        double z = actual_box_final.quat.rotation_z;
        Eigen::Quaterniond quat_box( w, x, y, z);
        Eigen::MatrixXd matrix(quat_box.toRotationMatrix());
        Eigen::Vector3d dist_tool;
        dist_tool = matrix.inverse() * dist_box;
        actual_box_leave.origin_x = dist_tool[0];
        actual_box_leave.origin_y = dist_tool[1];
        actual_box_leave.origin_z = dist_tool[2];
        init_box_leave = true;
        ui.buttonAddLeavePositionBox->setEnabled(false);
        ui.buttonRemoveLeavePositionBox->setEnabled(true);
    }
    else
    {
        plotMsg("Final position is not set.");
    }
}

void MainWindow::on_buttonRemoveLeavePositionSlot_clicked(bool check)
{
    init_slot_leave = false;
    ui.buttonAddLeavePositionSlot->setEnabled(true);
    ui.buttonRemoveLeavePositionSlot->setEnabled(false);
}

void MainWindow::on_buttonRemoveLeavePositionBox_clicked(bool check)
{
    init_box_leave = false;
    ui.buttonAddLeavePositionBox->setEnabled(true);
    ui.buttonRemoveLeavePositionBox->setEnabled(false);
}

void MainWindow::plotMsg(std::string msg)
{
    QMessageBox msgBox;
    msgBox.setText(msg.c_str());
    msgBox.exec();
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
        ui.goToList -> scrollToBottom();
        ui.placeList -> scrollToBottom();
        ui.pickList  -> scrollToBottom();
        ui.listGoTo -> scrollToBottom();
        ui.listPlace -> scrollToBottom();
        ui.listPick  -> scrollToBottom();
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

