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
    ui.go_to_list             ->setModel(qnode.loggingModelGoTo());
    ui.place_list             ->setModel(qnode.loggingModelPlace());
    ui.pick_list              ->setModel(qnode.loggingModelPick());
    ui.object_list            ->setModel(qnode.loggingModelObject());
    ui.slot_list              ->setModel(qnode.loggingModelSlot());
    ui.group_list             ->setModel(qnode.loggingModelGroup());
    ui.box_list               ->setModel(qnode.loggingModelBox());
    ui.locations_list         ->setModel(qnode.loggingModelLocation());
    ui.list_location_modify   ->setModel(qnode.loggingModelLocationModify());
    ui.list_box_modify        ->setModel(qnode.loggingModelBoxModify());
    ui.list_slot_modify       ->setModel(qnode.loggingModelSlotModify());
    ui.list_object_modify     ->setModel(qnode.loggingModelObjectModify());
    ui.component_list         ->setModel(qnode.loggingModelComponents());
    ui.list_info_action       ->setModel(qnode.loggingModelInfoAction());
    ui.list_go_to             ->setModel(qnode.loggingModelSecondGoto());
    ui.list_place             ->setModel(qnode.loggingModelSecondPlace());
    ui.list_pick              ->setModel(qnode.loggingModelSecondPick());
    ui.list_recipe            ->setModel(qnode.loggingModelRecipe());

    ui.list_action_components ->setModel(qnode.loggingModelActionComponents());

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.object_list    ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.slot_list      ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.group_list     ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.box_list       ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.locations_list ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.component_list ->setSelectionMode(QAbstractItemView::MultiSelection);

    ui.list_recipe ->setDragDropMode(QAbstractItemView::DragDrop);
//    ui.list_recipe->model()->doSetSupportedDragActions(Qt::MoveAction);

    ui.list_go_to->setDragEnabled(true);
    ui.list_place->setDragEnabled(true);
    ui.list_pick ->setDragEnabled(true);

    ui.button_remove_approach_slot       ->setEnabled(false);
    ui.button_remove_final_position_slot ->setEnabled(false);
    ui.button_remove_approach_box        ->setEnabled(false);
    ui.button_remove_final_box           ->setEnabled(false);
    ui.button_remove_leave_position_slot ->setEnabled(false);
    ui.button_remove_leave_position_box  ->setEnabled(false);
    ui.button_save_components            ->setEnabled(false);
    ui.lateral_tab                       ->setEnabled(false);
    ui.check_human_info                  ->setEnabled(false);
    ui.check_robot_info                  ->setEnabled(false);
    ui.check_human_info_2                ->setEnabled(false);
    ui.check_robot_info_2                ->setEnabled(false);

    ui.go_to_list             ->setEditTriggers(QListView::NoEditTriggers);
    ui.place_list             ->setEditTriggers(QListView::NoEditTriggers);
    ui.pick_list              ->setEditTriggers(QListView::NoEditTriggers);
    ui.object_list            ->setEditTriggers(QListView::NoEditTriggers);
    ui.slot_list              ->setEditTriggers(QListView::NoEditTriggers);
    ui.group_list             ->setEditTriggers(QListView::NoEditTriggers);
    ui.box_list               ->setEditTriggers(QListView::NoEditTriggers);
    ui.locations_list         ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_location_modify   ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_box_modify        ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_slot_modify       ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_object_modify     ->setEditTriggers(QListView::NoEditTriggers);
    ui.component_list         ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_go_to             ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_place             ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_pick              ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_recipe            ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_info_action       ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_action_components ->setEditTriggers(QListView::NoEditTriggers);

    ui.edit_location_frame     ->setReadOnly(true);
    ui.edit_slot_frame         ->setReadOnly(true);
    ui.edit_slot_group         ->setReadOnly(true);
    ui.edit_box_frame          ->setReadOnly(true);
    ui.edit_object_tool        ->setReadOnly(true);
    ui.edit_action_description ->setReadOnly(true);
    ui.edit_gripper_state      ->setReadOnly(true);
    ui.edit_object_name        ->setReadOnly(true);

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
        ui.world_TF_list->addItem(tf);
        ui.combo_ref_frame->addItem(tf);
    }
    on_button_load_TF_clicked(false);
    for ( int i = 0; i < qnode.robots.size(); i++)
    {
        tf = QString::fromStdString(qnode.robots[i]);
        ui.robot_list->addItem(tf);
    }

    qnode.frame_id.clear();
    qnode.frame_id.append("/");
    qnode.frame_id.append( ui.combo_ref_frame->currentText().toStdString() );

    qnode.write_param(1);
    qnode.initial_add_components_in_manipulation();
    qnode.write_param(2);
    qnode.write_locations();
    std::vector<std::string> recipes_names =qnode.load_recipes_param();
    if ( !recipes_names.empty() )
    {
        for ( int i = 0; i < recipes_names.size(); i++)
        {
            ui.recipe_box->addItem( QString::fromStdString(recipes_names[i]) );
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

void MainWindow::add_go_to( int state )
{
    std::string go_to_name = ui.edit_action_name->text().toStdString();
    std::string description = ui.edit_description->toPlainText().toStdString();
    std::vector<std::string> locations;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.component_list->selectionModel()->selectedIndexes();
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
                    locations.push_back( qnode.return_location_list_text( indexes.at(i).row() ) );
                }
                if ( !qnode.add_go_to(go_to_name,locations,description,agents_) )
                {
                    QMessageBox msgBox;
                    msgBox.setText("There is another action with the same name or locations.");
                    msgBox.exec();
                    ui.edit_action_name->clear();
                }
                else
                {
                    ui.edit_action_name->clear();
                    ui.edit_description->clear();
                    ui.component_list->clearSelection();
                }
            }
            else
            {
                QMessageBox msgBox;
                msgBox.setText("Empty name");
                msgBox.exec();
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("There isn't any location selected.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Empty agent");
        msgBox.exec();
    }
}

void MainWindow::add_place( int state )
{
    std::string place_name = ui.edit_action_name->text().toStdString();
    std::string description = ui.edit_description->toPlainText().toStdString();
    std::vector<std::string> groups;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.component_list->selectionModel()->selectedIndexes();
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
                    groups.push_back( qnode.return_group_list_text( indexes.at(i).row() ) );
                }
                if ( !qnode.add_place(place_name,groups,description,agents_) )
                {
                    QMessageBox msgBox;
                    msgBox.setText("There is another action with the same name or groups.");
                    msgBox.exec();
                    ui.edit_action_name->clear();
                }
                else
                {
                    ui.edit_action_name->clear();
                    ui.edit_description->clear();
                    ui.component_list->clearSelection();
                }
            }
            else
            {
                QMessageBox msgBox;
                msgBox.setText("Empty name");
                msgBox.exec();
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("There isn't any slot group selected.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Empty agent");
        msgBox.exec();
    }
}

void MainWindow::add_pick( int state )
{
    std::string pick_name = ui.edit_action_name->text().toStdString();
    std::string description = ui.edit_description->toPlainText().toStdString();
    std::vector<std::string> objects;
    std::vector<std::string> agents_;

    QModelIndexList indexes =  ui.component_list->selectionModel()->selectedIndexes();
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
                    objects.push_back(qnode.return_object_list_text( indexes.at(i).row() ) );
                }
                if ( !qnode.add_pick(pick_name,objects,description,agents_) )
                {
                    QMessageBox msgBox;
                    msgBox.setText("There is another action with the same name or objects.");
                    msgBox.exec();
                    ui.edit_action_name->clear();
                }
                else
                {
                    ui.edit_action_name->clear();
                    ui.edit_description->clear();
                    ui.component_list->clearSelection();
                }
            }
            else
            {
                QMessageBox msgBox;
                msgBox.setText("There isn't any selected object type");
                msgBox.exec();
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Empty name");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Empty agent");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_action_clicked(bool check)
{
    std::string type = ui.combo_action_type->currentText().toStdString();
    int state = 0;
    if ( ui.check_robot->checkState() == Qt::Checked && ui.check_human->checkState() == Qt::Checked )
    {
        state = 3;
    }
    else if ( ui.check_robot->checkState() == Qt::Checked )
    {
        state = 1;
    }
    else if ( ui.check_human->checkState() == Qt::Checked )
    {
        state = 2;
    }
    else
    {
        state = 0;
    }
    if ( !type.compare("Go to"))
    {
        add_go_to( state );
    }
    else if ( !type.compare("Pick"))
    {
        add_pick( state );
    }
    else if ( !type.compare("Place") )
    {
        add_place( state );
    }
}

void MainWindow::on_button_add_grasp_clicked(bool check)
{
    std::string actual_base_frame = ui.TF_list->currentText().toStdString();
    actual_object_grasp.push_back(qnode.return_position(actual_base_frame, qnode.target_frame));
    actual_tool_grasp.push_back(qnode.target_frame);
    num_grasp++;
    std::string str = "grasp";
    str.append(std::to_string(num_grasp));
    QString grasp;
    grasp.append(str.c_str());
    ui.grasp_list->addItem(grasp);
    ui.grasp_list->setCurrentIndex(ui.grasp_list->count()-1);
    position pos;
    pos.origin_x = 0.0;
    pos.origin_y = 0.0;
    pos.origin_z = -0.10;
    actual_object_approach.push_back(pos);
    actual_object_leave.push_back(pos);
    actual_pre_gripper_position.push_back(max_gripper_position);
    actual_approach_gripper_position.push_back(std::nan("1"));
//    actual_post_gripper_position.push_back(qnode.return_gripper_position());
    actual_post_gripper_position.push_back(0.0);
    actual_gripper_grasp_force.push_back(actual_gripper_force);
    actual_tool_approach.push_back(qnode.target_frame);
    actual_tool_leave.push_back(qnode.target_frame);
}

void MainWindow::on_button_set_approach_clicked(bool check)
{
    if ( ui.grasp_list->count() == 0 )
    {
        return;
    }
    int index = ui.grasp_list->currentIndex();
    std::string actual_base_frame = ui.TF_list->currentText().toStdString();
    location actual_approach = qnode.return_position(actual_base_frame, qnode.target_frame);
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
//    actual_approach_gripper_position[index] = qnode.return_gripper_position();
    actual_approach_gripper_position[index] = 85;
    actual_pre_gripper_position[index] = actual_approach_gripper_position[index];
    actual_tool_approach[index] = qnode.target_frame;
}

void MainWindow::on_button_set_leave_clicked(bool check)
{
    if ( ui.grasp_list->count() == 0 )
    {
        return;
    }
    int index = ui.grasp_list->currentIndex();
    std::string actual_base_frame = ui.TF_list->currentText().toStdString();
    location actual_leave = qnode.return_position(actual_base_frame, qnode.target_frame);
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
//      actual_pre_gripper_position[index] = qnode.return_gripper_position();
      actual_pre_gripper_position[index] = 85;
    }
    actual_tool_leave[index] = qnode.target_frame;
}


void MainWindow::on_button_add_approach_slot_clicked(bool check)
{
    actual_slot_approach = qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_slot_approach = true;
    ui.button_add_approach_slot->setEnabled(false);
    ui.button_remove_approach_slot->setEnabled(true);
}

void MainWindow::on_button_add_final_position_slot_clicked(bool check)
{
    actual_slot_final_position = qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_slot_final = true;
    ui.button_add_final_position_slot->setEnabled(false);
    ui.button_remove_final_position_slot->setEnabled(true);
}

void MainWindow::on_button_add_location_clicked(bool check)
{
    std::string location_name = ui.edit_location_name->text().toStdString();
    if ( !location_name.empty() )
    {
        if ( !qnode.add_location(location_name) )
        {
            QMessageBox msgBox;
            msgBox.setText("There is another location with the same name.");
            msgBox.exec();
            ui.edit_location_name->clear();
        }
        else
        {
            ui.edit_location_name->clear();
            if ( ui.combo_action_type->currentIndex() == 0)
            {
                qnode.write_locations();
            }
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Empty name");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_recipe_clicked(bool check)
{
    std::string recipe_name = ui.edit_recipe_name->text().toStdString();
    if ( !recipe_name.empty() )
    {
        if ( ui.list_recipe->model()->rowCount() != 0 )
        {
            if ( !qnode.add_recipe( recipe_name ) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another recipe with the same name or the same actions order.");
                msgBox.exec();
                return;
            }
            else
            {
                ui.edit_recipe_name->clear();
                ui.list_recipe->model()->removeRows( 0, ui.list_recipe->model()->rowCount() );
                ui.recipe_box->addItem( QString::fromStdString( recipe_name ) );
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("The recipe is empty.");
            msgBox.exec();
            return;
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe_name is empty.");
        msgBox.exec();
        return;
    }
}

void MainWindow::on_button_add_approach_box_clicked(bool check)
{
    actual_box_approach = qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_box_approach = true;
    ui.button_add_approach_box->setEnabled(false);
    ui.button_remove_approach_box->setEnabled(true);
}

void MainWindow::on_button_add_final_box_clicked(bool check)
{
    actual_box_final = qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_box_final = true;
    ui.button_add_final_box->setEnabled(false);
    ui.button_remove_final_box->setEnabled(true);
}

void MainWindow::on_button_remove_go_to_clicked(bool check)
{
    QModelIndexList indexes =  ui.go_to_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.go_to_list->model();
        QAbstractItemModel* model_ = ui.list_go_to->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode.remove_go_to(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.go_to_list->selectionModel()->selectedIndexes();
        }
        ui.edit_action_description->clear();
        ui.list_info_action->model()->removeRows( 0, ui.list_info_action->model()->rowCount() );
        ui.go_to_list->clearSelection();

        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected position");
    msgBox.exec();
}

void MainWindow::on_button_remove_location_clicked(bool check)
{
    QModelIndexList indexes =  ui.locations_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;

        QAbstractItemModel* model  = ui.locations_list->model();
        QAbstractItemModel* model_ = ui.list_location_modify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode.remove_location(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.locations_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected position");
    msgBox.exec();
}

void MainWindow::on_button_remove_place_clicked(bool check)
{
    QModelIndexList indexes =  ui.place_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.place_list->model();;
        QAbstractItemModel* model_ = ui.list_place->model();;
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_place(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.place_list->selectionModel()->selectedIndexes();
        }
        ui.edit_action_description->clear();
        ui.list_info_action->model()->removeRows( 0, ui.list_info_action->model()->rowCount() );
        ui.place_list->clearSelection();

        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected position");
    msgBox.exec();
}

void MainWindow::on_button_remove_pick_clicked(bool check)
{
    QModelIndexList indexes =  ui.pick_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.pick_list->model();
        QAbstractItemModel* model_ = ui.list_pick->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_pick(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.pick_list->selectionModel()->selectedIndexes();
        }
        ui.edit_action_description->clear();
        ui.list_info_action->model()->removeRows( 0, ui.list_info_action->model()->rowCount() );
        ui.pick_list->clearSelection();

        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected Pick");
    msgBox.exec();
}

void MainWindow::on_button_remove_object_clicked(bool check)
{
    QModelIndexList indexes =  ui.object_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.object_list->model();
        QAbstractItemModel* model_ = ui.list_object_modify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_object(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.object_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected object");
    msgBox.exec();
}

void MainWindow::on_button_remove_box_clicked(bool check)
{
    QModelIndexList indexes =  ui.box_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.box_list->model();
        QAbstractItemModel* model_ = ui.list_box_modify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_box(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.box_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected box");
    msgBox.exec();
}

void MainWindow::on_button_remove_group_clicked(bool check)
{
    QModelIndexList indexes =  ui.group_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.group_list->model();
        QAbstractItemModel* model2 = ui.slot_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            std::vector<int> indexes2 = qnode.remove_group(index.row());
            for ( int i = indexes2.size()-1; i >= 0; i--)
            {
                model2->removeRow(indexes2[i]);
            }
            model->removeRow(index.row());
            indexes =  ui.group_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected group");
    msgBox.exec();
}

void MainWindow::on_button_remove_slot_clicked(bool check)
{
    QModelIndexList indexes =  ui.slot_list->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model  = ui.slot_list->model();
        QAbstractItemModel* model_ = ui.list_slot_modify->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_slot(index.row());
            model ->removeRow(index.row());
            model_->removeRow(index.row());
            indexes =  ui.slot_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected slot");
    msgBox.exec();
}

void MainWindow::on_button_remove_grasp_clicked(bool check)
{
    int index = ui.grasp_list->currentIndex();

    if ( ui.grasp_list->count() > 0 )
    {
        ui.grasp_list->removeItem(index);
        actual_object_grasp.erase(actual_object_grasp.begin()+index);
        actual_tool_grasp.erase(actual_tool_grasp.begin()+index);
        actual_object_approach.erase(actual_object_approach.begin()+index);
        actual_pre_gripper_position.erase(actual_pre_gripper_position.begin()+index);
        actual_tool_approach.erase(actual_tool_approach.begin()+index);
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected grasp");
    msgBox.exec();
}

void MainWindow::on_button_remove_approach_slot_clicked(bool check)
{
    init_slot_approach = false;
    ui.button_add_approach_slot->setEnabled(true);
    ui.button_remove_approach_slot->setEnabled(false);
}

void MainWindow::on_button_remove_final_position_slot_clicked(bool check)
{
    init_slot_final = false;
    ui.button_add_final_position_slot->setEnabled(true);
    ui.button_remove_final_position_slot->setEnabled(false);
}

void MainWindow::on_button_remove_approach_box_clicked(bool check)
{
    init_box_approach = false;
    ui.button_add_approach_box->setEnabled(true);
    ui.button_remove_approach_box->setEnabled(false);
}

void MainWindow::on_button_remove_final_box_clicked(bool check)
{
    init_box_final = false;
    ui.button_add_final_box->setEnabled(true);
    ui.button_remove_final_box->setEnabled(false);
}

void MainWindow::on_button_remove_recipe_clicked(bool check)
{
    int index = ui.recipe_box->currentIndex();
    qnode.remove_recipe( index );
    ui.recipe_box->removeItem( index );
}

void MainWindow::on_button_run_recipe_clicked(bool check)
{
    int risp = qnode.run_recipe();
    QMessageBox msgBox;

    switch (risp)
    {
    case 0 :
      msgBox.setText("The recipe was done without error");
      msgBox.exec();
      break;
    case 1 :
      msgBox.setText("The saving of components has not finished ");
      msgBox.exec();
      break;
    case 2 :
      msgBox.setText("Failed to call service run_recipe");
      msgBox.exec();
      break;
    }
}

void MainWindow::on_button_gripper_clicked(bool check)
{
//    double actual_gripper_position_target = (max_gripper_position/100) * ui.gripper_percentage->value();
//    actual_gripper_force    = ui.gripper_force_percentage->value();

//    std::string name_action = "pos_";
//    name_action.append( std::to_string(actual_gripper_position_target) );
//    name_action.append("_force_");
//    name_action.append( std::to_string(actual_gripper_force) );

  int perc = ui.gripper_percentage->value();

  if (perc < 50)
  {
    perc = 0;
  }
  else
  {
    perc = 100;
  }

    switch(perc) {

        case 0  :
           qnode.move_gripper("close");
//           actual_gripper_state = "close";
           break;
        case 10  :
           qnode.move_gripper("open_225");
//           actual_gripper_state = "open_225";
           break;
        case 20  :
           qnode.move_gripper("open_200");
//           actual_gripper_state = "open_200";
           break;
        case 30  :
           qnode.move_gripper("open_175");
//           actual_gripper_state = "open_175";
           break;
        case 40  :
           qnode.move_gripper("open_150");
//           actual_gripper_state = "open_150";
           break;
        case 50  :
           qnode.move_gripper("open_125");
//           actual_gripper_state = "open_125";
           break;
        case 60  :
           qnode.move_gripper("open_100");
//           actual_gripper_state = "open_100";
           break;
        case 70  :
           qnode.move_gripper("open_75");
//           actual_gripper_state = "open_75";
           break;
        case 80  :
           qnode.move_gripper("open_50");
//           actual_gripper_state = "open_50";
           break;
        case 90  :
           qnode.move_gripper("open_25");
//           actual_gripper_state = "open_25";
           break;
        case 100  :
           qnode.move_gripper("open");
//           actual_gripper_state = "open";
           break;
        default :
            qnode.move_gripper("open");
//            actual_gripper_state = "open";
    }
    return;
}

void MainWindow::on_button_gripper_2_clicked(bool check)
{
    on_button_gripper_clicked(false);
}

void MainWindow::on_button_save_components_clicked(bool check)
{
    if ( qnode.save_components() )
    {
        QMessageBox msgBox;
        msgBox.setText("The save are done.");
        msgBox.exec();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("The previous saving did not finish");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_object_clicked(bool check)
{ 
    QString object_name = ui.edit_object_name->text();
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
                            if( !qnode.add_object(obj_name,
                                                  actual_object_approach,
                                                  actual_object_grasp,
                                                  actual_object_leave,
                                                  actual_tool_grasp,
                                                  actual_pre_gripper_position,
                                                  actual_post_gripper_position,
                                                  actual_gripper_grasp_force) )
                            {
                                ROS_ERROR("Object just set");
                                QMessageBox msgBox;
                                msgBox.setText("There is another object with this name");
                                msgBox.exec();
                                ui.edit_object_name->clear();
                            }
                            else
                            {
                                num_grasp = 0;
                                ui.edit_object_name->clear();
                                ui.grasp_list->clear();
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
                                if ( ui.combo_action_type->currentIndex() == 1 )
                                {
                                    qnode.write_objects();
                                }
                            }
                        }
                        else
                        {
                            QMessageBox msgBox;
                            msgBox.setText("Leave and grasp have different tool");
                            msgBox.exec();
                        }
                    }
                    else
                    {
                        QMessageBox msgBox;
                        msgBox.setText("Apporach and grasp have different tool");
                        msgBox.exec();
                    }
                }
                else {
                    ROS_ERROR("Apporach and gripper state have different sizes: %zu, %zu", actual_object_approach.size(), actual_pre_gripper_position.size() );
                    QMessageBox msgBox;
                    msgBox.setText("Apporach and gripper have different sizes");
                    msgBox.exec();
                }
            }
            else {
                ROS_ERROR("Apporach and grasp have different sizes: %zu, %zu", actual_object_grasp.size(), actual_object_approach.size() );
                QMessageBox msgBox;
                msgBox.setText("Apporach and grasp have different sizes");
                msgBox.exec();
            }
        }
        else {
            ROS_ERROR("Empty grasp");
            QMessageBox msgBox;
            msgBox.setText("Empty grasp");
            msgBox.exec();
        }

    }
    else {
        ROS_ERROR("Empty name");
        QMessageBox msgBox;
        msgBox.setText("Empty name");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_slot_clicked(bool check)
{
    bool ok;

    std::string slot_name = ui.edit_slot_name->text().toStdString();
    std::string max_obj   = ui.edit_max_object->text().toStdString();
    int num_max_obj       = ui.edit_max_object->text().toInt(&ok);
    std::string group_name= ui.edit_group_name->text().toStdString();

    if ( !slot_name.empty() )
    {
        if (init_slot_approach)
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

                            if( !qnode.add_slot(slot_name, actual_slot_approach, actual_slot_final_position, actual_slot_leave, group_name, num_max_obj ) )
                            {
                                QMessageBox msgBox;
                                msgBox.setText("There is another slot with this name");
                                msgBox.exec();
                                ui.edit_slot_name->clear();
                            }
                            else
                            {
                                ui.edit_slot_name->clear();
                                ui.edit_max_object->clear();
                                ui.edit_group_name->clear();
                                init_slot_approach = false;
                                init_slot_final = false;
                                ui.button_add_approach_slot->setEnabled(true);
                                ui.button_remove_approach_slot->setEnabled(false);
                                ui.button_add_final_position_slot->setEnabled(true);
                                ui.button_remove_final_position_slot->setEnabled(false);
                                if ( ui.combo_action_type->currentIndex() == 2 )
                                {
                                    qnode.write_groups();
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR("Max number isn't a number");
                            QMessageBox msgBox;
                            msgBox.setText("Max number isn't a number");
                            msgBox.exec();
                            ui.edit_max_object->clear();
                        }
                    }
                    else
                    {
                        ROS_ERROR("Empty final position");
                        QMessageBox msgBox;
                        msgBox.setText("Empty final position");
                        msgBox.exec();
                    }
                }
                else
                {
                    ROS_ERROR("Empty max number");
                    QMessageBox msgBox;
                    msgBox.setText("Empty max number");
                    msgBox.exec();
                }
            }
            else {
                ROS_ERROR("Empty group");
                QMessageBox msgBox;
                msgBox.setText("Empty group");
                msgBox.exec();
            }
        }
        else {
            ROS_ERROR("Empty approach");
            QMessageBox msgBox;
            msgBox.setText("Empty approach");
            msgBox.exec();
        }
    }
    else {
        ROS_ERROR("Empty name");
        QMessageBox msgBox;
        msgBox.setText("Empty name");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_box_clicked(bool check)
{
    std::string box_name = ui.edit_box_name->text().toStdString();
    if ( init_box_approach )
    {
        if ( init_box_final )
        {
            if ( !box_name.empty() )
            {
                if ( !init_box_leave )
                {
                    actual_box_leave = actual_box_approach;
                }

                if( !qnode.add_box( box_name, actual_box_approach, actual_box_final, actual_box_leave ) )
                {
                    ROS_ERROR("box just set");
                    QMessageBox msgBox;
                    msgBox.setText("There is another box with this name");
                    msgBox.exec();
                    ui.edit_box_name->clear();
                }
                else
                {
                    ui.edit_box_name->clear();
                    init_box_approach = false;
                    init_box_final = false;
                    ui.button_add_approach_box->setEnabled(true);
                    ui.button_remove_approach_box->setEnabled(false);
                    ui.button_add_final_box->setEnabled(true);
                    ui.button_remove_final_box->setEnabled(false);
                }
            }
            else
            {
                ROS_ERROR("Empty name");
                QMessageBox msgBox;
                msgBox.setText("Empty name");
                msgBox.exec();
            }
        }
        else
        {
            ROS_ERROR("Empty final position");
            QMessageBox msgBox;
            msgBox.setText("Empty final position");
            msgBox.exec();
        }
    }
    else
    {
        ROS_ERROR("Empty approach position");
        QMessageBox msgBox;
        msgBox.setText("Empty approach position");
        msgBox.exec();
    }
}

void MainWindow::on_button_add_location_changes_clicked(bool check)
{
    go_to_location loc;

    QModelIndex index = ui.list_location_modify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6;
    loc.location_.pos.origin_x    = ui.edit_location_position_x   ->text().toDouble(&ok0);
    loc.location_.pos.origin_y    = ui.edit_location_position_y   ->text().toDouble(&ok1);
    loc.location_.pos.origin_z    = ui.edit_location_position_z   ->text().toDouble(&ok2);
    loc.location_.quat.rotation_x = ui.edit_location_quaternion_x ->text().toDouble(&ok3);
    loc.location_.quat.rotation_y = ui.edit_location_quaternion_y ->text().toDouble(&ok4);
    loc.location_.quat.rotation_z = ui.edit_location_quaternion_z ->text().toDouble(&ok5);
    loc.location_.quat.rotation_w = ui.edit_location_quaternion_w ->text().toDouble(&ok6);
    loc.frame                     = ui.edit_location_frame        ->text().toStdString();
    loc.name                      = ui.list_location_modify       ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6)
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't double");
        msgBox.exec();
    }
    else
    {
        qnode.add_location_changes( index.row(), loc);
    }
}

void MainWindow::on_button_add_slot_changes_clicked(bool check)
{
    manipulation_slot slt;

    QModelIndex index = ui.list_slot_modify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9, ok10;
    slt.location_.pos.origin_x    = ui.edit_slot_position_x   ->text().toDouble(&ok0);
    slt.location_.pos.origin_y    = ui.edit_slot_position_y   ->text().toDouble(&ok1);
    slt.location_.pos.origin_z    = ui.edit_slot_position_z   ->text().toDouble(&ok2);
    slt.location_.quat.rotation_x = ui.edit_slot_quaternion_x ->text().toDouble(&ok3);
    slt.location_.quat.rotation_y = ui.edit_slot_quaternion_y ->text().toDouble(&ok4);
    slt.location_.quat.rotation_z = ui.edit_slot_quaternion_z ->text().toDouble(&ok5);
    slt.location_.quat.rotation_w = ui.edit_slot_quaternion_w ->text().toDouble(&ok6);
    slt.approach.origin_x         = ui.edit_slot_approach_x   ->text().toDouble(&ok7);
    slt.approach.origin_y         = ui.edit_slot_approach_y   ->text().toDouble(&ok8);
    slt.approach.origin_z         = ui.edit_slot_approach_z   ->text().toDouble(&ok9);
    slt.max_objects               = ui.edit_slot_max_objects  ->text().toInt(&ok10);
    slt.frame                     = ui.edit_slot_frame        ->text().toStdString();
    slt.group                     = ui.edit_slot_group        ->text().toStdString();
    slt.name                      = ui.list_slot_modify       ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 || !ok10 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't numers");
        msgBox.exec();
    }
    else
    {
        qnode.add_slot_changes( index.row(), slt);
    }
}

void MainWindow::on_button_add_box_changes_clicked(bool check)
{
    box bx;

    QModelIndex index = ui.list_box_modify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9;
    bx.location_.pos.origin_x    = ui.edit_box_position_x   ->text().toDouble(&ok0);
    bx.location_.pos.origin_y    = ui.edit_box_position_y   ->text().toDouble(&ok1);
    bx.location_.pos.origin_z    = ui.edit_box_position_z   ->text().toDouble(&ok2);
    bx.location_.quat.rotation_x = ui.edit_box_quaternion_x ->text().toDouble(&ok3);
    bx.location_.quat.rotation_y = ui.edit_box_quaternion_y ->text().toDouble(&ok4);
    bx.location_.quat.rotation_z = ui.edit_box_quaternion_z ->text().toDouble(&ok5);
    bx.location_.quat.rotation_w = ui.edit_box_quaternion_w ->text().toDouble(&ok6);
    bx.approach.origin_x         = ui.edit_box_approach_x   ->text().toDouble(&ok7);
    bx.approach.origin_y         = ui.edit_box_approach_y   ->text().toDouble(&ok8);
    bx.approach.origin_z         = ui.edit_box_approach_z   ->text().toDouble(&ok9);
    bx.frame                     = ui.edit_box_frame        ->text().toStdString();
    bx.name                      = ui.list_box_modify       ->model()->data(index).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't numers");
        msgBox.exec();
    }
    else
    {
        qnode.add_box_changes( index.row(), bx);
    }
}

void MainWindow::on_button_add_object_changes_clicked(bool check)
{
    QModelIndex index = ui.list_object_modify->currentIndex();
    int index2 = ui.combo_grasp_number->currentIndex();

    object_type obj = qnode.return_object_info( index.row() );

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9;

    obj.approach[index2].origin_x     = ui.edit_object_approach_x   ->text().toDouble(&ok7);
    obj.approach[index2].origin_y     = ui.edit_object_approach_y   ->text().toDouble(&ok8);
    obj.approach[index2].origin_z     = ui.edit_object_approach_z   ->text().toDouble(&ok9);
    obj.grasp[index2].pos.origin_x    = ui.edit_object_position_x   ->text().toDouble(&ok0);
    obj.grasp[index2].pos.origin_y    = ui.edit_object_position_y   ->text().toDouble(&ok1);
    obj.grasp[index2].pos.origin_z    = ui.edit_object_position_z   ->text().toDouble(&ok2);
    obj.grasp[index2].quat.rotation_x = ui.edit_object_quaternion_x ->text().toDouble(&ok3);
    obj.grasp[index2].quat.rotation_y = ui.edit_object_quaternion_y ->text().toDouble(&ok4);
    obj.grasp[index2].quat.rotation_z = ui.edit_object_quaternion_z ->text().toDouble(&ok5);
    obj.grasp[index2].quat.rotation_w = ui.edit_object_quaternion_w ->text().toDouble(&ok6);
    obj.tool[index2]                  = ui.edit_object_tool         ->text().toStdString();
    obj.type                          = ui.list_object_modify       ->model()->data( index ).toString().toStdString();

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't double");
        msgBox.exec();
    }
    else
    {
        qnode.add_object_changes( index.row(), obj);
    }
}

void MainWindow::on_button_save_actions_clicked(bool check)
{
    if ( qnode.save_actions() )
    {
        QMessageBox msgBox;
        msgBox.setText("The save are done.");
        msgBox.exec();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("There is some problem with the save");
        msgBox.exec();
    }
}

void MainWindow::on_button_save_all_changes0_clicked(bool check)
{
    qnode.save_components();
}

void MainWindow::on_button_save_all_changes1_clicked(bool check)
{
    qnode.save_components();
}

void MainWindow::on_button_save_all_changes2_clicked(bool check)
{
    qnode.save_components();
}

void MainWindow::on_button_save_all_changes3_clicked(bool check)
{
    qnode.save_components();
}

void MainWindow::on_button_load_TF_clicked(bool chack)
{
    qnode.load_TF();

    QString tf;
    ui.TF_list->clear();
    std::string tf_name;
    for ( int i = 0; i < qnode.TFs.size(); i++ )
    {
        std::size_t found = qnode.TFs[i].find( tf_name_space.c_str() );
        if ( found != std::string::npos )
        {
            tf_name = qnode.TFs[i];
            found = tf_name.find("/",2);
            tf_name.erase( 0, found+1 );
            tf = QString::fromStdString(qnode.TFs[i]);
            ui.TF_list->addItem(tf);
        }
    }
}

void MainWindow::on_button_load_clicked(bool chack)
{
    qnode.write_param(1);
}

void MainWindow::on_button_copy_grasp_clicked(bool chack)
{
    int index  = ui.list_object_modify->currentIndex().row();
    int index2 = ui.combo_grasp_number->currentIndex();

    qnode.add_object_copy_grasp( index, index2);
    QString qstr = QString::fromStdString( std::to_string( ui.combo_grasp_number->count() ) );
    ui.combo_grasp_number->addItem( qstr );
    actual_object_to_modify = qnode.return_object_info(index);
}

void MainWindow::on_button_reset_location_info_clicked(bool chack)
{
    if ( !ui.list_location_modify->selectionModel()->selectedIndexes().empty() )
    {
        reset_location( ui.list_location_modify->currentIndex().row() );
    }
}

void MainWindow::reset_location(int index)
{
    ui.edit_location_frame        ->clear();
    ui.edit_location_position_x   ->clear();
    ui.edit_location_position_y   ->clear();
    ui.edit_location_position_z   ->clear();
    ui.edit_location_quaternion_w ->clear();
    ui.edit_location_quaternion_x ->clear();
    ui.edit_location_quaternion_y ->clear();
    ui.edit_location_quaternion_z ->clear();

    go_to_location loc = qnode.return_location_info(index);
    QString frame_ = QString::fromStdString( loc.frame);
    QString pos_x  = QString::fromStdString( std::to_string(loc.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(loc.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(loc.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(loc.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(loc.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(loc.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(loc.location_.quat.rotation_w) );
    QString name_  = QString::fromStdString( loc.name );

    ui.edit_location_frame        ->insert(frame_);
    ui.edit_location_position_x   ->insert(pos_x);
    ui.edit_location_position_y   ->insert(pos_y);
    ui.edit_location_position_z   ->insert(pos_z);
    ui.edit_location_quaternion_w ->insert(quat_w);
    ui.edit_location_quaternion_x ->insert(quat_x);
    ui.edit_location_quaternion_y ->insert(quat_y);
    ui.edit_location_quaternion_z ->insert(quat_z);
}

void MainWindow::on_button_reset_slot_info_clicked(bool chack)
{
    if ( !ui.list_slot_modify->selectionModel()->selectedIndexes().empty() )
    {
        reset_slot( ui.list_slot_modify->currentIndex().row() );
    }
}

void MainWindow::reset_slot(int index)
{
    ui.edit_slot_frame        ->clear();
    ui.edit_slot_position_x   ->clear();
    ui.edit_slot_position_y   ->clear();
    ui.edit_slot_position_z   ->clear();
    ui.edit_slot_quaternion_w ->clear();
    ui.edit_slot_quaternion_x ->clear();
    ui.edit_slot_quaternion_y ->clear();
    ui.edit_slot_quaternion_z ->clear();
    ui.edit_slot_approach_x   ->clear();
    ui.edit_slot_approach_y   ->clear();
    ui.edit_slot_approach_z   ->clear();
    ui.edit_slot_group        ->clear();
    ui.edit_slot_max_objects  ->clear();
    ui.edit_slot_leave_x      ->clear();
    ui.edit_slot_leave_y      ->clear();
    ui.edit_slot_leave_z      ->clear();

    manipulation_slot sl = qnode.return_slot_info(index);
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
    QString leav_x = QString::fromStdString( std::to_string(sl.leave.origin_z) );
    QString leav_y = QString::fromStdString( std::to_string(sl.leave.origin_z) );
    QString leav_z = QString::fromStdString( std::to_string(sl.leave.origin_z) );

    ui.edit_slot_frame        ->insert(frame_);
    ui.edit_slot_group        ->insert(group_);
    ui.edit_slot_position_x   ->insert(pos_x);
    ui.edit_slot_position_y   ->insert(pos_y);
    ui.edit_slot_position_z   ->insert(pos_z);
    ui.edit_slot_quaternion_w ->insert(quat_w);
    ui.edit_slot_quaternion_x ->insert(quat_x);
    ui.edit_slot_quaternion_y ->insert(quat_y);
    ui.edit_slot_quaternion_z ->insert(quat_z);
    ui.edit_slot_approach_x   ->insert(appr_x);
    ui.edit_slot_approach_y   ->insert(appr_y);
    ui.edit_slot_approach_z   ->insert(appr_z);
    ui.edit_slot_max_objects  ->insert(max_ob);
    ui.edit_slot_leave_x      ->insert(leav_x);
    ui.edit_slot_leave_y      ->insert(leav_y);
    ui.edit_slot_leave_z      ->insert(leav_z);

}

void MainWindow::on_button_reset_box_info_clicked(bool chack)
{
    if ( !ui.list_box_modify->selectionModel()->selectedIndexes().empty() )
    {
        reset_box( ui.list_box_modify->currentIndex().row() );
    }
}

void MainWindow::on_button_up_pressed      ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, 0, vel, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_down_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, 0, -vel, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_left_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, -vel, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_right_pressed   ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { 0, vel, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_front_pressed   ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { -vel, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_back_pressed    ()
{
    std::vector<float> twist_move;
    float vel = max_vel*perc_vel/100;

    twist_move = { vel, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_x_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, rot, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_y_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, rot, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_z_pressed ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, 0, rot };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_x_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, -rot, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_y_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, -rot, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_z_pressed  ()
{
    std::vector<float> twist_move;
    float rot = max_rot*perc_vel/100;

    twist_move = { 0, 0, 0, 0, 0, -rot };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_up_released      ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_down_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_left_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_right_released   ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_front_released   ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_back_released    ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_x_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_y_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_clock_z_released ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_x_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_y_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_anti_z_released  ()
{
    std::vector<float> twist_move;
    twist_move = { 0, 0, 0, 0, 0, 0 };
    qnode.cartMove ( twist_move );
}

void MainWindow::on_button_load_objects_clicked(bool check)
{
    qnode.load_objects_in_manipulation();
}

void MainWindow::on_button_run_selected_action_clicked(bool check)
{
    QModelIndexList indexes =  ui.list_recipe->selectionModel()->selectedIndexes();
    if ( !indexes.empty() )
    {
        int index = indexes.at(0).row();
        int risp = qnode.run_selected_action(index);
        QMessageBox msgBox;

        switch (risp)
        {
        case 0 :
          msgBox.setText("The recipe was done without error");
          msgBox.exec();
          break;
        case 1 :
          msgBox.setText("The saving of components has not finished ");
          msgBox.exec();
          break;
        case 2 :
          msgBox.setText("Failed to call service run_recipe");
          msgBox.exec();
          break;
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No selected action");
        msgBox.exec();
        ui.edit_action_name->clear();
    }
}

void MainWindow::on_button_copy_location_clicked(bool chack)
{
    if ( !ui.list_location_modify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.list_location_modify->currentIndex().row();
        go_to_location loc = qnode.return_location_info( index );
        loc.name = ui.edit_new_location->text().toStdString();

        if ( !loc.name.empty() )
        {
            if ( !qnode.add_location_copy( loc ) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another location whit the same name.");
                msgBox.exec();
                ui.edit_new_location->clear();
            }
            else
            {
                ui.edit_new_location->clear();
                if ( ui.combo_action_type->currentIndex() == 0)
                {
                    qnode.write_locations();
                }
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Empty name.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No selected location.");
        msgBox.exec();
    }
}

void MainWindow::on_button_copy_object_clicked(bool chack)
{
    if ( !ui.list_object_modify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.list_object_modify->currentIndex().row();
        object_type obj = qnode.return_object_info( index );
        obj.type = ui.edit_new_object->text().toStdString();

        if ( !obj.type.empty() )
        {
            if ( !qnode.add_object_copy( obj ) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another object whit the same name.");
                msgBox.exec();
                ui.edit_new_object->clear();
            }
            else
            {
                ui.edit_new_object->clear();
                if ( ui.combo_action_type->currentIndex() == 1 )
                {
                    qnode.write_objects();
                }
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Empty name.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No selected object.");
        msgBox.exec();
    }
}

void MainWindow::on_button_copy_slot_clicked(bool chack)
{
    if ( !ui.list_slot_modify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.list_slot_modify->currentIndex().row();
        manipulation_slot slt = qnode.return_slot_info( index );
        slt.name = ui.edit_new_slot->text().toStdString();

        if ( !slt.name.empty() )
        {
            if ( !qnode.add_slot_copy( slt ) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another slot whit the same name.");
                msgBox.exec();
                ui.edit_new_slot->clear();
            }
            else
            {
                ui.edit_new_slot->clear();
                if ( ui.combo_action_type->currentIndex() == 2 )
                {
                    qnode.write_groups();
                }
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Empty name.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No selected slot.");
        msgBox.exec();
    }
}

void MainWindow::on_button_copy_box_clicked(bool chack)
{
    if ( !ui.list_box_modify->selectionModel()->selectedIndexes().empty() )
    {
        int index = ui.list_box_modify->currentIndex().row();
        box bx = qnode.return_box_info( index );
        bx.name = ui.edit_new_box->text().toStdString();

        if ( !bx.name.empty() )
        {
            if ( !qnode.add_box_copy( bx ) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another box whit the same name.");
                msgBox.exec();
                ui.edit_new_box->clear();
            }
            else
            {
                ui.edit_new_box->clear();
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Empty name.");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No selected box.");
        msgBox.exec();
    }
}

void MainWindow::reset_box(int index)
{
    ui.edit_box_frame        ->clear();
    ui.edit_box_position_x   ->clear();
    ui.edit_box_position_y   ->clear();
    ui.edit_box_position_z   ->clear();
    ui.edit_box_quaternion_w ->clear();
    ui.edit_box_quaternion_x ->clear();
    ui.edit_box_quaternion_y ->clear();
    ui.edit_box_quaternion_z ->clear();
    ui.edit_box_approach_x   ->clear();
    ui.edit_box_approach_y   ->clear();
    ui.edit_box_approach_z   ->clear();
    ui.edit_box_leave_x      ->clear();
    ui.edit_box_leave_y      ->clear();
    ui.edit_box_leave_z      ->clear();

    box bx = qnode.return_box_info(index);
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
    QString leav_x = QString::fromStdString( std::to_string(bx.leave.origin_z) );
    QString leav_y = QString::fromStdString( std::to_string(bx.leave.origin_z) );
    QString leav_z = QString::fromStdString( std::to_string(bx.leave.origin_z) );
    QString name_  = QString::fromStdString( bx.name );

    ui.edit_box_frame         ->insert(frame_);
    ui.edit_box_position_x    ->insert(pos_x);
    ui.edit_box_position_y    ->insert(pos_y);
    ui.edit_box_position_z    ->insert(pos_z);
    ui.edit_box_quaternion_w  ->insert(quat_w);
    ui.edit_box_quaternion_x  ->insert(quat_x);
    ui.edit_box_quaternion_y  ->insert(quat_y);
    ui.edit_box_quaternion_z  ->insert(quat_z);
    ui.edit_box_approach_x    ->insert(appr_x);
    ui.edit_box_approach_y    ->insert(appr_y);
    ui.edit_box_approach_z    ->insert(appr_z);
    ui.edit_box_leave_x       ->insert(leav_x);
    ui.edit_box_leave_y       ->insert(leav_y);
    ui.edit_box_leave_z       ->insert(leav_z);

}

void MainWindow::on_button_reset_object_info_clicked(bool chack)
{
    if ( !ui.list_object_modify->selectionModel()->selectedIndexes().empty() )
    {
        reset_object( ui.list_object_modify->currentIndex().row() );
    }
}

void MainWindow::reset_object(int index)
{
    init_objects = false;
    ui.combo_grasp_number->clear();

    ui.edit_object_position_x   ->clear();
    ui.edit_object_position_y   ->clear();
    ui.edit_object_position_z   ->clear();
    ui.edit_object_quaternion_w ->clear();
    ui.edit_object_quaternion_x ->clear();
    ui.edit_object_quaternion_y ->clear();
    ui.edit_object_quaternion_z ->clear();
    ui.edit_object_approach_x   ->clear();
    ui.edit_object_approach_y   ->clear();
    ui.edit_object_approach_z   ->clear();
    ui.edit_object_tool         ->clear();
    ui.edit_gripper_state       ->clear();
    ui.edit_pre_gripper_state   ->clear();
    ui.edit_post_gripper_state  ->clear();
    ui.edit_object_leave_x      ->clear();
    ui.edit_object_leave_y      ->clear();
    ui.edit_object_leave_z      ->clear();

    actual_object_to_modify = qnode.return_object_info(index);

    for ( int i = 0; i < actual_object_to_modify.grasp.size(); i++ )
    {
        ui.combo_grasp_number->addItem( QString::fromStdString( std::to_string(i) ) );
    }
    init_objects = true;

    int i = ui.combo_grasp_number->currentIndex();
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
    QString leav_x = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_z) );
    QString leav_y = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_z) );
    QString leav_z = QString::fromStdString( std::to_string(actual_object_to_modify.leave[i].origin_z) );
    QString tool   = QString::fromStdString( actual_object_to_modify.tool[i] );
    QString type_  = QString::fromStdString( actual_object_to_modify.type );
    QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify.gripper_force[i] ) );
    QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify.pre_gripper_position[i] ) );
    QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify.post_gripper_position[i] ) );

    ui.edit_object_position_x   ->insert(pos_x);
    ui.edit_object_position_y   ->insert(pos_y);
    ui.edit_object_position_z   ->insert(pos_z);
    ui.edit_object_quaternion_w ->insert(quat_w);
    ui.edit_object_quaternion_x ->insert(quat_x);
    ui.edit_object_quaternion_y ->insert(quat_y);
    ui.edit_object_quaternion_z ->insert(quat_z);
    ui.edit_object_approach_x   ->insert(appr_x);
    ui.edit_object_approach_y   ->insert(appr_y);
    ui.edit_object_approach_z   ->insert(appr_z);
    ui.edit_object_leave_x      ->insert(leav_x);
    ui.edit_object_leave_y      ->insert(leav_y);
    ui.edit_object_leave_z      ->insert(leav_z);
    ui.edit_object_tool         ->insert(tool);
    ui.edit_gripper_state       ->insert(gripper_force);
    ui.edit_pre_gripper_state   ->insert(pre_position);
    ui.edit_post_gripper_state  ->insert(post_position);
}

void MainWindow::on_check_robot_TF_stateChanged(int state)
{
    if ( state != 0 )
    {
        ui.button_save_components         ->setEnabled(true);
        ui.lateral_tab                    ->setEnabled(true);

        ui.check_robot_TF ->setEnabled(false);
        ui.world_TF_list  ->setEnabled(false);

        qnode.base_frame = ui.world_TF_list->currentText().toStdString();
    }
}

void MainWindow::on_robot_list_currentIndexChanged(int index)
{
    qnode.set_target_frame( index );
}

void MainWindow::on_combo_grasp_number_currentIndexChanged(int index)
{
    ui.edit_object_position_x   ->clear();
    ui.edit_object_position_y   ->clear();
    ui.edit_object_position_z   ->clear();
    ui.edit_object_quaternion_w ->clear();
    ui.edit_object_quaternion_x ->clear();
    ui.edit_object_quaternion_y ->clear();
    ui.edit_object_quaternion_z ->clear();
    ui.edit_object_approach_x   ->clear();
    ui.edit_object_approach_y   ->clear();
    ui.edit_object_approach_z   ->clear();
    ui.edit_object_tool         ->clear();
    ui.edit_gripper_state       ->clear();
    ui.edit_pre_gripper_state   ->clear();
    ui.edit_post_gripper_state  ->clear();
    ui.edit_object_leave_x      ->clear();
    ui.edit_object_leave_y      ->clear();
    ui.edit_object_leave_z      ->clear();

    if ( init_objects)
    {
        QString pos_x  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_x) );
        QString pos_y  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_y) );
        QString pos_z  = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].pos.origin_z) );
        QString quat_x = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_x) );
        QString quat_y = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_y) );
        QString quat_z = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_z) );
        QString quat_w = QString::fromStdString( std::to_string(actual_object_to_modify.grasp[index].quat.rotation_w) );
        QString appr_x = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_x) );
        QString appr_y = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_y) );
        QString appr_z = QString::fromStdString( std::to_string(actual_object_to_modify.approach[index].origin_z) );
        QString tool   = QString::fromStdString( actual_object_to_modify.tool[index] );
        QString gripper_force  = QString::fromStdString( std::to_string( actual_object_to_modify.gripper_force[index] ) );
        QString pre_position   = QString::fromStdString( std::to_string( actual_object_to_modify.pre_gripper_position[index] ) );
        QString post_position  = QString::fromStdString( std::to_string( actual_object_to_modify.post_gripper_position[index] ) );
        QString leav_x = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_z) );
        QString leav_y = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_z) );
        QString leav_z = QString::fromStdString( std::to_string(actual_object_to_modify.leave[index].origin_z) );

        ui.edit_object_position_x   ->insert(pos_x);
        ui.edit_object_position_y   ->insert(pos_y);
        ui.edit_object_position_z   ->insert(pos_z);
        ui.edit_object_quaternion_w ->insert(quat_w);
        ui.edit_object_quaternion_x ->insert(quat_x);
        ui.edit_object_quaternion_y ->insert(quat_y);
        ui.edit_object_quaternion_z ->insert(quat_z);
        ui.edit_object_approach_x   ->insert(appr_x);
        ui.edit_object_approach_y   ->insert(appr_y);
        ui.edit_object_approach_z   ->insert(appr_z);
        ui.edit_object_tool         ->insert(tool);
        ui.edit_gripper_state       ->insert(gripper_force);
        ui.edit_object_leave_x      ->insert(leav_x);
        ui.edit_object_leave_y      ->insert(leav_y);
        ui.edit_object_leave_z      ->insert(leav_z);
        ui.edit_pre_gripper_state   ->insert(pre_position);
        ui.edit_post_gripper_state  ->insert(post_position);
    }
}

void MainWindow::on_combo_action_type_currentIndexChanged(int index)
{
    if ( index == 0 )
    {
        qnode.write_locations();
    }
    else if ( index == 1)
    {
        qnode.write_objects();
    }
    else if ( index == 2 )
    {
        qnode.write_groups();
    }
}

void MainWindow::on_combo_configuration_currentIndexChanged(int index)
{
    if ( index == 0 )
    {
        qnode.active_configuration("watch");
    }
    else if ( index == 1 )
    {
        qnode.active_configuration("manual_guidance");
    }
    else if ( index == 2 )
    {
        qnode.active_configuration("lin_xyz_manual_guidance");
    }
    else if ( index == 3 )
    {
        qnode.active_configuration("rot_xyz_manual_guidance");
    }
    else if ( index == 4 )
    {
        qnode.active_configuration("cart_teleop");
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("There is a problem with configuration");
        msgBox.exec();
    }
    ui.combo_configuration_->setCurrentIndex( index );
}

void MainWindow::on_combo_configuration__currentIndexChanged(int index)
{
    ui.combo_configuration->setCurrentIndex( index );
}

void MainWindow::on_combo_ref_frame_currentIndexChanged(int index)
{
    qnode.frame_id.clear();
    qnode.frame_id.append("/");
    qnode.frame_id.append( ui.combo_ref_frame->currentText().toStdString() );
}

void MainWindow::on_TF_list_currentIndexChanged(int index)
{
  std::string name = ui.TF_list->currentText().toStdString();
  std::size_t found  = name.find( "/" );
  if ( found != std::string::npos)
  {
    name.erase( found, name.size() );
  }
  ui.edit_object_name->setText(QString::fromStdString(name));
}

void MainWindow::on_list_location_modify_pressed(const QModelIndex &index)
{
    reset_location( index.row() );
}

void MainWindow::on_list_box_modify_pressed(const QModelIndex &index)
{
    reset_box( index.row() );
}
void MainWindow::on_list_slot_modify_pressed(const QModelIndex &index)
{
    reset_slot( index.row() );
}

void MainWindow::on_list_object_modify_pressed(const QModelIndex &index)
{
    reset_object( index.row() );
}

void MainWindow::on_gripper_percentage_valueChanged (int value)
{
//  if ( value < 5)
//  {
//    value = 0;
//  }
//  else if ( value < 15 )
//  {
//    value = 10;
//  }else if ( value < 25 )
//  {
//    value = 20;
//  }else if ( value < 35 )
//  {
//    value = 30;
//  }
//  else if ( value < 45 )
//  {
//    value = 40;
//  }
//  else if ( value < 55 )
//  {
//    value = 50;
//  }
//  else if ( value < 65 )
//  {
//    value = 60;
//  }
//  else if ( value < 75 )
//  {
//    value = 70;
//  }
//  else if ( value < 85 )
//  {
//    value = 80;
//  }
//  else if ( value < 95 )
//  {
//    value = 90;
//  }
//  else
//  {
//    value = 100;
//  }

  ui.gripper_percentage->setValue(value);
  ui.gripper_percentage_2->setValue(value);
  std::string str = std::to_string(value);
  QString qstr = QString::fromStdString(str);
  ui.open_gripper_label->setText(qstr);
  ui.open_gripper_label_2->setText(qstr);
}

void MainWindow::on_gripper_force_percentage_valueChanged (int value)
{
//    if ( value < 5)
//    {
//      value = 0;
//    }
//    else if ( value < 15 )
//    {
//      value = 10;
//    }else if ( value < 25 )
//    {
//      value = 20;
//    }else if ( value < 35 )
//    {
//      value = 30;
//    }
//    else if ( value < 45 )
//    {
//      value = 40;
//    }
//    else if ( value < 55 )
//    {
//      value = 50;
//    }
//    else if ( value < 65 )
//    {
//      value = 60;
//    }
//    else if ( value < 75 )
//    {
//      value = 70;
//    }
//    else if ( value < 85 )
//    {
//      value = 80;
//    }
//    else if ( value < 95 )
//    {
//      value = 90;
//    }
//    else
//    {
//      value = 100;
//    }

    ui.gripper_force_percentage->setValue(value);
    ui.gripper_force_percentage_2->setValue(value);
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui.gripper_force_label->setText(qstr);
    ui.gripper_force_label_2->setText(qstr);
}

void MainWindow::on_gripper_percentage_2_valueChanged (int value)
{
    ui.gripper_percentage->setValue(value);
}

void MainWindow::on_gripper_force_percentage_2_valueChanged (int value)
{
    ui.gripper_force_percentage->setValue(value);
}

void MainWindow::on_velocity_slider_valueChanged(int value)
{
    std::string str = std::to_string(value);
    QString qstr = QString::fromStdString(str);
    ui.velocity_label->setText(qstr);
    perc_vel = value;
}

void MainWindow::on_lateral_layout_currentChanged(int index)
{
    on_button_load_TF_clicked(false);
}

void MainWindow::on_button_remove_element_clicked(bool check )
{
    QModelIndexList indexes =  ui.list_recipe->selectionModel()->selectedIndexes();
    if (!indexes.empty())
    {
        QModelIndex index;
        QAbstractItemModel* model = ui.list_recipe->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            model->removeRow(index.row());
            indexes =  ui.list_recipe->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected position");
    msgBox.exec();
}

void MainWindow::on_button_load_recipe_clicked(bool check)
{
    std::vector<std::string> recipes_names = qnode.load_recipes_param();
    for ( int i = 0; i < recipes_names.size(); i++)
    {
        bool presence = false;
        for ( int j = 0; j < ui.recipe_box->count(); j++)
        {
            if ( !recipes_names[i].compare( ui.recipe_box->itemText(j).toStdString() ))
            {
                presence = true;
            }
        }
        if ( !presence )
        {
            ui.recipe_box->addItem( QString::fromStdString( recipes_names[i] ));
        }
    }
}

void MainWindow::on_button_write_recipe_clicked(bool check)
{
    int ind = ui.recipe_box->currentIndex();
    if ( ui.recipe_box->count() != 0 )
    {
        qnode.write_recipe( ind );
    }
}

void MainWindow::on_button_load_actions_clicked  ( bool check )
{
    qnode.write_param(2);
}

void MainWindow::on_button_save_recipe_clicked ( bool check )
{

    if ( qnode.save_recipe() )
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe is saved.");
        msgBox.exec();
        ui.edit_recipe_name->clear();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("There is some problem with the save");
        msgBox.exec();
        return;
    }
}

void MainWindow::on_list_place_pressed(const QModelIndex &index)
{
    qnode.add_slot_groups( index.row() );
    place pl = qnode.return_place_info( index.row() );
    QString qstr = QString::fromStdString( pl.description );
    ui.edit_action_description_2->setText( qstr );
    ui.list_go_to->clearSelection();
    ui.list_pick->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
    else if ( pl.agents[0] == "robot" )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( pl.agents[0] == "human" )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(false);
    }
    else
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
}

void MainWindow::on_list_pick_pressed(const QModelIndex &index)
{
    qnode.add_object_type( index.row() );
    pick pk = qnode.return_pick_info( index.row() );
    QString qstr = QString::fromStdString( pk.description );
    ui.edit_action_description_2->setText( qstr );
    ui.list_place->clearSelection();
    ui.list_go_to->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
    else if ( pk.agents[0] == "robot" )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( pk.agents[0] == "human" )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(false);
    }
    else
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
}

void MainWindow::on_list_go_to_pressed(const QModelIndex &index)
{
    qnode.add_location_info( index.row() );
    go_to_action gt = qnode.return_go_to_info( index.row() );
    QString qstr = QString::fromStdString( gt.description );
    ui.edit_action_description_2->setText( qstr );
    ui.list_place->clearSelection();
    ui.list_pick->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
    else if ( gt.agents[0] == "robot" )
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(true);
    }
    else if ( gt.agents[0] == "human" )
    {
        ui.check_human_info_2->setChecked(true);
        ui.check_robot_info_2->setChecked(false);
    }
    else
    {
        ui.check_human_info_2->setChecked(false);
        ui.check_robot_info_2->setChecked(false);
    }
}

void MainWindow::on_go_to_list_pressed(const QModelIndex &index)
{
    qnode.add_second_location_info( index.row() );
    go_to_action gt = qnode.return_go_to_info( index.row() );
    QString qstr = QString::fromStdString( gt.description );
    ui.edit_action_description->setText( qstr );
    ui.place_list->clearSelection();
    ui.pick_list->clearSelection();
    if ( gt.agents.size() == 2 )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(true);
    }
    else if ( gt.agents.size() == 0 )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
    else if ( gt.agents[0] == "robot" )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(true);
    }
    else if ( gt.agents[0] == "human" )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(false);
    }
    else
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
}

void MainWindow::on_place_list_pressed(const QModelIndex &index)
{
    qnode.add_second_slot_groups( index.row() );
    place pl = qnode.return_place_info( index.row() );
    QString qstr = QString::fromStdString( pl.description );
    ui.edit_action_description->setText( qstr );
    ui.go_to_list->clearSelection();
    ui.pick_list->clearSelection();
    if ( pl.agents.size() == 2 )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(true);
    }
    else if ( pl.agents.size() == 0 )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
    else if ( pl.agents[0] == "robot" )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(true);
    }
    else if ( pl.agents[0] == "human" )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(false);
    }
    else
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
}

void MainWindow::on_pick_list_pressed (const QModelIndex &index)
{
    qnode.add_second_object_type( index.row() );
    pick pk = qnode.return_pick_info( index.row() );
    QString qstr = QString::fromStdString( pk.description );
    ui.edit_action_description->setText( qstr );
    ui.place_list->clearSelection();
    ui.go_to_list->clearSelection();
    if ( pk.agents.size() == 2 )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(true);
    }
    else if ( pk.agents.size() == 0 )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
    else if ( pk.agents[0] == "robot" )
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(true);
    }
    else if ( pk.agents[0] == "human" )
    {
        ui.check_human_info->setChecked(true);
        ui.check_robot_info->setChecked(false);
    }
    else
    {
        ui.check_human_info->setChecked(false);
        ui.check_robot_info->setChecked(false);
    }
}

void MainWindow::on_button_add_leave_position_slot_clicked(bool check)
{
    actual_slot_leave = qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_slot_leave = true;
    ui.button_add_leave_position_slot->setEnabled(false);
    ui.button_remove_leave_position_slot->setEnabled(true);
}

void MainWindow::on_button_add_leave_position_box_clicked(bool check)
{
    actual_box_leave =  qnode.return_position(qnode.base_frame, qnode.target_frame);
    init_box_leave = true;
    ui.button_add_leave_position_box->setEnabled(false);
    ui.button_remove_leave_position_box->setEnabled(true);
}

void MainWindow::on_button_remove_leave_position_slot_clicked(bool check)
{
    init_slot_leave = false;
    ui.button_add_leave_position_slot->setEnabled(true);
    ui.button_remove_leave_position_slot->setEnabled(false);
}

void MainWindow::on_button_remove_leave_position_box_clicked(bool check)
{
    init_box_leave = false;
    ui.button_add_leave_position_box->setEnabled(true);
    ui.button_remove_leave_position_box->setEnabled(false);
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
        ui.go_to_list -> scrollToBottom();
        ui.place_list -> scrollToBottom();
        ui.pick_list  -> scrollToBottom();
        ui.list_go_to -> scrollToBottom();
        ui.list_place -> scrollToBottom();
        ui.list_pick  -> scrollToBottom();
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

