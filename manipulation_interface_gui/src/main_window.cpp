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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    ui.go_to_list           ->setModel(qnode.loggingModelGoTo());
    ui.place_list           ->setModel(qnode.loggingModelPlace());
    ui.pick_list            ->setModel(qnode.loggingModelPick());
    ui.object_list          ->setModel(qnode.loggingModelObject());
    ui.slot_list            ->setModel(qnode.loggingModelSlot());
    ui.group_list           ->setModel(qnode.loggingModelGroup());
    ui.box_list             ->setModel(qnode.loggingModelBox());
    ui.list_groups_place    ->setModel(qnode.loggingModelGrpPlace());
    ui.list_objects_pick    ->setModel(qnode.loggingModelObjPick());
    ui.locations_list       ->setModel(qnode.loggingModelLocation());
    ui.list_locations_goto  ->setModel(qnode.loggingModelLocationGoTo());
    ui.list_location_modify ->setModel(qnode.loggingModelLocationModify());
    ui.list_box_modify      ->setModel(qnode.loggingModelBoxModify());
    ui.list_slot_modify     ->setModel(qnode.loggingModelSlotModify());
    ui.list_object_modify   ->setModel(qnode.loggingModelObjectModify());

    ui.list_go_to  ->setModel(qnode.loggingModelSecondGoto());
    ui.list_place  ->setModel(qnode.loggingModelSecondPlace());
    ui.list_pick   ->setModel(qnode.loggingModelSecondPick());
    ui.list_recipe ->setModel(qnode.loggingModelRecipe());
    ui.list_object ->setModel(qnode.loggingModelSecondObjects());
    ui.list_slot   ->setModel(qnode.loggingModelSecondSlots());

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.object_list    ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.slot_list      ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.group_list     ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.box_list       ->setSelectionMode(QAbstractItemView::MultiSelection);
    ui.locations_list ->setSelectionMode(QAbstractItemView::MultiSelection);
    //    ui.place_list  ->setSelectionMode(QAbstractItemView::MultiSelection);
    //    ui.pick_list   ->setSelectionMode(QAbstractItemView::MultiSelection);

    ui.list_recipe ->setDragDropMode(QAbstractItemView::DragDrop);
//    ui.list_recipe->model()->doSetSupportedDragActions(Qt::MoveAction);

    ui.list_go_to->setDragEnabled(true);
    ui.list_place->setDragEnabled(true);
    ui.list_pick ->setDragEnabled(true);

    ui.button_add_approach_box           ->setEnabled(false);
    ui.button_add_approach_object        ->setEnabled(false);
    ui.button_add_approach_slot          ->setEnabled(false);
    ui.button_add_final_box              ->setEnabled(false);
    ui.button_add_final_position_slot    ->setEnabled(false);
    ui.button_add_go_to                  ->setEnabled(false);
    ui.button_add_grasp                  ->setEnabled(false);
    ui.button_add_pick                   ->setEnabled(false);
    ui.button_add_place                  ->setEnabled(false);
    ui.button_remove_approach_object     ->setEnabled(false);
    ui.button_remove_approach_slot       ->setEnabled(false);
    ui.button_remove_final_position_slot ->setEnabled(false);
    ui.button_remove_approach_box        ->setEnabled(false);
    ui.button_remove_final_box           ->setEnabled(false);
    ui.grasp_list                        ->setEnabled(false);
    ui.robot_list                        ->setEnabled(false);
    ui.TF_list                           ->setEnabled(false);
    ui.button_remove_grasp               ->setEnabled(false);
    ui.button_save_components            ->setEnabled(false);
    ui.button_save_box                   ->setEnabled(false);
    ui.button_save_object                ->setEnabled(false);
    ui.button_save_slot                  ->setEnabled(false);
    ui.edit_action_name                  ->setEnabled(false);
    ui.edit_box_name                     ->setEnabled(false);
    ui.edit_group_name                   ->setEnabled(false);
    ui.edit_max_object                   ->setEnabled(false);
    ui.edit_object_name                  ->setEnabled(false);
    ui.edit_slot_name                    ->setEnabled(false);
    ui.button_remove_object              ->setEnabled(false);
    ui.button_remove_group               ->setEnabled(false);
    ui.button_remove_go_to               ->setEnabled(false);
    ui.button_remove_box                 ->setEnabled(false);
    ui.button_remove_pick                ->setEnabled(false);
    ui.button_remove_place               ->setEnabled(false);
    ui.button_remove_slot                ->setEnabled(false);
    ui.button_manual_guidance            ->setEnabled(false);
    ui.check_gripper                     ->setEnabled(false);
    ui.check_manual_guidance             ->setEnabled(false);
    ui.button_gripper                    ->setEnabled(false);
    ui.button_load                       ->setEnabled(false);
    ui.button_load_TF                    ->setEnabled(false);
    ui.approach_list                     ->setEnabled(false);
    ui.edit_location_name                ->setEnabled(false);
    ui.button_add_location               ->setEnabled(false);

    ui.go_to_list           ->setEditTriggers(QListView::NoEditTriggers);
    ui.place_list           ->setEditTriggers(QListView::NoEditTriggers);
    ui.pick_list            ->setEditTriggers(QListView::NoEditTriggers);
    ui.object_list          ->setEditTriggers(QListView::NoEditTriggers);
    ui.slot_list            ->setEditTriggers(QListView::NoEditTriggers);
    ui.group_list           ->setEditTriggers(QListView::NoEditTriggers);
    ui.box_list             ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_groups_place    ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_objects_pick    ->setEditTriggers(QListView::NoEditTriggers);
    ui.locations_list       ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_locations_goto  ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_location_modify ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_box_modify      ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_slot_modify     ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_object_modify   ->setEditTriggers(QListView::NoEditTriggers);

    ui.list_go_to  ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_place  ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_pick   ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_recipe ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_object ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_slot   ->setEditTriggers(QListView::NoEditTriggers);

    ui.edit_location_frame ->setReadOnly(true);
    ui.edit_slot_frame     ->setReadOnly(true);
    ui.edit_slot_group     ->setReadOnly(true);
    ui.edit_box_frame      ->setReadOnly(true);
    ui.edit_object_tool    ->setReadOnly(true);

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
        ui.TF_list->addItem(tf);
        ui.world_TF_list->addItem(tf);
    }
    for ( int i = 0; i < qnode.robots.size(); i++)
    {
        tf = QString::fromStdString(qnode.robots[i]);
        ui.robot_list->addItem(tf);
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

void MainWindow::on_button_add_go_to_clicked(bool check)
{
    std::string go_to_name=ui.edit_action_name->text().toStdString();
    std::vector<std::string> locations;
    QModelIndexList indexes =  ui.locations_list->selectionModel()->selectedIndexes();
    if ( !indexes.empty() )
    {
        if ( ! go_to_name.empty() )
        {
            for ( int i = 0; i < indexes.size(); i++)
            {
                locations.push_back( qnode.return_location_list_text( indexes.at(i).row() ) );
            }
            if ( !qnode.add_go_to(go_to_name,locations) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another goto with the same name or locations.");
                msgBox.exec();
                ui.edit_action_name->clear();
            }
            else
            {
                ui.edit_action_name->clear();
                ui.locations_list->clearSelection();
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

void MainWindow::on_button_add_place_clicked(bool check)
{
    std::string place_name = ui.edit_action_name->text().toStdString();
    std::vector<std::string> groups;
    QModelIndexList indexes =  ui.group_list->selectionModel()->selectedIndexes();
    if ( !indexes.empty() )
    {
        if ( !place_name.empty())
        {
            for ( int i = 0; i < indexes.size(); i++)
            {
                groups.push_back( qnode.return_group_list_text( indexes.at(i).row() ) );
            }
            if ( !qnode.add_place(place_name,groups) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another place with the same name or groups.");
                msgBox.exec();
                ui.edit_action_name->clear();
            }
            else
            {
                ui.edit_action_name->clear();
                ui.group_list->clearSelection();
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

void MainWindow::on_button_add_pick_clicked(bool check)
{
    std::string pick_name = ui.edit_action_name->text().toStdString();
    std::vector<std::string> objects;
    QModelIndexList indexes =  ui.object_list->selectionModel()->selectedIndexes();
    if ( !pick_name.empty())
    {
        if ( !indexes.empty())
        {
            for ( int i = 0; i < indexes.size(); i++)
            {
                objects.push_back(qnode.return_object_list_text( indexes.at(i).row() ) );
            }
            if ( !qnode.add_pick(pick_name,objects) )
            {
                QMessageBox msgBox;
                msgBox.setText("There is another pick with the same name or objects.");
                msgBox.exec();
                ui.edit_action_name->clear();
            }
            else
            {
                ui.edit_action_name->clear();
                ui.object_list->clearSelection();
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

void MainWindow::on_button_add_approach_object_clicked(bool check)
{
    std::string actual_base_frame = ui.TF_list->currentText().toStdString();
    location loc;
    position pos;
    loc = qnode.return_position(actual_base_frame, qnode.target_frame);
    pos.origin_x = loc.pos.origin_x;
    pos.origin_y = loc.pos.origin_y;
    pos.origin_z = loc.pos.origin_z;
    actual_object_approach.push_back(pos);
    actual_tool_approach.push_back(qnode.target_frame);
    num_approach++;
    std::string str = "approach";
    str.append(std::to_string(num_approach));
    QString approach;
    approach.append(str.c_str());
    ui.approach_list->addItem(approach);
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
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Empty name");
        msgBox.exec();
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
        QAbstractItemModel* model = ui.go_to_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.at(i);
            qnode.remove_go_to(index.row());
            model->removeRow(index.row());
            indexes =  ui.go_to_list->selectionModel()->selectedIndexes();
        }
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

        QAbstractItemModel* model = ui.locations_list->model();
        QAbstractItemModel* model_ = ui.locations_list->model();
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
        QAbstractItemModel* model = ui.place_list->model();;
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_place(index.row());
            model->removeRow(index.row());
            indexes =  ui.place_list->selectionModel()->selectedIndexes();
            ui.list_groups_place->model()->removeRows( 0, ui.list_groups_place->model()->rowCount() );
        }
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
        QAbstractItemModel* model = ui.pick_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_pick(index.row());
            model->removeRow(index.row());
            indexes =  ui.pick_list->selectionModel()->selectedIndexes();
            ui.list_objects_pick->model()->removeRows( 0, ui.list_objects_pick->model()->rowCount() );
        }
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
        QAbstractItemModel* model = ui.object_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_object(index.row());
            model->removeRow(index.row());
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
        QAbstractItemModel* model = ui.box_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_box(index.row());
            model->removeRow(index.row());
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
        QAbstractItemModel* model = ui.group_list->model();
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
        QAbstractItemModel* model = ui.slot_list->model();
        for ( int i = indexes.size()-1 ; i >= 0; i--)
        {
            index = indexes.first();
            qnode.remove_slot(index.row());
            model->removeRow(index.row());
            indexes =  ui.slot_list->selectionModel()->selectedIndexes();
        }
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected slot");
    msgBox.exec();
}

void MainWindow::on_button_remove_approach_object_clicked(bool check)
{
    int index = ui.approach_list->currentIndex();

    if ( ui.approach_list->count() > 0 )
    {
        ui.approach_list->removeItem(index);
        actual_object_approach.erase(actual_object_approach.begin()+index);
        actual_tool_approach.erase(actual_tool_approach.begin()+index);
        return;
    }

    QMessageBox msgBox;
    msgBox.setText("There isn't a selected approach");
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

void MainWindow::on_button_manual_guidance_clicked(bool check)
{
    ui.check_manual_guidance->click();
}

void MainWindow::on_button_gripper_clicked(bool check)
{
    ui.check_gripper->click();
    return;
}

void MainWindow::on_button_save_components_clicked(bool check)
{
    if ( qnode.save_components() )
    {
        QMessageBox msgBox;
        msgBox.setText("The position are saved.");
        msgBox.exec();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("There is some problem with the save");
        msgBox.exec();
    }
}

void MainWindow::on_button_save_object_clicked(bool check)
{ 
    QString object_name = ui.edit_object_name->text();
    std::string obj_name = object_name.toStdString();

    if ( !obj_name.empty() )
    {

        if ( !actual_object_grasp.empty() )
        {
            if ( actual_object_grasp.size() == actual_object_approach.size() )
            {
                if ( actual_tool_approach == actual_tool_grasp )
                {
                    if( !qnode.save_object(obj_name, actual_object_approach, actual_object_grasp, actual_tool_grasp))
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
                        ui.button_add_approach_object->setEnabled(true);
                        ui.button_remove_approach_object->setEnabled(false);
                        actual_object_grasp.clear();
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
                ROS_ERROR("Apporach and grasp have different sizes");
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

void MainWindow::on_button_save_slot_clicked(bool check)
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
                            if( !qnode.save_slot(slot_name, actual_slot_approach, actual_slot_final_position, group_name, num_max_obj ) )
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

void MainWindow::on_button_save_box_clicked(bool check)
{
    std::string box_name = ui.edit_box_name->text().toStdString();
    if ( init_box_approach )
    {
        if ( init_box_final )
        {
            if ( !box_name.empty() )
            {
                if( !qnode.save_box( box_name, actual_box_approach, actual_box_final ) )
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

void MainWindow::on_button_save_location_changes_clicked(bool check)
{
    location loc;

    QModelIndex index = ui.list_location_modify->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6;
    loc.pos.origin_x    = ui.edit_location_position_x->text().toDouble(&ok0);
    loc.pos.origin_y    = ui.edit_location_position_y->text().toDouble(&ok1);
    loc.pos.origin_z    = ui.edit_location_position_z->text().toDouble(&ok2);
    loc.quat.rotation_x = ui.edit_location_quaternion_x->text().toDouble(&ok3);
    loc.quat.rotation_y = ui.edit_location_quaternion_y->text().toDouble(&ok4);
    loc.quat.rotation_z = ui.edit_location_quaternion_z->text().toDouble(&ok5);
    loc.quat.rotation_w = ui.edit_location_quaternion_w->text().toDouble(&ok6);

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6)
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't double");
        msgBox.exec();
    }
    else
    {
        qnode.save_location_changes( index.row(), loc);
    }
}

void MainWindow::on_button_save_slot_changes_clicked(bool check)
{
    slot slt;

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

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 || !ok10 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't numers");
        msgBox.exec();
    }
    else
    {
        qnode.save_slot_changes( index.row(), slt);
    }
}

void MainWindow::on_button_save_box_changes_clicked(bool check)
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

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't numers");
        msgBox.exec();
    }
    else
    {
        qnode.save_box_changes( index.row(), bx);
    }
}

void MainWindow::on_button_save_object_changes_clicked(bool check)
{
    location loc;
    position appr;

    QModelIndex index = ui.list_object_modify->currentIndex();
    int index2 = ui.combo_grasp_number->currentIndex();

    bool ok0, ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8, ok9;
    loc.pos.origin_x    = ui.edit_object_position_x   ->text().toDouble(&ok0);
    loc.pos.origin_y    = ui.edit_object_position_y   ->text().toDouble(&ok1);
    loc.pos.origin_z    = ui.edit_object_position_z   ->text().toDouble(&ok2);
    loc.quat.rotation_x = ui.edit_object_quaternion_x ->text().toDouble(&ok3);
    loc.quat.rotation_y = ui.edit_object_quaternion_y ->text().toDouble(&ok4);
    loc.quat.rotation_z = ui.edit_object_quaternion_z ->text().toDouble(&ok5);
    loc.quat.rotation_w = ui.edit_object_quaternion_w ->text().toDouble(&ok6);
    appr.origin_x       = ui.edit_object_approach_x   ->text().toDouble(&ok7);
    appr.origin_y       = ui.edit_object_approach_y   ->text().toDouble(&ok8);
    appr.origin_z       = ui.edit_object_approach_z   ->text().toDouble(&ok9);

    if ( !ok0 || !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 || !ok7 || !ok8 || !ok9 )
    {
        QMessageBox msgBox;
        msgBox.setText("One or more number aren't double");
        msgBox.exec();
    }
    else
    {
        qnode.save_object_changes( index.row(), index2, loc, appr);
    }
}

void MainWindow::on_button_load_TF_clicked(bool chack)
{
    qnode.load_TF();

    QString tf;
    ui.TF_list->clear();
    for ( int i = 0; i < qnode.TFs.size(); i++ )
    {
        tf = QString::fromStdString(qnode.TFs[i]);
        ui.TF_list->addItem(tf);
    }
}

void MainWindow::on_button_load_clicked(bool chack)
{
    qnode.write_param();
}

void MainWindow::on_check_robot_TF_stateChanged(int state)
{
    if ( ! state == 0 )
    {
        ui.button_add_approach_box        ->setEnabled(true);
        ui.button_add_approach_object     ->setEnabled(true);
        ui.button_add_approach_slot       ->setEnabled(true);
        ui.button_add_final_box           ->setEnabled(true);
        ui.button_add_final_position_slot ->setEnabled(true);
        ui.button_add_go_to               ->setEnabled(true);
        ui.button_add_grasp               ->setEnabled(true);
        ui.button_add_pick                ->setEnabled(true);
        ui.button_add_place               ->setEnabled(true);
        ui.button_save_components         ->setEnabled(true);
        ui.button_save_box                ->setEnabled(true);
        ui.button_save_object             ->setEnabled(true);
        ui.button_save_slot               ->setEnabled(true);
        ui.button_save_box                ->setEnabled(true);
        ui.button_save_object             ->setEnabled(true);
        ui.button_save_slot               ->setEnabled(true);
        ui.edit_action_name               ->setEnabled(true);
        ui.edit_box_name                  ->setEnabled(true);
        ui.edit_group_name                ->setEnabled(true);
        ui.edit_max_object                ->setEnabled(true);
        ui.edit_object_name               ->setEnabled(true);
        ui.edit_slot_name                 ->setEnabled(true);
        ui.button_remove_object           ->setEnabled(true);
        ui.button_remove_group            ->setEnabled(true);
        ui.button_remove_go_to            ->setEnabled(true);
        ui.button_remove_box              ->setEnabled(true);
        ui.button_remove_pick             ->setEnabled(true);
        ui.button_remove_place            ->setEnabled(true);
        ui.button_remove_slot             ->setEnabled(true);
        ui.grasp_list                     ->setEnabled(true);
        ui.robot_list                     ->setEnabled(true);
        ui.button_remove_grasp            ->setEnabled(true);
        ui.TF_list                        ->setEnabled(true);
        ui.button_remove_approach_object  ->setEnabled(true);
        ui.button_manual_guidance         ->setEnabled(true);
        ui.button_gripper                 ->setEnabled(true);
        ui.check_gripper                  ->setEnabled(true);
        ui.check_manual_guidance          ->setEnabled(true);
        ui.button_load                    ->setEnabled(true);
        ui.button_load_TF                 ->setEnabled(true);
        ui.approach_list                  ->setEnabled(true);
        ui.edit_location_name             ->setEnabled(true);
        ui.button_add_location            ->setEnabled(true);


        ui.check_robot_TF ->setEnabled(false);
        ui.world_TF_list  ->setEnabled(false);

        qnode.base_frame = ui.world_TF_list->currentText().toStdString();
    }
}

void MainWindow::on_check_gripper_stateChanged(int state)
{
    if ( ! state == 0)
    {
        qnode.close_gripper(true);
    }
    else
    {
        qnode.close_gripper(false);
    }
    return;
}

void MainWindow::on_check_manual_guidance_stateChanged(int state)
{
    if ( ! state == 0 )
    {
        qnode.active_manual_guidance(true);
    }
    else
    {
        qnode.active_manual_guidance(false);
    }
}

void MainWindow::on_robot_list_currentIndexChanged(int index)
{
    qnode.set_target_frame( ui.robot_list->currentIndex() );
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
    }
}


void MainWindow::on_place_list_pressed(const QModelIndex &index)
{
////    for multiselection
//    if ( ui.place_list->selectionModel()->selectedIndexes().size() == 1 )
//    {
//        qnode.add_groups_place( ui.place_list->selectionModel()->selectedIndexes().at(0).row() );
//    }
//    else
//    {
//        ui.list_groups_place->model()->removeRows( 0, ui.list_groups_place->model()->rowCount() );
//    }
//    for simpleselection
    ui.list_groups_place->model()->removeRows( 0, ui.list_groups_place->model()->rowCount() );
    qnode.add_groups_place( ui.place_list->selectionModel()->selectedIndexes().at(0).row() );
}

void MainWindow::on_pick_list_pressed(const QModelIndex &index)
{
////    for multiselection
//    if ( ui.pick_list->selectionModel()->selectedIndexes().size() == 1 )
//    {
//        qnode.add_objects_pick ( ui.pick_list->selectionModel()->selectedIndexes().at(0).row() );
//    }
//    else
//    {
//        ui.list_objects_pick->model()->removeRows( 0, ui.list_objects_pick->model()->rowCount() );
//    }
//    for simpleselection
    ui.list_objects_pick->model()->removeRows( 0, ui.list_objects_pick->model()->rowCount() );
    qnode.add_objects_pick ( ui.pick_list->selectionModel()->selectedIndexes().at(0).row() );
}

void MainWindow::on_go_to_list_pressed(const QModelIndex &index)
{
    ui.list_locations_goto->model()->removeRows( 0, ui.list_locations_goto->model()->rowCount() );
    qnode.add_locations_go_to ( ui.go_to_list->selectionModel()->selectedIndexes().at(0).row() );
}

void MainWindow::on_list_location_modify_pressed(const QModelIndex &index)
{
    ui.edit_location_frame        ->clear();
    ui.edit_location_position_x   ->clear();
    ui.edit_location_position_y   ->clear();
    ui.edit_location_position_z   ->clear();
    ui.edit_location_quaternion_w ->clear();
    ui.edit_location_quaternion_x ->clear();
    ui.edit_location_quaternion_y ->clear();
    ui.edit_location_quaternion_z ->clear();

    go_to_location loc = qnode.return_location_info(index.row());
    QString frame_ = QString::fromStdString( loc.frame);
    QString pos_x  = QString::fromStdString( std::to_string(loc.location_.pos.origin_x) );
    QString pos_y  = QString::fromStdString( std::to_string(loc.location_.pos.origin_y) );
    QString pos_z  = QString::fromStdString( std::to_string(loc.location_.pos.origin_z) );
    QString quat_x = QString::fromStdString( std::to_string(loc.location_.quat.rotation_x) );
    QString quat_y = QString::fromStdString( std::to_string(loc.location_.quat.rotation_y) );
    QString quat_z = QString::fromStdString( std::to_string(loc.location_.quat.rotation_z) );
    QString quat_w = QString::fromStdString( std::to_string(loc.location_.quat.rotation_w) );
    ui.edit_location_frame         ->insert(frame_);
    ui.edit_location_position_x    ->insert(pos_x);
    ui.edit_location_position_y    ->insert(pos_y);
    ui.edit_location_position_z    ->insert(pos_z);
    ui.edit_location_quaternion_w  ->insert(quat_w);
    ui.edit_location_quaternion_x  ->insert(quat_x);
    ui.edit_location_quaternion_y  ->insert(quat_y);
    ui.edit_location_quaternion_z  ->insert(quat_z);
}

void MainWindow::on_list_box_modify_pressed(const QModelIndex &index)
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

    box bx = qnode.return_box_info(index.row());
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
    ui.edit_box_frame         ->insert(frame_);
    ui.edit_box_position_x    ->insert(pos_x);
    ui.edit_box_position_y    ->insert(pos_y);
    ui.edit_box_position_z    ->insert(pos_z);
    ui.edit_box_quaternion_w  ->insert(quat_w);
    ui.edit_box_quaternion_x  ->insert(quat_x);
    ui.edit_box_quaternion_y  ->insert(quat_y);
    ui.edit_box_quaternion_z  ->insert(quat_z);
    ui.edit_box_approach_x         ->insert(appr_x);
    ui.edit_box_approach_y         ->insert(appr_y);
    ui.edit_box_approach_z         ->insert(appr_z);
}
void MainWindow::on_list_slot_modify_pressed(const QModelIndex &index)
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

    slot sl = qnode.return_slot_info(index.row());
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
}

void MainWindow::on_list_object_modify_pressed(const QModelIndex &index)
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

    actual_object_to_modify = qnode.return_object_info(index.row());

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
    QString tool   = QString::fromStdString( actual_object_to_modify.tool[i] );

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
    if ( ui.recipe_box->count() == 0 )
    {
        std::vector<std::string> recipes_names = qnode.load_recipe(false, " ");
        if ( recipes_names.empty() )
        {
            QMessageBox msgBox;
            msgBox.setText("Recipes_param is empty.");
            msgBox.exec();
            return;
        }
        for ( int i = 0; i < recipes_names.size(); i++)
        {
            ui.recipe_box->addItem( QString::fromStdString(recipes_names[i]) );
        }
    }
    else
    {
        std::string name_recipe = ui.recipe_box->currentText().toStdString();
        qnode.load_recipe(true, name_recipe);
    }
}

void MainWindow::on_button_load_actions_clicked  ( bool check )
{
    qnode.load_actions();
}

void MainWindow::on_button_save_recipe_clicked ( bool check )
{
    std::string recipe_name = ui.edit_recipe_name->text().toStdString();
    if ( recipe_name.empty() )
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe_name is empty.");
        msgBox.exec();
        return;
    }
    else if ( !recipe_name.compare("recipe"))
    {
        QMessageBox msgBox;
        msgBox.setText("Change name, 'recipe' is a taboo.");
        msgBox.exec();
        return;
    }
    if ( ui.list_recipe->model()->rowCount() == 0 )
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe is empty.");
        msgBox.exec();
        return;
    }

    if ( qnode.save_recipe( recipe_name ) )
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
    std::vector<std::string> recipes_names = qnode.load_recipe(false, " ");
    if ( recipes_names.empty() )
    {
        QMessageBox msgBox;
        msgBox.setText("Recipes_param is empty.");
        msgBox.exec();
        return;
    }
    ui.recipe_box->clear();
    for ( int i = 0; i < recipes_names.size(); i++)
    {
        ui.recipe_box->addItem( QString::fromStdString(recipes_names[i]) );
    }

}

void MainWindow::on_button_set_recipe_clicked ( bool check )
{
    if ( ui.list_recipe->model()->rowCount() == 0 )
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe is empty.");
        msgBox.exec();
        return;
    }

    if ( !qnode.set_recipe() )
    {
        QMessageBox msgBox;
        msgBox.setText("There are one or more actions in recipe that there aren't in saved actions.");
        msgBox.exec();
        return;
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("The recipe is setted.");
        msgBox.exec();
        ui.edit_recipe_name->clear();
    }
}

void MainWindow::on_list_place_pressed(const QModelIndex &index)
{
    qnode.add_slot_groups( index.row() );
}

void MainWindow::on_list_pick_pressed(const QModelIndex &index)
{
    qnode.add_object_type( index.row() );
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

