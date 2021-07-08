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

namespace second_interface {

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
    ui.list_go_to  ->setModel(qnode.loggingModelGoto());
    ui.list_place  ->setModel(qnode.loggingModelPlace());
    ui.list_pick   ->setModel(qnode.loggingModelPick());
    ui.list_recipe ->setModel(qnode.loggingModelRecipe());
    ui.list_object ->setModel(qnode.loggingModelObjects());
    ui.list_slot   ->setModel(qnode.loggingModelSlots());

    ui.list_go_to  ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_place  ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_pick   ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_recipe ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_object ->setEditTriggers(QListView::NoEditTriggers);
    ui.list_slot   ->setEditTriggers(QListView::NoEditTriggers);

    ui.list_recipe ->setDragDropMode(QAbstractItemView::DragDrop);
//    ui.list_recipe->model()->doSetSupportedDragActions(Qt::MoveAction);

    ui.list_go_to->setDragEnabled(true);
    ui.list_place->setDragEnabled(true);
    ui.list_pick->setDragEnabled(true);

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Window connect to ROS
    **********************/
    if ( !qnode.init() ) {
        showNoMasterMessage();
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
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */


/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
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
        ROS_ERROR("son detro il load recipe");
        std::string name_recipe = ui.recipe_box->currentText().toStdString();
        ROS_ERROR("name_recipe: %s", name_recipe.c_str());
        qnode.load_recipe(true, name_recipe);
    }
}

void MainWindow::on_button_load_actions_clicked  ( bool check )
{
    qnode.load_actions();
}

void MainWindow::on_button_save_clicked ( bool check )
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

    if ( qnode.save_all( recipe_name ) )
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

void MainWindow::on_list_place_pressed(const QModelIndex &index)
{
    qnode.add_slot_groups( index.row() );
}

void MainWindow::on_list_pick_pressed(const QModelIndex &index)
{
    qnode.add_object_type( index.row() );
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace second_interface

