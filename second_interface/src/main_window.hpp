/**
 * @file /include/second_interface/main_window.hpp
 *
 * @brief Qt based gui for second_interface.
 *
 * @date November 2010
 **/
#ifndef second_interface_MAIN_WINDOW_H
#define second_interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace second_interface {

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

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();
    void on_button_remove_element_clicked( bool check );
    void on_button_load_recipe_clicked   ( bool check );
    void on_button_load_actions_clicked  ( bool check );
    void on_button_save_clicked          ( bool check );
    void on_list_place_pressed ( const QModelIndex &index );
    void on_list_pick_pressed  ( const QModelIndex &index );

//	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace second_interface

#endif // second_interface_MAIN_WINDOW_H
