/**
 * @file /include/second_interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef second_interface_QNODE_HPP_
#define second_interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace second_interface {

struct go__to
{
    std::string frame;
    std::string name;
};

struct place
{
    std::string name;
    std::vector<std::string> groups;
};

struct pick
{
    std::string name;
    std::vector<std::string> objects;
};

struct action
{
    std::string type;
    int index;
};

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
    void load_actions();
    void add_object_type (int ind);
    void add_slot_groups (int ind);
    std::vector<std::string> load_recipe(bool init, std::string name_recipe);
    bool save_recipe(std::string recipe_name);

    std::string get_xml_string_param ( std::string param_name, std::string value );
    std::string get_xml_group_string ( std::string name, std::vector<std::string> string_group );
    XmlRpc::XmlRpcValue get_action_go_to_param (int index);
    XmlRpc::XmlRpcValue get_action_place_param (int index);
    XmlRpc::XmlRpcValue get_action_pick_param  (int index);

    /*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

    QStringListModel* loggingModelSecondGoto()   { return &logging_model_second_go_to; }
    QStringListModel* loggingModelSecondPlace()  { return &logging_model_second_place; }
    QStringListModel* loggingModelSecondPick()   { return &logging_model_second_pick; }
    QStringListModel* loggingModelSecondObjects(){ return &logging_model_second_objects; }
    QStringListModel* loggingModelSecondSlots()  { return &logging_model_second_slots; }
    QStringListModel* loggingModelRecipe()       { return &logging_model_recipe; }
    void log_second_go_to  (const std::string &msg);
    void log_second_place  (const std::string &msg);
    void log_second_pick   (const std::string &msg);
    void log_second_objects(const std::string &msg);
    void log_second_slots  (const std::string &msg);
    void log_recipe (const std::string &msg);
    bool compare(std::vector<std::string>& v1, std::vector<std::string>& v2);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    QStringListModel logging_model_second_go_to;
    QStringListModel logging_model_second_place;
    QStringListModel logging_model_second_pick;
    QStringListModel logging_model_second_objects;
    QStringListModel logging_model_second_slots;
    QStringListModel logging_model_recipe;
    std::vector<go__to> go_to_actions;
    std::vector<place> place_actions;
    std::vector<pick>  pick_actions;
    std::vector<action> action_list;

    std::vector<std::string> param_names;

    const std::string init_value  = "<value>";
    const std::string end_value   = "</value>";
    const std::string init_struct = "<struct>";
    const std::string end_struct  = "</struct>";
    const std::string init_member = "<member>";
    const std::string end_member  = "</member>";
    const std::string init_name   = "<name>";
    const std::string end_name    = "</name>";
    const std::string init_array  = "<array>";
    const std::string end_array   = "</array>";
    const std::string init_data   = "<data>";
    const std::string end_data    = "</data>";
    const std::string init_double = "<double>";
    const std::string end_double  = "</double>";
    const std::string init_int    = "<int>";
    const std::string end_int     = "</int>";
    const std::string init_string = "<string>";
    const std::string end_string  = "</string>";
};

}  // namespace second_interface

#endif /* second_interface_QNODE_HPP_ */
