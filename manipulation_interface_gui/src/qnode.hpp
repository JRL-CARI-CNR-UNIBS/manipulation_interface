/**
 * @file /include/manipulation_interface_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef manipulation_interface_gui_QNODE_HPP_
#define manipulation_interface_gui_QNODE_HPP_

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
#include <configuration_msgs/StartConfiguration.h>
#include <manipulation_msgs/JobExecution.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

struct position
{
    double origin_x;
    double origin_y;
    double origin_z;
};

struct quaternion
{
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
};

struct location
{
    position pos;
    quaternion quat;
};

struct go_to_location
{
    std::string name;
    std::string frame;
    location    location_;
};

struct go_to
{
    std::string name;
    std::vector<std::string> locations;
    std::string description;
    std::vector<std::string> agents;
};

struct place
{
    std::string name;
    std::vector<std::string> groups;
    std::string description;
    std::vector<std::string> agents;
};

struct pick
{
    std::string name;
    std::vector<std::string> objects;
    std::string description;
    std::vector<std::string> agents;
};

struct object_type
{
    std::string              name;
    std::vector<position>    approach;
    std::vector<location>    grasp;
    std::vector<std::string> tool;
};

struct slot
{
    std::string name;
    std::string group;
    position    approach;
    location    location_;
    int         max_objects;
    std::string frame;
};

struct box
{
    std::string name;
    location    location_;
    position    approach;
    std::string frame;
};

struct action
{
    std::string name;
    std::string type;
    int index;
};

struct recipe
{
    std::string name;
    std::vector<std::string> recipe_;
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

    void cartMove (std::vector<float> twist_move);

    bool save_recipe();
    std::vector<std::string> load_recipes_param();

    XmlRpc::XmlRpcValue get_recipe_param       (int index);
    XmlRpc::XmlRpcValue get_action_go_to_param (int index);
    XmlRpc::XmlRpcValue get_action_place_param (int index);
    XmlRpc::XmlRpcValue get_action_pick_param  (int index);

    bool add_location (std::string location_name);
    bool add_recipe   (std::string recipe_name);
    bool add_go_to    (std::string go_to_name, std::vector<std::string> locations_, std::string description, std::vector<std::string> agents_);
    bool add_place    (std::string place_name, std::vector<std::string> groups_, std::string description, std::vector<std::string> agents_);
    bool add_pick     (std::string pick_name, std::vector<std::string> objects_, std::string description, std::vector<std::string> agents_);
    bool add_object   (std::string object_name, std::vector<position> object_approach, std::vector<location> object_grasp, std::vector<std::string> object_tools);
    bool add_slot     (std::string slot_name, location slot_approach, location slot_final_pos, std::string goup_name, int max_number);
    bool add_box      (std::string box_name, location approach_position, location final_position);
    void add_object_type          (int ind);
    void add_slot_groups          (int ind);
    void add_location_info        (int ind);
    bool add_objects_pick         (int ind);
    bool add_groups_place         (int ind);
    bool add_locations_go_to      (int ind);
    bool add_second_location_info (int ind);
    bool add_second_slot_groups   (int ind);
    bool add_second_object_type   (int ind);
    bool add_location_changes     (int ind, go_to_location new_location);
    bool add_slot_changes         (int ind, slot new_slot);
    bool add_box_changes          (int ind, box new_box);
    bool add_object_changes       (int ind, object_type new_object);
    bool add_object_copy_grasp    (int index, int index2);
    bool add_location_copy (go_to_location new_loc);
    bool add_object_copy   (object_type new_obj);
    bool add_slot_copy     (slot new_slot);
    bool add_box_copy      (box new_box);


    location       return_position(std::string base_frame, std::string target_frame);
    go_to          return_go_to_info         (int ind);
    pick           return_pick_info          (int ind);
    place          return_place_info         (int ind);
    go_to_location return_location_info      (int ind);
    object_type    return_object_info        (int ind);
    box            return_box_info           (int ind);
    slot           return_slot_info          (int ind);
    recipe         return_recipe_info        (int ind);
    std::string    return_location_list_text (int ind);
    std::string    return_group_list_text    (int ind);
    std::string    return_object_list_text   (int ind);
    std::string    return_obj_dist_list_text (int ind);
    std::string    return_box_list_text      (int ind);

    std::string get_xml_max_number_string         (int value);
    std::string get_xml_double_string             (double value);
    std::string get_xml_string_param              (std::string param_name, std::string value);
    std::string get_xml_position_string           (std::string name_pos,  position pos);
    std::string get_xml_quaternion_string         (quaternion quat);
    std::string get_xml_group_string              (std::string name, std::vector<std::string> string_group);
    std::string get_xml_object_grasp_string       (int index, int index2);
    std::string get_xml_object_grasp_poses_string (int index);

    XmlRpc::XmlRpcValue get_object_grasp_param   (int index, int index2);
    XmlRpc::XmlRpcValue get_object_param         (int index);
    XmlRpc::XmlRpcValue get_go_to_location_param (int index);
    XmlRpc::XmlRpcValue get_box_param            (int index);
    XmlRpc::XmlRpcValue get_group_param          (int index);
    XmlRpc::XmlRpcValue get_object_name_param    (int index);
    XmlRpc::XmlRpcValue get_pick_param           (int index);
    XmlRpc::XmlRpcValue get_place_param          (int index);
    XmlRpc::XmlRpcValue get_slot_param           (int index);
    XmlRpc::XmlRpcValue get_go_to_param_         (int index);
    XmlRpc::XmlRpcValue get_pick_param_          (int index);
    XmlRpc::XmlRpcValue get_place_param_         (int index);

    void load_TF                       ();
    void load_robots                   ();
    void write_groups                  ();
    void write_objects                 ();
    void write_locations               ();
    void check_other_param             ();
    void check_objects_param           ();
    void check_recipes_param           ();
    void write_go_to_location_server   ();
    bool save_actions                  ();
    bool save_components               ();
    bool readBoxesFromParam            ();
    bool readSlotsFromParam            ();
    bool readObjectFromParam           ();
    bool readLocationsFromParam        ();
    bool readSlotsGroupFromParam       ();
    bool readGotoPickAndPlaceFromParam ();

    void set_target_frame (int ind);
    void load_param       (int ind);
    void write_param      (int ind);
    void write_recipe     (int ind);
    bool remove_recipe    (int ind);

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

    QStringListModel* loggingModelGoTo             () {return &logging_model_go_to;}
    QStringListModel* loggingModelPlace            () {return &logging_model_place;}
    QStringListModel* loggingModelPick             () {return &logging_model_pick;}
    QStringListModel* loggingModelObject           () {return &logging_model_object;}
    QStringListModel* loggingModelSlot             () {return &logging_model_slot;}
    QStringListModel* loggingModelBox              () {return &logging_model_box;}
    QStringListModel* loggingModelGroup            () {return &logging_model_group;}
    QStringListModel* loggingModelLocation         () {return &logging_model_location;}
    QStringListModel* loggingModelObjectModify     () {return &logging_model_object_modify;}
    QStringListModel* loggingModelSlotModify       () {return &logging_model_slot_modify;}
    QStringListModel* loggingModelBoxModify        () {return &logging_model_box_modify;}
    QStringListModel* loggingModelLocationModify   () {return &logging_model_location_modify;}
    QStringListModel* loggingModelComponents       () {return &logging_model_components;}
    QStringListModel* loggingModelInfoAction       () {return &logging_model_info_action;}
    QStringListModel* loggingModelSecondGoto       () {return &logging_model_second_go_to;}
    QStringListModel* loggingModelSecondPlace      () {return &logging_model_second_place;}
    QStringListModel* loggingModelSecondPick       () {return &logging_model_second_pick;}
    QStringListModel* loggingModelRecipe           () {return &logging_model_recipe;}
    QStringListModel* loggingModelActionComponents () {return &logging_model_action_components;}
    void log_go_to            (const std::string &msg);
    void log_place            (const std::string &msg);
    void log_pick             (const std::string &msg);
    void log_object           (const std::string &msg);
    void log_slot             (const std::string &msg);
    void log_box              (const std::string &msg);
    void log_group            (const std::string &msg);
    void log_location         (const std::string &msg);
    void log_object_modify    (const std::string &msg);
    void log_slot_modify      (const std::string &msg);
    void log_box_modify       (const std::string &msg);
    void log_location_modify  (const std::string &msg);
    void log_components       (const std::string &msg);
    void log_info_action      (const std::string &msg);
    void log_second_go_to     (const std::string &msg);
    void log_second_place     (const std::string &msg);
    void log_second_pick      (const std::string &msg);
    void log_recipe           (const std::string &msg);
    void log_action_components(const std::string &msg);
    void log_second_objects   (const std::string &msg);
    void log_second_slots     (const std::string &msg);
    void log_second_locations (const std::string &msg);

    void remove_go_to    (int ind);
    void remove_location (int ind);
    void remove_place    (int ind);
    void remove_pick     (int ind);
    void remove_object   (int ind);
    void remove_slot     (int ind);
    void remove_box      (int ind);
    void active_configuration(std::string config);
    void move_gripper(std::string str);
    std::vector<int> remove_group( int ind);
    bool compare(std::vector<std::string> &v1, std::vector<std::string> &v2);

    std::vector<std::string> TFs;
    std::vector<std::string> robots;
    std::string base_frame;
    std::string target_frame;
    std::string frame_id;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

    ros::Publisher     twist_pub;
    ros::ServiceClient set_ctrl_srv;
    ros::ServiceClient gripper_srv;
    configuration_msgs::StartConfiguration start_ctrl_req;
    manipulation_msgs::JobExecution gripper_req;

    QStringListModel logging_model_go_to;
    QStringListModel logging_model_place;
    QStringListModel logging_model_pick;
    QStringListModel logging_model_object;
    QStringListModel logging_model_slot;
    QStringListModel logging_model_box;
    QStringListModel logging_model_group;
    QStringListModel logging_model_location;
    QStringListModel logging_model_object_modify;
    QStringListModel logging_model_slot_modify;
    QStringListModel logging_model_box_modify;
    QStringListModel logging_model_location_modify;
    QStringListModel logging_model_components;
    QStringListModel logging_model_info_action;
    QStringListModel logging_model_second_go_to;
    QStringListModel logging_model_second_place;
    QStringListModel logging_model_second_pick;
    QStringListModel logging_model_action_components;
    QStringListModel logging_model_recipe;

    std::vector<go_to_location> go_to_locations;
    std::vector<go_to>          go_to_actions;
    std::vector<place>          place_actions;
    std::vector<pick>           pick_actions;
    std::vector<object_type>    objects;
    std::vector<slot>           manipulation_slots;
    std::vector<std::string>    groups;
    std::vector<box>            boxes;
    std::vector<recipe>         recipes;
    std::vector<go_to_location> go_to_locations_compare;
    std::vector<go_to>          go_to_actions_compare;
    std::vector<place>          place_actions_compare;
    std::vector<pick>           pick_actions_compare;
    std::vector<object_type>    objects_compare;
    std::vector<slot>           slots_compare;
    std::vector<std::string>    groups_compare;
    std::vector<box>            boxes_compare;
    std::vector<recipe>         recipes_compare;
    std::vector<go_to>          second_go_to_actions;
    std::vector<place>          second_place_actions;
    std::vector<pick>           second_pick_actions;
    std::vector<action>         action_list;
    std::vector<std::string>    param_names;
    std::vector<std::string>    robot_name_params;

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

    bool manual_guidance = false;
};

}  // namespace manipulation_interface_gui

#endif /* manipulation_interface_gui_QNODE_HPP_ */
