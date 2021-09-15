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
#include <thread>
#include <QStringListModel>
#include <configuration_msgs/StartConfiguration.h>
#include <manipulation_msgs/JobExecution.h>
#include <manipulation_utils/manipulation_load_params_utils.h>
#include <subscription_notifier/subscription_notifier.h>
#include <sensor_msgs/JointState.h>


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

struct go_to_action
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
    std::string              type;
    std::vector<std::string> tool;
    std::vector<position>    approach;
    std::vector<location>    grasp;
    std::vector<position>    leave;
    std::vector<double>      pre_gripper_position;
    std::vector<double>      post_gripper_position;
    std::vector<double>      gripper_force;
};

struct manipulation_slot
{
    std::string name;
    std::string group;
    position    approach;
    position    leave;
    location    location_;
    int         max_objects;
    std::string frame;
    int max_gripper_position;
    int min_gripper_position;
};

struct box
{
    std::string name;
    location    location_;
    position    approach;
    position    leave;
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
    QNode(int argc, char** argv,
          ros::NodeHandle n_,
          ros::NodeHandle nh_i_,
          ros::NodeHandle nh_o_,
          ros::NodeHandle nh_g_);
	virtual ~QNode();

	bool init();

    void cartMove (std::vector<float> twist_move);

    bool saveRecipe();
    int runRecipe();
    int runSelectedAction( int index );
    std::vector<std::string> loadRecipesParam();

//    void initialAddComponentsInManipulation();

    XmlRpc::XmlRpcValue getRecipeParam(std::vector<std::string> recipe_);
    XmlRpc::XmlRpcValue getRecipeParam      (int index);
    XmlRpc::XmlRpcValue getActionGoToParam  (int index);
    XmlRpc::XmlRpcValue getActionPlaceParam (int index);
    XmlRpc::XmlRpcValue getActionPickParam  (int index);

    void addLocation (go_to_location location_to_add);
    bool addRecipe   (std::string recipe_name);
    bool addGoTo     (std::string go_to_name, std::vector<std::string> locations_, std::string description, std::vector<std::string> agents_);
    bool addPlace    (std::string place_name, std::vector<std::string> groups_, std::string description, std::vector<std::string> agents_);
    bool addPick     (std::string pick_name, std::vector<std::string> objects_, std::string description, std::vector<std::string> agents_);
    bool add_object  (object_type object_);
    bool addSlot     (manipulation_slot slot_);
    bool addBox      (box box_);
    void addObjectType         (int ind);
    void addSlotGroups         (int ind);
    void addLocationInfo       (int ind);
    bool addObjectsPick        (int ind);
    bool addGroupsPlace        (int ind);
    bool addLocationsGoTo      (int ind);
    bool addSecondLocationInfo (int ind);
    bool addSecondSlotGroups   (int ind);
    bool addSecondObjectType   (int ind);
    bool addLocationChanges    (int ind, go_to_location new_location);
    bool addSlotChanges        (int ind, manipulation_slot new_slot);
    bool addBoxChanges         (int ind, box new_box);
    bool addObjectChanges      (int ind, object_type new_object);
    bool addObjectCopyGrasp    (int index, int index2);
    bool addLocationCopy (go_to_location new_loc);
    bool addObjectCopy   (object_type new_obj);
    bool addSlotCopy     (manipulation_slot new_slot);
    bool addBoxCopy      (box new_box);

    std::vector<std::string> loadObjectsInManipulation();
    bool loadNewLocation (const go_to_location &location_name_);
    bool loadNewBox      (const box &box_to_add);
    bool loadNewGroup    (const std::string &group_to_add);
    bool loadNewSlot     (const manipulation_slot &slot_name_);

    location          returnPosition(std::string base_frame, std::string target_frame);
    go_to_action      returnGoToInfo         (int ind);
    pick              returnPickInfo         (int ind);
    place             returnPlaceInfo        (int ind);
    go_to_location    returnLocationInfo     (int ind);
    object_type       returnObjectInfo       (int ind);
    box               returnBoxInfo          (int ind);
    manipulation_slot returnSlotInfo         (int ind);
    recipe            returnRecipeInfo       (int ind);
    std::string       returnLocationListText (int ind);
    std::string       returnGroupListText    (int ind);
    std::string       returnObjectListText   (int ind);
    std::string       returnObjDistListText  (int ind);
    std::string       returnBoxListText      (int ind);
    double returnGripperPosition();

    std::string getXmlMaxNumberString        (int value);
    std::string getXmlDoubleString           (double value);
    std::string getXmlDoubleStringWithName   ( std::string param_name, double value);
    std::string getXmlStringParam            (std::string param_name, std::string value);
    std::string getXmlPositionString         (std::string name_pos,  position pos);
    std::string getXmlQuaternionString       (quaternion quat);
    std::string getXmlGroupString            (std::string name, std::vector<std::string> string_group);
    std::string getXmlObjectGraspString      (int index, int index2);
    std::string getXmlObjectGraspPosesString (int index);

    XmlRpc::XmlRpcValue getObjectGraspParam  (int index, int index2);
    XmlRpc::XmlRpcValue getObjectParam       (int index);
    XmlRpc::XmlRpcValue getGoToLocationParam (int index);
    XmlRpc::XmlRpcValue getBoxParam          (int index);
    XmlRpc::XmlRpcValue getGroupParam        (int index);
    XmlRpc::XmlRpcValue getObjectNameParam   (int index);
    XmlRpc::XmlRpcValue getSlotParam         (int index);
    XmlRpc::XmlRpcValue getGoToParam         (int index);
    XmlRpc::XmlRpcValue getPickParam         (int index);
    XmlRpc::XmlRpcValue getPlaceParam        (int index);

    void loadTF                        ();
    void loadRobots                    ();
    void writeGroups                   ();
    void writeObjects                  ();
    void writeLocations                ();
    void checkOtherParam               ();
    void checkObjectsParam             ();
    void checkRecipesParam             ();
    void writeGoToLocationServer       ();
    bool saveActions                   ();
    bool saveComponents                ();
    bool readBoxesFromParam            ();
    bool readSlotsFromParam            ();
    bool readObjectFromParam           ();
    bool readLocationsFromParam        ();
    bool readSlotsGroupFromParam       ();
    bool readGotoPickAndPlaceFromParam ();

    void setTargetFrame (int ind);
    void loadParam      (int ind);
    void writeParam     (int ind);
    void writeRecipe    (int ind);
    bool removeRecipe   (int ind);

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
    void logGoTo             (const std::string &msg);
    void logPlace            (const std::string &msg);
    void logPick             (const std::string &msg);
    void logObject           (const std::string &msg);
    void logSlot             (const std::string &msg);
    void logBox              (const std::string &msg);
    void logGroup            (const std::string &msg);
    void logLocation         (const std::string &msg);
    void logObjectModify     (const std::string &msg);
    void logSlotModify       (const std::string &msg);
    void logBoxModify        (const std::string &msg);
    void logLocationModify   (const std::string &msg);
    void logComponents       (const std::string &msg);
    void logInfoAction       (const std::string &msg);
    void logSecondGoTo       (const std::string &msg);
    void logSecondPlace      (const std::string &msg);
    void logSecondPick       (const std::string &msg);
    void logRecipe           (const std::string &msg);
    void logActionComponents (const std::string &msg);
    void logSecondObjects    (const std::string &msg);
    void logSecondSlots      (const std::string &msg);
    void logSecondLocations  (const std::string &msg);

    void removeGoTo     (int ind);
    void removeLocation (int ind);
    void removePlace    (int ind);
    void removePick     (int ind);
    void removeObject   (int ind);
    void removeSlot     (int ind);
    void removeBox      (int ind);
    void activeConfiguration(std::string config);
    void moveGripper        (std::string str);
    std::vector<int> removeGroup( int ind);
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
    std::thread t_component;

    ros::NodeHandle n;
    ros::NodeHandle nh_i;
    ros::NodeHandle nh_o;
    ros::NodeHandle nh_g;

    std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> js_sub;
    sensor_msgs::JointState gripper_state;

    ros::ServiceClient add_objs_to_scene_client_;
    ros::ServiceClient add_locations_client_;
    ros::ServiceClient add_boxes_client_;
    ros::ServiceClient add_objs_client_;
    ros::ServiceClient add_slots_group_client_;
    ros::ServiceClient add_slots_client_;
    ros::ServiceClient remove_locations_client_;
    ros::ServiceClient remove_boxes_client_;
    ros::ServiceClient remove_objs_client_;
    ros::ServiceClient remove_slots_group_client_;
    ros::ServiceClient remove_slots_client_;
    ros::ServiceClient list_objects_client_;
    ros::ServiceClient list_manipulation_objects_client_;

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

    std::vector<object_type>       changed_objects;
    std::vector<go_to_location>    changed_locations;
    std::vector<manipulation_slot> changed_slots;
    std::vector<box>               changed_boxes;
    std::vector<std::string>       changed_groups;
    std::vector<object_type>       objects_to_remove;
    std::vector<go_to_location>    locations_to_remove;
    std::vector<manipulation_slot> slots_to_remove;
    std::vector<box>               boxes_to_remove;
    std::vector<std::string>       groups_to_remove;

    std::vector<go_to_location>    go_to_locations;
    std::vector<go_to_action>      go_to_actions;
    std::vector<place>             place_actions;
    std::vector<pick>              pick_actions;
    std::vector<object_type>       objects;
    std::vector<manipulation_slot> manipulation_slots;
    std::vector<std::string>       groups;
    std::vector<box>               boxes;
    std::vector<recipe>            recipes;
    std::vector<go_to_location>    go_to_locations_compare;
    std::vector<go_to_action>      go_to_actions_compare;
    std::vector<place>             place_actions_compare;
    std::vector<pick>              pick_actions_compare;
    std::vector<object_type>       objects_compare;
    std::vector<manipulation_slot> slots_compare;
    std::vector<std::string>       groups_compare;
    std::vector<box>               boxes_compare;
    std::vector<recipe>            recipes_compare;
    std::vector<go_to_action>      second_go_to_actions;
    std::vector<place>             second_place_actions;
    std::vector<pick>              second_pick_actions;
    std::vector<action>            action_list;
    std::vector<std::string>       param_names;
    std::vector<std::string>       robot_name_params;

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


    bool tc_finito = false;
};

}  // namespace manipulation_interface_gui

#endif /* manipulation_interface_gui_QNODE_HPP_ */
