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
#include <std_msgs/String.h>
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
    std::string job_exec_name;
    std::string pre_exec_property_id;
    std::string exec_property_id;
    std::string post_exec_property_id;
};

struct place
{
    std::string name;
    std::vector<std::string> groups;
    std::string description;
    std::vector<std::string> agents;
    std::string job_exec_name;
    std::string pre_exec_property_id;
    std::string exec_property_id;
    std::string post_exec_property_id;
};

struct pick
{
    std::string name;
    std::vector<std::string> objects;
    std::string description;
    std::vector<std::string> agents;
    std::string job_exec_name;
    std::string pre_exec_property_id;
    std::string exec_property_id;
    std::string post_exec_property_id;
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
          ros::NodeHandle n_);
	virtual ~QNode();

	bool init();

    void cartMove (const std::vector<float> &twist_move);

    bool saveRecipe();
    std::string runRecipe();
    std::string runSelectedAction (const int &index );
    std::string callRunRecipe     (const std::vector<std::string> &recipe);
    std::vector<std::string> loadRecipesParam();

//    void initialAddComponentsInManipulation();

    XmlRpc::XmlRpcValue getRecipeParam      (const std::vector<std::string> &recipe);
    XmlRpc::XmlRpcValue getRecipeParam      (const std::string &name);
    XmlRpc::XmlRpcValue getActionGoToParam  (const std::string &name);
    XmlRpc::XmlRpcValue getActionPlaceParam (const std::string &name);
    XmlRpc::XmlRpcValue getActionPickParam  (const std::string &name);

    bool addGoTo               (const go_to_action &gt_action);
    bool addPlace              (const place &place_action);
    bool addPick               (const pick &pick_action);
    bool addRecipe             (const std::string &recipe_name);
    bool addLocationCopy       (const go_to_location    &new_loc);
    bool addObjectCopy         (const object_type       &new_obj);
    bool addSlotCopy           (const manipulation_slot &new_slot);
    bool addBoxCopy            (const box               &new_box);
    void addLocation           (const go_to_location &location_to_add);
    void addObject             (const object_type &object);
    void addSlot               (const manipulation_slot &slot);
    void addBox                (const box &internal_box);
    void addObjectType         (const std::string &name);
    void addSlotGroups         (const std::string &name);
    void addLocationInfo       (const std::string &name);
    void addSecondLocationInfo (const std::string &name);
    void addSecondSlotGroups   (const std::string &name);
    void addSecondObjectType   (const std::string &name);
    bool addLocationChanges    (const go_to_location &new_location);
    bool addSlotChanges        (const manipulation_slot &new_slot);
    bool addBoxChanges         (const box &new_box);
    void addObjectChanges      (const object_type &new_object);
    void addObjectCopyGrasp    (const std::string &name, const int &index);

    std::vector<std::string> loadObjectsInManipulation();

    bool loadNewLocation (const go_to_location    &location_to_add);
    bool loadNewBox      (const box               &box_to_add);
    bool loadNewGroup    (const std::string       &group_to_add);
    bool loadNewSlot     (const manipulation_slot &slot_to_add);

    location          returnPosition(const std::string &base_frame, const std::string &target_frame);
    go_to_action      returnGoToInfo         (const std::string &name);
    pick              returnPickInfo         (const std::string &name);
    place             returnPlaceInfo        (const std::string &name);
    go_to_location    returnLocationInfo     (const std::string &name);
    object_type       returnObjectInfo       (const std::string &name);
    box               returnBoxInfo          (const std::string &name);
    manipulation_slot returnSlotInfo         (const std::string &name);
    recipe            returnRecipeInfo       (const std::string &name);
    double returnGripperPosition();

    std::string getXmlMaxNumberString        (const int &value);
    std::string getXmlDoubleString           (const double &value);
    std::string getXmlDoubleStringWithName   (const std::string &param_name, const double &value);
    std::string getXmlStringParam            (const std::string &param_name, const std::string &value);
    std::string getXmlPositionString         (const std::string &name_pos,  const position &pos);
    std::string getXmlQuaternionString       (const quaternion  &quat);
    std::string getXmlGroupString            (const std::string &name, const std::vector<std::string> &string_group);
    std::string getXmlObjectGraspString      (const std::string &name, const int &index);
    std::string getXmlObjectGraspPosesString (const std::string &name);

    XmlRpc::XmlRpcValue getObjectGraspParam  (const std::string &name, const int &index);
    XmlRpc::XmlRpcValue getObjectParam       (const std::string &name);
    XmlRpc::XmlRpcValue getGoToLocationParam (const std::string &name);
    XmlRpc::XmlRpcValue getBoxParam          (const std::string &name);
    XmlRpc::XmlRpcValue getGroupParam        (const std::string &name);
    XmlRpc::XmlRpcValue getObjectNameParam   (const std::string &name);
    XmlRpc::XmlRpcValue getSlotParam         (const std::string &name);
    XmlRpc::XmlRpcValue getGoToParam         (const std::string &name);
    XmlRpc::XmlRpcValue getPickParam         (const std::string &name);
    XmlRpc::XmlRpcValue getPlaceParam        (const std::string &name);

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

    void setTargetFrame (const int &ind);
    bool loadParam      (const int &ind);
    bool writeParam     (const int &ind);
    void writeRecipe    (const std::string &name);
    bool removeRecipe   (const std::string &name);

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

    QStringListModel* loggingModelGoTo             () {return &logging_model_go_to_;}
    QStringListModel* loggingModelPlace            () {return &logging_model_place_;}
    QStringListModel* loggingModelPick             () {return &logging_model_pick_;}
    QStringListModel* loggingModelObject           () {return &logging_model_object_;}
    QStringListModel* loggingModelSlot             () {return &logging_model_slot_;}
    QStringListModel* loggingModelBox              () {return &logging_model_box_;}
    QStringListModel* loggingModelGroup            () {return &logging_model_group_;}
    QStringListModel* loggingModelLocation         () {return &logging_model_location_;}
    QStringListModel* loggingModelObjectModify     () {return &logging_model_object_modify_;}
    QStringListModel* loggingModelSlotModify       () {return &logging_model_slot_modify_;}
    QStringListModel* loggingModelBoxModify        () {return &logging_model_box_modify_;}
    QStringListModel* loggingModelLocationModify   () {return &logging_model_location_modify_;}
    QStringListModel* loggingModelComponents       () {return &logging_model_components_;}
    QStringListModel* loggingModelInfoAction       () {return &logging_model_info_action_;}
    QStringListModel* loggingModelSecondGoto       () {return &logging_model_second_go_to_;}
    QStringListModel* loggingModelSecondPlace      () {return &logging_model_second_place_;}
    QStringListModel* loggingModelSecondPick       () {return &logging_model_second_pick_;}
    QStringListModel* loggingModelRecipe           () {return &logging_model_recipe_;}
    QStringListModel* loggingModelActionComponents () {return &logging_model_action_components_;}
    QStringListModel* loggingModelJobProperties    () {return &logging_model_job_properties_;}
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

    void removeGoTo         (const std::string &name);
    void removePlace        (const std::string &name);
    void removePick         (const std::string &name);
    void removeLocation     (const std::string &name);
    void removeObject       (const std::string &name);
    void removeSlot         (const std::string &name);
    void removeBox          (const std::string &name);
    void activeConfiguration(const std::string &config);
    void moveGripper        (const std::string &str);
    std::vector<std::string> removeGroup(const std::string &name);
    bool compare( const std::vector<std::string> &v1, const std::vector<std::string> &v2);
    void readGripperPosition();

    bool getGoToJobList (std::vector<std::string> &goto_job_list);
    bool getPlaceJobList(std::vector<std::string> &place_job_list);
    bool getPickJobList (std::vector<std::string> &pick_job_list);
    bool getPreExecProp (const std::string &job_name, std::vector<std::string> &pre_exec_prop_list);
    bool getExecProp    (const std::string &job_name, std::vector<std::string> &exec_prop_list);
    bool getPostExecProp(const std::string &job_name, std::vector<std::string> &post_exec_prop_list);

    std::vector<std::string> TFs_;
    std::vector<std::string> robots_;
    std::string base_frame_;
    std::string target_frame_;
    std::string frame_id_;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
    int init_argc_;
    char** init_argv_;

    ros::NodeHandle n_;


    std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::String>> js_sub_;
    std_msgs::String gripper_pos_;

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
    ros::ServiceClient run_recipe_client_;
    ros::ServiceClient go_to_job_list_client_;
    ros::ServiceClient pick_job_list_client_;
    ros::ServiceClient place_job_list_client_;

    ros::Publisher     twist_pub_;
    ros::ServiceClient set_ctrl_srv_;
    ros::ServiceClient gripper_srv_;
    configuration_msgs::StartConfiguration start_ctrl_req_;
    manipulation_msgs::JobExecution gripper_req_;

    QStringListModel logging_model_go_to_;
    QStringListModel logging_model_place_;
    QStringListModel logging_model_pick_;
    QStringListModel logging_model_object_;
    QStringListModel logging_model_slot_;
    QStringListModel logging_model_box_;
    QStringListModel logging_model_group_;
    QStringListModel logging_model_location_;
    QStringListModel logging_model_object_modify_;
    QStringListModel logging_model_slot_modify_;
    QStringListModel logging_model_box_modify_;
    QStringListModel logging_model_location_modify_;
    QStringListModel logging_model_components_;
    QStringListModel logging_model_info_action_;
    QStringListModel logging_model_second_go_to_;
    QStringListModel logging_model_second_place_;
    QStringListModel logging_model_second_pick_;
    QStringListModel logging_model_action_components_;
    QStringListModel logging_model_recipe_;
    QStringListModel logging_model_job_properties_;

    std::map<std::string, go_to_action>      go_to_actions_;
    std::map<std::string, place>             place_actions_;
    std::map<std::string, pick>              pick_actions_;
    std::map<std::string, go_to_action>      go_to_actions_compare_;
    std::map<std::string, place>             place_actions_compare_;
    std::map<std::string, pick>              pick_actions_compare_;

    std::map<std::string, go_to_location>    go_to_locations_;
    std::map<std::string, object_type>       objects_;
    std::map<std::string, manipulation_slot> manipulation_slots_;
    std::map<std::string, std::string>       groups_;
    std::map<std::string, box>               boxes_;
    std::map<std::string, recipe>            recipes_;
    std::map<std::string, go_to_location>    go_to_locations_compare_;
    std::map<std::string, object_type>       objects_compare_;
    std::map<std::string, manipulation_slot> slots_compare_;
    std::map<std::string, std::string>       groups_compare_;
    std::map<std::string, box>               boxes_compare_;
    std::map<std::string, recipe>            recipes_compare_;

    std::vector<std::string>       param_names_;
    std::vector<std::string>       robot_name_params_;

    const std::string init_value_  = "<value>";
    const std::string end_value_   = "</value>";
    const std::string init_struct_ = "<struct>";
    const std::string end_struct_  = "</struct>";
    const std::string init_member_ = "<member>";
    const std::string end_member_  = "</member>";
    const std::string init_name_   = "<name>";
    const std::string end_name_    = "</name>";
    const std::string init_array_  = "<array>";
    const std::string end_array_   = "</array>";
    const std::string init_data_   = "<data>";
    const std::string end_data_    = "</data>";
    const std::string init_double_ = "<double>";
    const std::string end_double_  = "</double>";
    const std::string init_int_    = "<int>";
    const std::string end_int_     = "</int>";
    const std::string init_string_ = "<string>";
    const std::string end_string_  = "</string>";

    std::string grasped_object_;
};

}  // namespace manipulation_interface_gui

#endif /* manipulation_interface_gui_QNODE_HPP_ */
