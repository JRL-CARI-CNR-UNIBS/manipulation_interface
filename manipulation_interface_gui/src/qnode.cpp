/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <vector>
#include <sstream>
#include <tf/transform_listener.h>
#include "qnode.hpp"
#include <fstream>
#include <rosparam_utilities/rosparam_utilities.h>
#include <manipulation_interface_mongo/SaveParam.h>
#include <manipulation_interface_mongo/LoadParam.h>
#include <manipulation_interface_gui/RunRecipeTest.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/ListObjects.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/AddBoxes.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddSlots.h>
#include <manipulation_msgs/AddSlotsGroup.h>
#include <manipulation_msgs/RemoveLocations.h>
#include <manipulation_msgs/RemoveBoxes.h>
#include <manipulation_msgs/RemoveObjects.h>
#include <manipulation_msgs/RemoveSlots.h>
#include <manipulation_msgs/RemoveSlotsGroup.h>
#include <manipulation_msgs/ListOfJobExecuters.h>
#include <manipulation_jobs_msgs/ListOfExecuterProperties.h>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosparam_utilities/rosparam_utilities.h>

//#include <manipulation_msgs/RemoveObjectFromSlot.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ,
             ros::NodeHandle n_) :
    n_(n_)
{}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}



bool QNode::init()
{
    twist_pub_= n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);
    set_ctrl_srv_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    gripper_srv_ = n_.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");

    start_ctrl_req_.request.start_configuration = "watch";
    start_ctrl_req_.request.strictness = 1;

    set_ctrl_srv_.waitForExistence();

    if ( !set_ctrl_srv_.call(start_ctrl_req_) )
    {
        ROS_ERROR("Unable to call %s service to set controller %s",set_ctrl_srv_.getService().c_str(),start_ctrl_req_.request.start_configuration.c_str());
        return false;
    }

    if (!start_ctrl_req_.response.ok)
    {
        ROS_ERROR("Error on service %s response", set_ctrl_srv_.getService().c_str());
        return false;
    }

    ROS_INFO("Controller %s started.",start_ctrl_req_.request.start_configuration.c_str());

    loadTF();
    loadRobots();
    gripper_pos_.data = "85";

    js_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::String>>(n_,"/gripper/joint_states",10);

    add_locations_client_             = n_.serviceClient<manipulation_msgs::AddLocations>       ("/go_to_location_server/add_locations");
    add_boxes_client_                 = n_.serviceClient<manipulation_msgs::AddBoxes>           ("/inbound_pick_server/add_boxes");
    add_objs_client_                  = n_.serviceClient<manipulation_msgs::AddObjects>         ("/inbound_pick_server/add_objects");
    add_slots_group_client_           = n_.serviceClient<manipulation_msgs::AddSlotsGroup>      ("/outbound_place_server/add_slots_group");
    add_slots_client_                 = n_.serviceClient<manipulation_msgs::AddSlots>           ("/outbound_place_server/add_slots");
    remove_locations_client_          = n_.serviceClient<manipulation_msgs::RemoveLocations>    ("/go_to_location_server/remove_locations");
    remove_boxes_client_              = n_.serviceClient<manipulation_msgs::RemoveBoxes>        ("/inbound_pick_server/remove_boxes");
    remove_objs_client_               = n_.serviceClient<manipulation_msgs::RemoveObjects>      ("/inbound_pick_server/remove_objects");
    remove_slots_group_client_        = n_.serviceClient<manipulation_msgs::RemoveSlotsGroup>   ("/outbound_place_server/remove_slots_group");
    remove_slots_client_              = n_.serviceClient<manipulation_msgs::RemoveSlots>        ("/outbound_place_server/remove_slots");
    list_objects_client_              = n_.serviceClient<object_loader_msgs::ListObjects>       ("/list_objects");
    list_manipulation_objects_client_ = n_.serviceClient<manipulation_msgs::ListOfObjects>      ("/inbound_pick_server/list_objects");
    go_to_job_list_client_            = n_.serviceClient<manipulation_msgs::ListOfJobExecuters> ("/go_to_location_server/list_executers");
    pick_job_list_client_             = n_.serviceClient<manipulation_msgs::ListOfJobExecuters> ("/inbound_pick_server/list_executers");
    place_job_list_client_            = n_.serviceClient<manipulation_msgs::ListOfJobExecuters> ("/outbound_place_server/list_executers");
    run_recipe_client_                = n_.serviceClient<manipulation_interface_gui::RunRecipeTest>("run_recipe");

    ROS_INFO("Waiting for: %s server", add_locations_client_.getService().c_str());
    add_locations_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", add_locations_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", remove_locations_client_.getService().c_str());
    remove_locations_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", remove_locations_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", add_boxes_client_.getService().c_str());
    add_boxes_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", add_boxes_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", add_objs_client_.getService().c_str());
    add_objs_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", add_objs_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", remove_boxes_client_.getService().c_str());
    remove_boxes_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", remove_boxes_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", remove_objs_client_.getService().c_str());
    remove_objs_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", remove_objs_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", add_slots_group_client_.getService().c_str());
    add_slots_group_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", add_slots_group_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", add_slots_client_.getService().c_str());
    add_slots_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", add_slots_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", remove_slots_group_client_.getService().c_str());
    remove_slots_group_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", remove_slots_group_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", remove_slots_client_.getService().c_str());
    remove_slots_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", remove_slots_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", list_objects_client_.getService().c_str());
    list_objects_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", list_objects_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", list_manipulation_objects_client_.getService().c_str());
    list_manipulation_objects_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", list_manipulation_objects_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", run_recipe_client_.getService().c_str());
    run_recipe_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", run_recipe_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", go_to_job_list_client_.getService().c_str());
    go_to_job_list_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", go_to_job_list_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", pick_job_list_client_.getService().c_str());
    pick_job_list_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", pick_job_list_client_.getService().c_str());

    ROS_INFO("Waiting for: %s server", place_job_list_client_.getService().c_str());
    place_job_list_client_.waitForExistence();
    ROS_INFO("Client %s connected to server", place_job_list_client_.getService().c_str());

    return true;
}

std::vector<std::string> QNode::loadObjectsInManipulation()
{
    manipulation_msgs::ListOfObjects manipulation_object_list;

    if ( !list_manipulation_objects_client_.call( manipulation_object_list ) )
    {
        ROS_ERROR("Unable to obtain the object list by manipulation");
        std::vector<std::string> objects_list;
        return objects_list;
    }

    manipulation_msgs::RemoveObjects remove_objects_srv;
    remove_objects_srv.request.object_names = manipulation_object_list.response.object_names;

    if ( !remove_objs_client_.call(remove_objects_srv) )
    {
        ROS_ERROR("Unable to remove the objects by the manipulation");
        std::vector<std::string> objects_list;
        return objects_list;
    }


    object_loader_msgs::ListObjects objects_list;

    if ( !list_objects_client_.call( objects_list ) )
    {
        ROS_ERROR("Unable to obtain the object list by object loader");
        return objects_list.response.ids;
    }

    manipulation_msgs::AddObjects add_objects_srv;

    if ( !objects_list.response.ids.empty() )
    {
        for ( std::size_t i = 0; i < objects_list.response.ids.size(); i++ )
        {
            manipulation_msgs::Object obj;
            obj.name = objects_list.response.ids.at(i);
            obj.type = objects_list.response.types.at(i);
            std::string type_ = obj.type;
            bool presence = false;
            for ( const std::pair<std::string,object_type> &single_obj: objects_ )
            {
                if ( !obj.type.compare( single_obj.first ) )
                {
                    presence = true;
                    for ( std::size_t j = 0; j < single_obj.second.grasp.size(); j++ )
                    {
                        manipulation_msgs::Grasp grasp_;
                        grasp_.tool_name      = single_obj.second.tool.at(j);
                        grasp_.location.name  = obj.name+"/grasp_"+std::to_string(j)+"_"+single_obj.second.tool.at(j);
                        grasp_.location.frame = obj.name;

                        grasp_.location.pose.position.x    = single_obj.second.grasp.at(j).pos.origin_x;
                        grasp_.location.pose.position.y    = single_obj.second.grasp.at(j).pos.origin_y;
                        grasp_.location.pose.position.z    = single_obj.second.grasp.at(j).pos.origin_z;
                        grasp_.location.pose.orientation.w = single_obj.second.grasp.at(j).quat.rotation_w;
                        grasp_.location.pose.orientation.x = single_obj.second.grasp.at(j).quat.rotation_x;
                        grasp_.location.pose.orientation.y = single_obj.second.grasp.at(j).quat.rotation_y;
                        grasp_.location.pose.orientation.z = single_obj.second.grasp.at(j).quat.rotation_z;

                        grasp_.location.approach_relative_pose.position.x = single_obj.second.approach.at(j).origin_x;
                        grasp_.location.approach_relative_pose.position.y = single_obj.second.approach.at(j).origin_y;
                        grasp_.location.approach_relative_pose.position.z = single_obj.second.approach.at(j).origin_z;

                        grasp_.location.leave_relative_pose.position.x = single_obj.second.leave.at(j).origin_x;
                        grasp_.location.leave_relative_pose.position.y = single_obj.second.leave.at(j).origin_y;
                        grasp_.location.leave_relative_pose.position.z = single_obj.second.leave.at(j).origin_z;

                        obj.grasping_locations.push_back(grasp_);
                    }
                }
            }
            if ( !presence )
                ROS_INFO("There isn't a grasp description of %s", type_.c_str() );
            add_objects_srv.request.add_objects.push_back( obj );
        }

        if ( !add_objs_client_.call(add_objects_srv) )
            ROS_ERROR("Unable to add the objects to the manipulation");
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::Success )
            ROS_INFO("The objects are added");
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::Error )
        {
            ROS_ERROR("Error");
        }
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::BoxNotFound )
            ROS_ERROR("Box not found");
    }

    if ( !list_manipulation_objects_client_.call( manipulation_object_list ) )
    {
        ROS_ERROR("Unable to obtain the manipulation object list");
        return manipulation_object_list.response.object_names;
    }
    return manipulation_object_list.response.object_names;
}

void QNode::cartMove(const std::vector<float> &twist_move)
{
    geometry_msgs::TwistStamped twist_command;

    twist_command.header.frame_id = frame_id_;
    twist_command.twist.linear.x  = twist_move.at(0);
    twist_command.twist.linear.y  = twist_move.at(1);
    twist_command.twist.linear.z  = twist_move.at(2);
    twist_command.twist.angular.x = twist_move.at(3);
    twist_command.twist.angular.y = twist_move.at(4);
    twist_command.twist.angular.z = twist_move.at(5);

    twist_command.header.stamp=ros::Time::now();
    twist_pub_.publish(twist_command);
}

void QNode::addObjectType(const std::string &name)
{
    if ( logging_model_action_components_.rowCount() != 0 )
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());
    for ( std::size_t i = 0; i < pick_actions_.at(name).objects.size(); i++)
        logActionComponents( pick_actions_.at(name).objects.at(i) );
}

void QNode::addSlotGroups(const std::string &name)
{
    if ( logging_model_action_components_.rowCount() != 0 )
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());
    for ( std::size_t i = 0; i < place_actions_.at(name).groups.size(); i++)
        logActionComponents( place_actions_.at(name).groups.at(i) );
}

void QNode::addLocationInfo(const std::string &name)
{
    if ( logging_model_action_components_.rowCount() != 0 )
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());

    for ( std::size_t i = 0; i < go_to_actions_.at(name).locations.size(); i++)
        logActionComponents( go_to_actions_.at(name).locations.at(i) );
}

void QNode::addSecondLocationInfo(const std::string &name)
{
    if ( logging_model_info_action_.rowCount() != 0 )
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());

    for ( std::size_t i = 0; i < go_to_actions_.at(name).locations.size(); i++)
        logInfoAction( go_to_actions_.at(name).locations.at(i) );
}

void QNode::addSecondSlotGroups( const std::string &name)
{
    if ( logging_model_info_action_.rowCount() != 0 )
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());
    for ( std::size_t i = 0; i < place_actions_.at(name).groups.size(); i++)
        logInfoAction( place_actions_.at(name).groups.at(i) );
}

void QNode::addSecondObjectType(const std::string &name)
{
    if ( logging_model_info_action_.rowCount() != 0 )
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());
    for ( std::size_t i = 0; i < pick_actions_.at(name).objects.size(); i++)
        logInfoAction( pick_actions_.at(name).objects.at(i) );
}

void QNode::addObjectCopyGrasp(const std::string &name, const int &index)
{
    objects_.at(name).approach.push_back( objects_.at(name).approach.at(index) );
    objects_.at(name).grasp.push_back   ( objects_.at(name).grasp.at(index) );
    objects_.at(name).tool.push_back    ( objects_.at(name).tool.at(index) );
}

void QNode::writeRecipe(const std::string &name)
{
    for ( std::size_t i = 0; i < recipes_.at(name).recipe_.size(); i++ )
        logRecipe( recipes_.at(name).recipe_.at(i) );
}

std::vector<std::string> QNode::loadRecipesParam()
{
    XmlRpc::XmlRpcValue config;

    std::vector<std::string> recipes_names;

    if ( !n_.getParam("/multi_skills/recipes", config) )
    {
        ROS_ERROR("Unable to find /multi_skills/recipes");
        return recipes_names;
    }
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The recipes param is not an array" );
        return recipes_names;
    }
    ROS_INFO("There are %d recipes",config.size());

    for ( int i = 0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue param = config[i];
        recipe single_recipe;

        if ( !param.hasMember("name") )
            ROS_WARN("The recipe #%d has not the field 'name'", i);
        single_recipe.name = rosparam_utilities::toString(param["name"]);

        std::string what;
        std::vector<std::string> recipe_;
        if( !rosparam_utilities::getParam(param,"recipe",recipe_,what) )
        {
            ROS_WARN("The element #%d has not the field 'recipe'", i);
            continue;
        }
        single_recipe.recipe_ = recipe_;
        bool presence = false;
        for ( const std::pair<std::string,recipe> one_recipe: recipes_ )
            if ( !single_recipe.name.compare(one_recipe.first) )
                presence = true;

        if ( !presence )
        {
            recipes_.insert(std::make_pair(single_recipe.name,single_recipe));
            recipes_compare_.insert(std::make_pair(single_recipe.name,single_recipe));
            recipes_names.push_back(single_recipe.name);
        }
    }

    return recipes_names;
}

bool QNode::addRecipe(const std::string &recipe_name)
{
    recipe in_recipe;
    in_recipe.name = recipe_name;

    for ( int i = 0; i < logging_model_recipe_.rowCount(); i++ )
        in_recipe.recipe_.push_back( logging_model_recipe_.data( logging_model_recipe_.index(i) ).toString().toStdString() );

    for ( const std::pair<std::string,recipe> single_recipe: recipes_ )
    {
        if ( !recipe_name.compare( single_recipe.first ) )
            return false;
        if ( compare( in_recipe.recipe_, single_recipe.second.recipe_ ) )
            return false;
    }

    recipes_.insert(std::make_pair(in_recipe.name,in_recipe));

    return true;
}

bool QNode::removeRecipe(const std::string &name)
{
    recipes_.erase(recipes_.find(name));
    return true;
}

bool QNode::saveRecipe()
{
    XmlRpc::XmlRpcValue param;

    checkRecipesParam();

    int i = 0;
    for ( const std::pair<std::string,recipe> &single_recipe: recipes_ )
        param[i++] = getRecipeParam(single_recipe.first);
    n_.setParam("/multi_skills/recipes", param);
    param.clear();

    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::SaveParam>("save_recipe_param_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    client.waitForExistence();
    if ( !client.call(srv) )
    {
        ROS_ERROR("Unable to call %s service",client.getService().c_str());
        return false;
    }
    return true;
}

std::string QNode::runRecipe()
{
    std::vector<std::string> recipe;

    for ( int i = 0; i < logging_model_recipe_.rowCount(); i++ )
        recipe.push_back( logging_model_recipe_.data( logging_model_recipe_.index(i) ).toString().toStdString() );

//This part is a feedback only for a specific application
    bool object_grasped = false;

    for ( std::size_t i = 0; i < recipe.size(); i++ )
    {
        if ( pick_actions_.find(recipe.at(i)) != pick_actions_.end() )
        {
            if ( object_grasped )
            {
                std::string str = "Recipe contain two consecutive pick without place";
                return str;
            }
            object_grasped = true;
        }
        if ( place_actions_.find(recipe.at(i)) != place_actions_.end() )
            object_grasped = false;
    }
//--------------------------------------------------------

    return callRunRecipe(recipe);
}

std::string QNode::runSelectedAction(const int &index)
{
    std::vector<std::string> recipe;

    recipe.push_back( logging_model_recipe_.data( logging_model_recipe_.index(index) ).toString().toStdString() );

    return callRunRecipe(recipe);
}

std::string QNode::callRunRecipe(const std::vector<std::string> &recipe)
{
    XmlRpc::XmlRpcValue param;
    param = getRecipeParam( recipe );

    n_.setParam("recipe_to_run", param);

    manipulation_interface_gui::RunRecipeTest recipe_msg;
    recipe_msg.request.robot_name = "manipulator";
    recipe_msg.request.grasped_object_in = grasped_object_;

    if (run_recipe_client_.call(recipe_msg))
    {
        if ( recipe_msg.response.result < 0 )
        {
            ROS_ERROR("%s", recipe_msg.response.result_string.c_str());
            return recipe_msg.response.result_string;
        }
        else
        {
            ROS_INFO("Done");
            grasped_object_ = recipe_msg.response.grasped_object_out;

            if ( grasped_object_.empty() )
                ROS_INFO("No grasped object");
            else
                ROS_INFO("Grasped object: %s", grasped_object_.c_str() );

            return recipe_msg.response.result_string;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service run_recipe");
        return "Failed to call service run_recipe";
    }

}

XmlRpc::XmlRpcValue QNode::getRecipeParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", name));
    xml_body.append(getXmlGroupString("recipe", recipes_.at(name).recipe_));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getRecipeParam(const std::vector<std::string> &recipe)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);


    for ( std::size_t i = 0; i < recipe.size(); i++)
    {
        xml_body.append(init_value_);
        xml_body.append(init_string_);
        xml_body.append(recipe.at(i));
        xml_body.append(end_string_);
        xml_body.append(end_value_);
    }

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getActionGoToParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "goto"));
    xml_body.append(getXmlStringParam("to_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    std::vector<std::string> names;
    names.push_back(name);
    xml_body.append(getXmlGroupString("description", names));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getActionPlaceParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "place"));
    xml_body.append(getXmlStringParam("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    xml_body.append(getXmlGroupString("description", place_actions_.at(name).groups));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getActionPickParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "pick"));
    xml_body.append(getXmlStringParam("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    xml_body.append(getXmlGroupString("description", pick_actions_.at(name).objects));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

bool QNode::addGoTo(const go_to_action &gt_action)
{
    if ( logging_model_go_to_.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_go_to_.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !gt_action.name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
                return false;
            if ( !gt_action.name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
                return false;
            if ( !gt_action.name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
                return false;
        }
        for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_ )
            if ( compare( gt_action.locations, single_go_to.second.locations ))
                return false;
    }
    logGoTo(gt_action.name);
    logSecondGoTo(gt_action.name);

    go_to_actions_.insert(std::make_pair(gt_action.name,gt_action));

    return true;
}

bool QNode::addPlace(const place &place_action)
{
    if ( logging_model_place_.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_place_.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !place_action.name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
                return false;
            if ( !place_action.name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
                return false;
            if ( !place_action.name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
                return false;
        }
        for ( const std::pair<std::string,place> &single_place: place_actions_ )
            if ( compare( place_action.groups, single_place.second.groups) )
                return false;
    }
    logPlace(place_action.name);
    logSecondPlace(place_action.name);

    place_actions_.insert(std::make_pair(place_action.name,place_action));

    return true;
}

bool QNode::addPick(const pick &pick_action)
{
    if ( !pick_action.name.empty())
    {
        if ( logging_model_pick_.rowCount()!=0 )
        {
            for (int i=0; i<logging_model_pick_.rowCount(); i++)
            {
                ros::Duration(0.1);
                if ( !pick_action.name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
                    return false;
                if ( !pick_action.name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
                    return false;
                if ( !pick_action.name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
                    return false;
            }
            for ( const std::pair<std::string,pick> &single_pick: pick_actions_ )
                if ( compare( pick_action.objects, single_pick.second.objects) )
                    return false;
        }
        logPick(pick_action.name);
        logSecondPick(pick_action.name);

        pick_actions_.insert(std::make_pair(pick_action.name,pick_action));
        return true;
    }

    ROS_ERROR("Empty name");
    ros::Duration(0.1);
    return true;
}

void QNode::addLocation(const go_to_location &location_to_add)
{
    logLocation(location_to_add.name);
    logLocationModify(location_to_add.name);
    go_to_locations_.insert(std::make_pair(location_to_add.name,location_to_add));
}

bool QNode::addLocationCopy(const go_to_location &new_loc)
{
    for ( const std::pair<std::string,go_to_location> single_location: go_to_locations_ )
    {
        if ( !new_loc.name.compare( single_location.first ) )
            return false;
    }
    go_to_locations_.insert(std::make_pair(new_loc.name,new_loc));
    logLocation( new_loc.name );
    logLocationModify( new_loc.name );
    return true;
}

bool QNode::addObjectCopy(const object_type &new_obj)
{
    for ( const std::pair<std::string,object_type> single_obj: objects_ )
    {
        if ( !new_obj.type.compare( single_obj.first ) )
            return false;
    }
    objects_.insert(std::make_pair(new_obj.type,new_obj));
    logObject( new_obj.type );
    logObjectModify( new_obj.type );
    return true;
}

bool QNode::addSlotCopy(const manipulation_slot &new_slot)
{
    for ( const std::pair<std::string,manipulation_slot> &single_slot: manipulation_slots_ )
    {
        if ( !new_slot.name.compare( single_slot.first ) )
            return false;
    }
    manipulation_slots_.insert(std::make_pair(new_slot.name,new_slot));
    logSlot( new_slot.name );
    logSlotModify( new_slot.name );
    return true;
}

bool QNode::addBoxCopy(const box &new_box)
{
    for ( const std::pair<std::string,box> &single_box: boxes_ )
    {
        if ( !new_box.name.compare( single_box.first ) )
            return false;
    }
    boxes_.insert(std::make_pair(new_box.name,new_box));
    logBox( new_box.name );
    logBoxModify( new_box.name );
    return true;
}

location QNode::returnPosition(const std::string &base_frame, const std::string &target_frame)
{
    tf::TransformListener listener;
    ros::Duration(0.3).sleep();
    tf::StampedTransform transform;
    location loc;
    try
    {
        listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    loc.pos.origin_x    = transform.getOrigin().getX();
    loc.pos.origin_y    = transform.getOrigin().getY();
    loc.pos.origin_z    = transform.getOrigin().getZ();
    loc.quat.rotation_x = transform.getRotation().getX();
    loc.quat.rotation_y = transform.getRotation().getY();
    loc.quat.rotation_z = transform.getRotation().getZ();
    loc.quat.rotation_w = transform.getRotation().getW();
    return loc;
}

go_to_action QNode::returnGoToInfo(const std::string &name)
{
    return go_to_actions_.at(name);
}

pick QNode::returnPickInfo(const std::string &name)
{
    return pick_actions_.at(name);
}

place QNode::returnPlaceInfo(const std::string &name)
{
    return place_actions_.at(name);
}

go_to_location QNode::returnLocationInfo(const std::string &name)
{
    return go_to_locations_.at(name);
}

object_type QNode::returnObjectInfo(const std::string &name)
{
    return objects_.at(name);
}

box QNode::returnBoxInfo(const std::string &name)
{
    return boxes_.at(name);
}

manipulation_slot QNode::returnSlotInfo(const  std::string &name)
{
    return manipulation_slots_.at(name);
}

double QNode::returnGripperPosition()
{
    return std::stod(gripper_pos_.data);
}

void QNode::logGoTo(const std::string &msg)
{
    logging_model_go_to_.insertRows(logging_model_go_to_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_go_to_.setData(logging_model_go_to_.index(logging_model_go_to_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logPlace(const std::string &msg)
{
    logging_model_place_.insertRows(logging_model_place_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_place_.setData(logging_model_place_.index(logging_model_place_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logPick(const std::string &msg)
{
    logging_model_pick_.insertRows(logging_model_pick_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_pick_.setData(logging_model_pick_.index(logging_model_pick_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logObject(const std::string &msg)
{
    logging_model_object_.insertRows(logging_model_object_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_object_.setData(logging_model_object_.index(logging_model_object_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logObjectModify(const std::string &msg)
{
    logging_model_object_modify_.insertRows(logging_model_object_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_object_modify_.setData(logging_model_object_modify_.index(logging_model_object_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSlot(const std::string &msg)
{
    logging_model_slot_.insertRows(logging_model_slot_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot_.setData(logging_model_slot_.index(logging_model_slot_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSlotModify(const std::string &msg)
{
    logging_model_slot_modify_.insertRows(logging_model_slot_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot_modify_.setData(logging_model_slot_modify_.index(logging_model_slot_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logBox(const std::string &msg)
{
    logging_model_box_.insertRows(logging_model_box_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box_.setData(logging_model_box_.index(logging_model_box_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logBoxModify(const std::string &msg)
{
    logging_model_box_modify_.insertRows(logging_model_box_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box_modify_.setData(logging_model_box_modify_.index(logging_model_box_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logGroup(const std::string &msg)
{
    logging_model_group_.insertRows(logging_model_group_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_group_.setData(logging_model_group_.index(logging_model_group_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logLocation(const std::string &msg)
{
    logging_model_location_.insertRows(logging_model_location_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_.setData(logging_model_location_.index(logging_model_location_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logLocationModify(const std::string &msg)
{
    logging_model_location_modify_.insertRows(logging_model_location_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_modify_.setData(logging_model_location_modify_.index(logging_model_location_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logComponents(const std::string &msg)
{
    logging_model_components_.insertRows(logging_model_components_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_components_.setData(logging_model_components_.index(logging_model_components_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logInfoAction(const std::string &msg)
{
    logging_model_info_action_.insertRows(logging_model_info_action_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_info_action_.setData(logging_model_info_action_.index(logging_model_info_action_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondGoTo(const std::string &msg)
{
    logging_model_second_go_to_.insertRows(logging_model_second_go_to_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_go_to_.setData(logging_model_second_go_to_.index(logging_model_second_go_to_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondPlace(const std::string &msg)
{
    logging_model_second_place_.insertRows(logging_model_second_place_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_place_.setData(logging_model_second_place_.index(logging_model_second_place_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondPick(const std::string &msg)
{
    logging_model_second_pick_.insertRows(logging_model_second_pick_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_pick_.setData(logging_model_second_pick_.index(logging_model_second_pick_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logActionComponents(const std::string &msg)
{
    logging_model_action_components_.insertRows(logging_model_action_components_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_action_components_.setData(logging_model_action_components_.index(logging_model_action_components_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logRecipe (const std::string &msg)
{
    logging_model_recipe_.insertRows(logging_model_recipe_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_recipe_.setData(logging_model_recipe_.index(logging_model_recipe_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::removeGoTo(const std::string &name)
{
    go_to_actions_.erase(go_to_actions_.find(name));
}

void QNode::removeLocation(const std::string &name)
{
    manipulation_msgs::RemoveLocations remove_location_srv;
    if ( go_to_locations_.find(name) != go_to_locations_.end() )
    {
        ROS_WARN("Location to remove: %s", name.c_str());
        remove_location_srv.request.location_names.push_back(name);
        if ( !remove_locations_client_.call(remove_location_srv) )
        {
            ROS_ERROR("Error while calling remove locations service");
            return;
        }
        go_to_locations_.erase(go_to_locations_.find(name));
    }

}

void QNode::removePlace(const std::string &name)
{
    if ( place_actions_.find(name) != place_actions_.end() )
        place_actions_.erase(place_actions_.find(name));
}

void QNode::removePick(const std::string &name)
{
    if ( pick_actions_.find(name) != pick_actions_.end() )
        pick_actions_.erase(pick_actions_.find(name));
}

void QNode::removeObject(const std::string &name)
{
    if ( objects_.find(name) != objects_.end() )
        objects_.erase(objects_.find(name));
}

void QNode::removeSlot(const std::string &name)
{
    manipulation_msgs::RemoveSlots remove_slots_srv;

    if ( manipulation_slots_.find(name) != manipulation_slots_.end() )
    {
        ROS_WARN("Slot to remove: %s", name.c_str());
        remove_slots_srv.request.slots_names.push_back(name);
        if ( !remove_slots_client_.call(remove_slots_srv) )
        {
            ROS_ERROR("Error while calling remove slots service");
            return;
        }
        manipulation_slots_.erase(manipulation_slots_.find(name));
    }
}

void QNode::removeBox(const std::string &name)
{
    manipulation_msgs::RemoveBoxes remove_boxes_srv;
    if ( boxes_.find(name) != boxes_.end() )
    {
        ROS_WARN("Box to remove: %s", name.c_str());
        remove_boxes_srv.request.box_names.push_back(name);
        if ( !remove_boxes_client_.call(remove_boxes_srv) )
            ROS_ERROR("Error while calling remove boxes service");
        boxes_.erase(boxes_.find(name));
    }
}

std::vector<std::string> QNode::removeGroup(const std::string &name)
{
    std::vector<std::string> slot_names;
    for ( const std::pair<std::string,manipulation_slot> &single_slot: manipulation_slots_ )
    {
        if ( !single_slot.second.group.compare(name) )
            slot_names.push_back(single_slot.first);
    }

    manipulation_msgs::RemoveSlotsGroup remove_groups_srv;

    if ( groups_.find(name) != groups_.end() )
    {
        ROS_WARN("Group to remove: %s", name.c_str());
        remove_groups_srv.request.slots_group_names.push_back(name);
        if ( !remove_slots_group_client_.call(remove_groups_srv))
        {
            ROS_ERROR("Error while calling remove groups service");
            std::vector<std::string> empty;
            return empty;
        }

        for ( int i = slot_names.size()-1; i >= 0; i--)
            removeSlot(slot_names.at(i));
        groups_.erase(groups_.find(name));
    }

    return slot_names;
}

void QNode::activeConfiguration(const std::string &config)
{
    start_ctrl_req_.request.start_configuration = config;
    start_ctrl_req_.request.strictness = 1;

    set_ctrl_srv_.waitForExistence();

    if ( !set_ctrl_srv_.call(start_ctrl_req_) )
    {
        ROS_ERROR("Unable to call %s service to set controller %s",set_ctrl_srv_.getService().c_str(),start_ctrl_req_.request.start_configuration.c_str());
        return;
    }

    if (!start_ctrl_req_.response.ok)
    {
        ROS_ERROR("Error on service %s response", set_ctrl_srv_.getService().c_str());
        return;
    }
    ROS_INFO("Controller %s started.",start_ctrl_req_.request.start_configuration.c_str());
}

void QNode::moveGripper(const std::string &str )
{
    ROS_INFO("Gripper are moving");
    gripper_req_.request.skill_name = " ";
    gripper_req_.request.tool_id = "gripper_fake";
    gripper_req_.request.property_id = str;

    gripper_srv_.waitForExistence();
    if (!gripper_srv_.call(gripper_req_))
    {
        ROS_ERROR("Unable to move gripper t %s state",gripper_req_.request.property_id.c_str());
        return;
    }

    if ( js_sub_->waitForANewData() )
        gripper_pos_ = js_sub_->getData();
}

std::string QNode::getXmlMaxNumberString(const int &value)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append("max_objects");
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_int_);
    std::string str = std::to_string(value);
    xml_body.append(str);
    xml_body.append(end_int_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlDoubleString(const double &value)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_double_);
    std::string str = std::to_string(value);
    std::replace( str.begin(), str.end(), ',', '.');
    xml_body.append(str);
    xml_body.append(end_double_);
    xml_body.append(end_value_);

    return xml_body;
}

std::string QNode::getXmlDoubleStringWithName(const std::string &param_name, const double &value)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append(param_name);
    xml_body.append(end_name_);

    xml_body.append( getXmlDoubleString(value) );

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlStringParam(const std::string &param_name, const std::string &value)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append(param_name);
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(value);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlPositionString(const std::string &name_pos, const position &pos)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append(name_pos);
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);

    xml_body.append(getXmlDoubleString(pos.origin_x));
    xml_body.append(getXmlDoubleString(pos.origin_y));
    xml_body.append(getXmlDoubleString(pos.origin_z));

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlQuaternionString(const quaternion &quat)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append("quaternion");
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);

    xml_body.append(getXmlDoubleString(quat.rotation_x));
    xml_body.append(getXmlDoubleString(quat.rotation_y));
    xml_body.append(getXmlDoubleString(quat.rotation_z));
    xml_body.append(getXmlDoubleString(quat.rotation_w));

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlGroupString(const std::string &name, const std::vector<std::string> &string_group)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append(name);
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);


    for ( std::size_t i = 0; i < string_group.size(); i++)
    {
        xml_body.append(init_value_);
        xml_body.append(init_string_);
        xml_body.append(string_group.at(i));
        xml_body.append(end_string_);
        xml_body.append(end_value_);
    }

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlObjectGraspString(const std::string &name, const int &index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam         ("tool",                  objects_compare_.at(name).tool.at(index)));
    xml_body.append(getXmlPositionString      ("position",              objects_compare_.at(name).grasp.at(index).pos));
    xml_body.append(getXmlQuaternionString    (                         objects_compare_.at(name).grasp.at(index).quat));
    xml_body.append(getXmlPositionString      ("approach_distance",     objects_compare_.at(name).approach.at(index)));
    xml_body.append(getXmlPositionString      ("leave_distance",        objects_compare_.at(name).leave.at(index)));
    xml_body.append(getXmlDoubleStringWithName("pre_gripper_position",  objects_compare_.at(name).pre_gripper_position.at(index)));
    xml_body.append(getXmlDoubleStringWithName("post_gripper_position", objects_compare_.at(name).post_gripper_position.at(index)));
    xml_body.append(getXmlDoubleStringWithName("gripper_force",         objects_compare_.at(name).gripper_force.at(index)));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    return xml_body;
}

std::string QNode::getXmlObjectGraspPosesString(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append("grasp_poses");
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);

    for ( std::size_t i = 0; i < objects_compare_.at(name).grasp.size(); i++ )
        xml_body.append( getXmlObjectGraspString(name, i) );

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

XmlRpc::XmlRpcValue QNode::getGoToLocationParam(const std::string &name)
{
    //    /go_to_location
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam     ("name",     name));
    xml_body.append(getXmlStringParam     ("frame",    go_to_locations_compare_.at(name).frame));
    xml_body.append(getXmlPositionString  ("position", go_to_locations_compare_.at(name).location_.pos));
    xml_body.append(getXmlQuaternionString(            go_to_locations_compare_.at(name).location_.quat));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getBoxParam(const std::string &name)
{
    //    /inbound/boxes
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam     ("name",              name));
    xml_body.append(getXmlStringParam     ("frame",             boxes_compare_.at(name).frame));
    xml_body.append(getXmlPositionString  ("position",          boxes_compare_.at(name).location_.pos));
    xml_body.append(getXmlPositionString  ("approach_distance", boxes_compare_.at(name).approach));
    xml_body.append(getXmlPositionString  ("leave_distance",    boxes_compare_.at(name).leave));
    xml_body.append(getXmlQuaternionString(                     boxes_compare_.at(name).location_.quat));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::getObjectGraspParam(const std::string &name, const int &index)
{
    //    /nameObj/grasp_poses
    std::string xml_body = getXmlObjectGraspString( name, index );

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::getObjectParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append( getXmlStringParam("type", objects_compare_.at(name).type) );
    xml_body.append( getXmlObjectGraspPosesString(name) );

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getGroupParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", groups_compare_.at(name)));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getSlotParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam     ("name",              name));
    xml_body.append(getXmlStringParam     ("frame",             slots_compare_.at(name).frame));
    xml_body.append(getXmlStringParam     ("slots_group",       slots_compare_.at(name).group));
    xml_body.append(getXmlMaxNumberString (                     slots_compare_.at(name).max_objects));
    xml_body.append(getXmlPositionString  ("position",          slots_compare_.at(name).location_.pos));
    xml_body.append(getXmlQuaternionString(                     slots_compare_.at(name).location_.quat));
    xml_body.append(getXmlPositionString  ("approach_distance", slots_compare_.at(name).approach));
    xml_body.append(getXmlPositionString  ("leave_distance",    slots_compare_.at(name).leave));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getGoToParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", name));
    xml_body.append(getXmlStringParam("type", "goto"));
    xml_body.append(getXmlStringParam("description",           go_to_actions_compare_.at(name).description));
    xml_body.append(getXmlGroupString("agent",                 go_to_actions_compare_.at(name).agents));
    xml_body.append(getXmlGroupString("goal",                  go_to_actions_compare_.at(name).locations));
    xml_body.append(getXmlStringParam("job_exec_name",         go_to_actions_compare_.at(name).job_exec_name));
    xml_body.append(getXmlStringParam("property_pre_exec_id",  go_to_actions_compare_.at(name).pre_exec_property_id));
    xml_body.append(getXmlStringParam("property_exec_id",      go_to_actions_compare_.at(name).exec_property_id));
    xml_body.append(getXmlStringParam("property_post_exec_id", go_to_actions_compare_.at(name).post_exec_property_id));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getPickParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name",name));
    xml_body.append(getXmlStringParam("type", "pick"));
    xml_body.append(getXmlStringParam("description",           pick_actions_compare_.at(name).description));
    xml_body.append(getXmlGroupString("agent",                 pick_actions_compare_.at(name).agents));
    xml_body.append(getXmlGroupString("goal",                  pick_actions_compare_.at(name).objects));
    xml_body.append(getXmlStringParam("job_exec_name",         pick_actions_compare_.at(name).job_exec_name));
    xml_body.append(getXmlStringParam("property_pre_exec_id",  pick_actions_compare_.at(name).pre_exec_property_id));
    xml_body.append(getXmlStringParam("property_exec_id",      pick_actions_compare_.at(name).exec_property_id));
    xml_body.append(getXmlStringParam("property_post_exec_id", pick_actions_compare_.at(name).post_exec_property_id));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getPlaceParam(const std::string &name)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name",name));
    xml_body.append(getXmlStringParam("type", "place"));
    xml_body.append(getXmlStringParam("description",           place_actions_compare_.at(name).description));
    xml_body.append(getXmlGroupString("agent",                 place_actions_compare_.at(name).agents));
    xml_body.append(getXmlGroupString("goal",                  place_actions_compare_.at(name).groups));
    xml_body.append(getXmlStringParam("job_exec_name",         place_actions_compare_.at(name).job_exec_name));
    xml_body.append(getXmlStringParam("property_pre_exec_id",  place_actions_compare_.at(name).pre_exec_property_id));
    xml_body.append(getXmlStringParam("property_exec_id",      place_actions_compare_.at(name).exec_property_id));
    xml_body.append(getXmlStringParam("property_post_exec_id", place_actions_compare_.at(name).post_exec_property_id));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

void QNode::setTargetFrame(const int &ind)
{
    if ( robot_name_params_.size() != 0 )
        n_.getParam(robot_name_params_.at(ind),target_frame_);
}

void QNode::checkObjectsParam()
{
    bool presence;

    for ( const std::pair<std::string,object_type> &single_obj: objects_ )
    {
        presence = false;
        for ( const std::pair<std::string,object_type> &single_obj_compare: objects_compare_ )
            if (single_obj.first == single_obj_compare.first)
                presence = true;
        if ( presence )
            objects_compare_.at(single_obj.first) = single_obj.second;
        else
            objects_compare_.insert(single_obj);
    }

    return;
}

void QNode::checkOtherParam()
{
    bool presence;

    for ( const std::pair<std::string,go_to_location> single_location: go_to_locations_ )
    {
        presence = false;
        for ( const std::pair<std::string,go_to_location> single_location_compare: go_to_locations_compare_ )
            if ( single_location.first == single_location_compare.first )
                presence = true;
        if (presence)
            go_to_locations_compare_.at(single_location.first) = single_location.second;
        else
            go_to_locations_compare_.insert(single_location);
    }

    for ( const std::pair<std::string,manipulation_slot> single_slot: manipulation_slots_ )
    {
        presence = false;
        for ( const std::pair<std::string,manipulation_slot> single_slot_compare: slots_compare_ )
            if ( single_slot.first == single_slot_compare.first )
                presence = true;
        if ( presence )
            slots_compare_.at(single_slot.first) = single_slot.second;
        else
            slots_compare_.insert(single_slot);
    }

    for ( const std::pair<std::string,std::string> &single_group: groups_ )
    {
        presence = false;
        for ( const std::pair<std::string,std::string> &single_group_compare: groups_compare_ )
            if ( single_group.first == single_group_compare.first )
                presence = true;
        if ( presence )
            groups_compare_.at(single_group.first) = single_group.second;
        else
            groups_compare_.insert(single_group);
    }

    for ( const std::pair<std::string,box> &single_box: boxes_ )
    {
        presence = false;
        for ( const std::pair<std::string,box> &single_box_compare: boxes_compare_ )
            if ( single_box.first == single_box_compare.first )
                presence = true;
        if ( presence )
            boxes_compare_.at(single_box.first) = single_box.second;
        else
            boxes_compare_.insert(single_box);
    }

    for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_ )
    {
        presence = false;
        for ( const std::pair<std::string,go_to_action> &single_go_to_compare: go_to_actions_compare_ )
            if ( single_go_to.first == single_go_to_compare.first )
                presence = true;
        if ( presence )
            go_to_actions_compare_.at(single_go_to.first) = single_go_to.second;
        else
            go_to_actions_compare_.insert( single_go_to );
    }

    for ( const std::pair<std::string,place> &single_place: place_actions_ )
    {
        presence = false;
        for ( const std::pair<std::string,place> &single_place_compare: place_actions_compare_ )
            if ( single_place.first == single_place_compare.first )
                presence = true;
        if ( presence )
            place_actions_compare_.at(single_place.first) = single_place.second;
        else
            place_actions_compare_.insert( single_place );
    }

    for ( const std::pair<std::string,pick> &single_pick: pick_actions_ )
    {
        presence = false;
        for ( const std::pair<std::string,pick> &single_pick_compare: pick_actions_compare_ )
            if ( single_pick.first == single_pick_compare.first )
                presence = true;
        if ( presence )
            pick_actions_compare_.at(single_pick.first) = single_pick.second;
        else
            pick_actions_compare_.insert(single_pick);
    }
}

void QNode::checkRecipesParam()
{
    for ( const std::pair<std::string,recipe> &single_recipe_compare: recipes_compare_ )
    {
        bool presence = false;
        for ( const std::pair<std::string,recipe> &single_recipe: recipes_ )
            if ( single_recipe.first == single_recipe_compare.first )
                presence = true;
        if ( !presence )
            recipes_.insert( single_recipe_compare );
    }

    recipes_compare_ = recipes_;
}

bool QNode::saveComponents()
{
    XmlRpc::XmlRpcValue param;

    checkObjectsParam();
    checkOtherParam();

    int i = 0;
    for ( const std::pair<std::string,go_to_location> &single_go_to: go_to_locations_compare_ )
        param[i++] = getGoToLocationParam(single_go_to.first);
    n_.setParam("/go_to_location", param);
    param.clear();

    i = 0;
    for ( const std::pair<std::string,box> &single_box: boxes_compare_ )
        param[i++] = getBoxParam(single_box.first);
    n_.setParam("/inbound/boxes", param);
    param.clear();

    i = 0;
    for ( const std::pair<std::string,std::string> &single_group: groups_compare_ )
        param[i++] = getGroupParam(single_group.first);
    n_.setParam("/outbound/slots_group", param);
    param.clear();

    i = 0;
    for ( const std::pair<std::string,manipulation_slot> &single_slot: slots_compare_)
        param[i++] = getSlotParam(single_slot.first);
    n_.setParam("/outbound/slots", param);
    param.clear();

    i = 0;
    for ( const std::pair<std::string,object_type> &single_obj: objects_compare_ )
        param[i++] = getObjectParam(single_obj.first);
    n_.setParam("/manipulation_object_types", param);
    param.clear();

    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::SaveParam>("/save_components_params_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    if (client.call(srv))
        return true;
    else
        return false;
}

bool QNode::loadNewLocation(const go_to_location &location_to_add)
{
    if ( !location_to_add.name.empty() )
    {
        manipulation_msgs::AddLocations add_locations_srv;
        ROS_INFO("Location to add: %s", location_to_add.name.c_str());
        manipulation_msgs::Location location_;
        location_.name               = location_to_add.name;
        location_.frame              = location_to_add.frame;
        location_.pose.position.x    = location_to_add.location_.pos.origin_x;
        location_.pose.position.y    = location_to_add.location_.pos.origin_y;
        location_.pose.position.z    = location_to_add.location_.pos.origin_z;
        location_.pose.orientation.w = location_to_add.location_.quat.rotation_w;
        location_.pose.orientation.x = location_to_add.location_.quat.rotation_x;
        location_.pose.orientation.y = location_to_add.location_.quat.rotation_y;
        location_.pose.orientation.z = location_to_add.location_.quat.rotation_z;

        add_locations_srv.request.locations.push_back(location_);

        if (!add_locations_client_.call(add_locations_srv))
        {
            ROS_ERROR("Error while calling add locations service.");
            return false;
        }

        if ( add_locations_srv.response.results == manipulation_msgs::AddLocations::Response::Error )
        {
            ROS_WARN("Can't add the location to location manager");
            return false;
        }
        else
            ROS_INFO("Location %s added to location manager", location_to_add.name.c_str() );
    }
    return true;
}

bool QNode::loadNewBox(const box &box_to_add)
{
    if ( !box_to_add.name.empty() )
    {
        manipulation_msgs::AddBoxes add_boxes_srv;
        ROS_INFO("Box to add: %s", box_to_add.name.c_str());
        manipulation_msgs::Box box_;
        box_.name = box_to_add.name;
        box_.location.name = box_.name;
        box_.location.frame = box_to_add.frame;
        box_.location.name                              = box_.name;
        box_.location.frame                             = box_to_add.frame;
        box_.location.pose.position.x                   = box_to_add.location_.pos.origin_x;
        box_.location.pose.position.y                   = box_to_add.location_.pos.origin_y;
        box_.location.pose.position.z                   = box_to_add.location_.pos.origin_z;
        box_.location.pose.orientation.w                = box_to_add.location_.quat.rotation_w;
        box_.location.pose.orientation.x                = box_to_add.location_.quat.rotation_x;
        box_.location.pose.orientation.y                = box_to_add.location_.quat.rotation_y;
        box_.location.pose.orientation.z                = box_to_add.location_.quat.rotation_z;
        box_.location.approach_relative_pose.position.x = box_to_add.approach.origin_x;
        box_.location.approach_relative_pose.position.y = box_to_add.approach.origin_y;
        box_.location.approach_relative_pose.position.z = box_to_add.approach.origin_z;
        box_.location.leave_relative_pose.position.x    = box_to_add.leave.origin_x;
        box_.location.leave_relative_pose.position.y    = box_to_add.leave.origin_y;
        box_.location.leave_relative_pose.position.z    = box_to_add.leave.origin_z;

        add_boxes_srv.request.add_boxes.push_back( box_ );

        if (!add_boxes_client_.call(add_boxes_srv))
        {
            ROS_ERROR("Error while calling add boxes service.");
            return false;
        }

        if (add_boxes_srv.response.results == manipulation_msgs::AddBoxes::Response::Error)
        {
            ROS_WARN("Can't add the box %s to location manager", box_to_add.name.c_str() );
            return false;
        }
        else
            ROS_INFO("Box %s added to location manager", box_to_add.name.c_str() );

    }
    return true;
}

bool QNode::loadNewGroup(const std::string &group_to_add)
{
    if ( !group_to_add.empty() )
    {
        ROS_INFO("Group to add: %s", group_to_add.c_str());
        bool presence = false;
        for ( int i = 0; i < logging_model_group_.rowCount(); i++ )
            if ( !group_to_add.compare(logging_model_group_.data( logging_model_group_.index( i ),0 ).toString().toStdString()) )
                presence = true;
        if (!presence)
        {
            manipulation_msgs::AddSlotsGroup add_groups_srv;
            manipulation_msgs::SlotsGroup group_srv;
            group_srv.name = group_to_add;
            add_groups_srv.request.add_slots_groups.push_back(group_srv);
            if ( !add_slots_group_client_.call(add_groups_srv) )
            {
                ROS_ERROR("Error while calling add groups service");
                return false;
            }
            if ( add_groups_srv.response.results == manipulation_msgs::AddSlotsGroup::Response::Error )
            {
                ROS_WARN("Can't add the group %s to location manager", group_to_add.c_str());
                return false;
            }
            else
                ROS_INFO("Added group: %s", group_to_add.c_str());
        }
    }
    return true;
}

bool QNode::loadNewSlot(const manipulation_slot &slot_to_add)
{
    if ( !slot_to_add.name.empty() )
    {
        ROS_INFO("Slot to add: %s", slot_to_add.name.c_str() );
        manipulation_msgs::AddSlots add_slots_srv;
        std::vector<manipulation_msgs::Slot> slot_vct;
        manipulation_msgs::Slot slot_;
        slot_.name                                       = slot_to_add.name;
        slot_.slot_size                                  = slot_to_add.max_objects;
        slot_.location.name                              = slot_.name;
        slot_.location.frame                             = slot_to_add.frame;
        slot_.location.pose.position.x                   = slot_to_add.location_.pos.origin_x;
        slot_.location.pose.position.y                   = slot_to_add.location_.pos.origin_y;
        slot_.location.pose.position.z                   = slot_to_add.location_.pos.origin_z;
        slot_.location.pose.orientation.w                = slot_to_add.location_.quat.rotation_w;
        slot_.location.pose.orientation.x                = slot_to_add.location_.quat.rotation_x;
        slot_.location.pose.orientation.y                = slot_to_add.location_.quat.rotation_y;
        slot_.location.pose.orientation.z                = slot_to_add.location_.quat.rotation_z;
        slot_.location.approach_relative_pose.position.x = slot_to_add.approach.origin_x;
        slot_.location.approach_relative_pose.position.y = slot_to_add.approach.origin_y;
        slot_.location.approach_relative_pose.position.z = slot_to_add.approach.origin_z;
        slot_.location.leave_relative_pose.position.x    = slot_to_add.leave.origin_x;
        slot_.location.leave_relative_pose.position.y    = slot_to_add.leave.origin_y;
        slot_.location.leave_relative_pose.position.z    = slot_to_add.leave.origin_z;

        slot_vct.push_back(slot_);

        add_slots_srv.request.slots_group_name = slot_to_add.group;
        add_slots_srv.request.add_slots = slot_vct;

        if (!add_slots_client_.call(add_slots_srv))
        {
            ROS_ERROR("Error while calling add slots service.");
            return false;
        }

        if (add_slots_srv.response.results == manipulation_msgs::AddSlots::Response::Error)
        {
            ROS_WARN("Can't add the slot %s to location manager", slot_to_add.name.c_str() );
            return false;
        }
        else
            ROS_INFO("Added slot: %s", slot_to_add.name.c_str());
    }
    return true;
}

bool QNode::saveActions()
{

    XmlRpc::XmlRpcValue param;

    checkObjectsParam();
    checkOtherParam();

    int i = 0;
    for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_compare_)
        param[i++] = getGoToParam(single_go_to.first);
    for ( const std::pair<std::string,pick> &single_pick: pick_actions_compare_)
        param[i++] = getPickParam(single_pick.first);
    for ( const std::pair<std::string,place> &single_place: place_actions_compare_)
        param[i++] = getPlaceParam(single_place.first);
    n_.setParam("/multi_skills/tasks", param);
    param.clear();

    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::SaveParam>("/save_actions_param_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    if (client.call(srv))
        return true;
    else
        return false;
}

void QNode::addObject(const object_type &object)
{
    logObject(object.type);
    logObjectModify(object.type);
    objects_.insert(std::make_pair(object.type,object));
}

void QNode::addSlot(const manipulation_slot &slot)
{
    if ( logging_model_group_.rowCount()!=0 )
    {
        bool add = true;
        for ( int i=0; i<logging_model_group_.rowCount(); i++)
            if ( !slot.group.compare( logging_model_group_.data( logging_model_group_.index( i ),0 ).toString().toStdString() ) )
                add = false;
        if ( add )
        {
            logGroup(slot.group);
            groups_.insert(std::make_pair(slot.group,slot.group));
        }
    }
    else
    {
        logGroup(slot.group);
        groups_.insert(std::make_pair(slot.group,slot.group));
    }

    logSlot(slot.name);
    logSlotModify(slot.name);

    manipulation_slots_.insert(std::make_pair(slot.name,slot));
}

void QNode::addBox(const box &internal_box)
{
    logBox(internal_box.name);
    logBoxModify(internal_box.name);
    boxes_.insert(std::make_pair(internal_box.name,internal_box));
}

bool QNode::addLocationChanges(const go_to_location &new_location)
{
  go_to_location internal_location;
    if ( go_to_locations_.find(new_location.name) != go_to_locations_.end() )
    {
      internal_location = go_to_locations_.at(new_location.name);
      removeLocation(new_location.name);
      if ( loadNewLocation(new_location) )
      {
        go_to_locations_.insert(std::make_pair(new_location.name,new_location));
        return true;
      }
      else
      {
        loadNewLocation(internal_location);
        go_to_locations_.insert(std::make_pair(internal_location.name,internal_location));
        return false;
      }
    }
    else
    {
      if ( loadNewLocation(new_location) )
      {
        go_to_locations_.insert(std::make_pair(new_location.name, new_location));
        logLocation(new_location.name);
        logLocationModify(new_location.name);
        return true;
      }
      else
        return false;
    }
}

bool QNode::addSlotChanges(const manipulation_slot &new_slot)
{
  manipulation_slot internal_slot;
    if ( manipulation_slots_.find(new_slot.name) != manipulation_slots_.end() )
    {
      internal_slot = manipulation_slots_.at(new_slot.name);
      removeSlot(new_slot.name);
      if ( loadNewSlot(new_slot) )
      {
        manipulation_slots_.insert(std::make_pair(new_slot.name,new_slot));
        return true;
      }
      else
      {
        loadNewSlot(internal_slot);
        manipulation_slots_.insert( std::make_pair(internal_slot.name,internal_slot) );
        return false;
      }
    }
    else
    {
      if ( loadNewSlot(new_slot))
      {
        manipulation_slots_.insert(std::make_pair(new_slot.name,new_slot));
        logSlot(new_slot.name);
        logSlotModify(new_slot.name);
        return true;
      }
      else
        return false;
    }
}

bool QNode::addBoxChanges(const box &new_box)
{
  box internal_box;
    if ( boxes_.find(new_box.name) != boxes_.end() )
    {
      internal_box = boxes_.at(new_box.name);
      removeBox(new_box.name);
      if ( loadNewBox(new_box) )
      {
        boxes_.insert(std::make_pair(new_box.name,new_box));
        return true;
      }
      else
      {
        loadNewBox(internal_box);
        boxes_.insert(std::make_pair(internal_box.name,internal_box));
        return false;
      }
    }
    else
    {
      if ( loadNewBox(new_box) )
      {
        boxes_.insert(std::make_pair(new_box.name,new_box));
        logBox(new_box.name);
        logBoxModify(new_box.name);
        return true;
      }
      else
        return false;
    }
}

void QNode::addObjectChanges(const object_type &new_object)
{
    if ( objects_.find(new_object.type) != objects_.end() )
        objects_.at(new_object.type) = new_object;
    else
    {
        objects_.insert(std::make_pair(new_object.type,new_object));
        logObjectModify(new_object.type);
        logObject(new_object.type);
    }
}

void QNode::loadTF()
{
    tf::TransformListener listener;
    ros::Duration(0.3).sleep();
    listener.getFrameStrings(TFs_);
}

bool QNode::loadParam(const int &ind)
{
    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::LoadParam>("/load_param_by_mongo");
    manipulation_interface_mongo::LoadParam srv;

    if (!client.call(srv))
        return false;

    if (!n_.getParamNames(param_names_))
    {
        ROS_ERROR("Empty robot parameter");
        return false;
    }

    if ( ind == 1 )
    {
        loadRobots();

        setTargetFrame(0);

        XmlRpc::XmlRpcValue config;
        int i = 0;
        while (i < 50)
        {
            if (!n_.getParam("/inbound/boxes",config))
            {
                if ( i == 0 )
                    ROS_WARN("Waiting the param publication");
                ros::Duration(0.1).sleep();
                i++;
            }
            else
                i = 1000;
        }

        if ( i == 50 )
        {
            ROS_ERROR("Mongo doesn't work");
            ROS_ERROR("You probably need to change the Python version to mongo_connection or start mongo");
            return false;
        }

        readBoxesFromParam();
        ROS_INFO("Read boxes finish");
        readObjectFromParam();
        ROS_INFO("Read objects finish");
        readSlotsGroupFromParam();
        ROS_INFO("Read slot group finish");
        readSlotsFromParam();
        ROS_INFO("Read slots finish");
        readLocationsFromParam();
        ROS_INFO("Read locations finish");

        ROS_INFO("Loaded param by Mongo");
    }
    else if ( ind == 2 )
    {
        readGotoPickAndPlaceFromParam();
        ROS_INFO("Read actions");
    }
    return true;
}

void QNode::loadRobots()
{
    if (!n_.getParamNames(param_names_))
    {
        ROS_ERROR("Empty robot parameter");
        return;
    }

    robots_.clear();
    robot_name_params_.clear();
    for ( std::size_t i = 0; i < param_names_.size(); i++)
    {
        std::size_t found  = param_names_.at(i).find("/go_to_location_server/groups/");
        std::size_t found2 = param_names_.at(i).find("/", 26);
        std::string str = param_names_.at(i);
        if ( found != std::string::npos && found2 != std::string::npos )
        {
            robots_.push_back( str.erase( 0, found2+1 ) );
            robot_name_params_.push_back( param_names_.at(i) );
        }
    }
}

bool QNode::writeParam(const int &ind)
{
    if ( !loadParam(ind) )
        return false;

    if ( ind == 1)
    {
        for ( const std::pair<std::string,box> &single_box: boxes_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box_.rowCount(); j++)
                if ( !single_box.first.compare(logging_model_box_.data( logging_model_box_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logBox(single_box.first);
        }
        for ( const std::pair<std::string,box> &single_box: boxes_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box_modify_.rowCount(); j++)
                if ( !single_box.first.compare(logging_model_box_modify_.data( logging_model_box_modify_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logBoxModify(single_box.first);
        }
        for ( const std::pair<std::string,object_type> &single_obj: objects_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object_.rowCount(); j++)
                if ( !single_obj.first.compare(logging_model_object_.data( logging_model_object_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logObject(single_obj.first);
        }

        for ( const std::pair<std::string,object_type> &single_obj: objects_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object_modify_.rowCount(); j++)
                if ( !single_obj.first.compare(logging_model_object_modify_.data( logging_model_object_modify_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logObjectModify(single_obj.first);
        }

        for ( const std::pair<std::string,std::string> &single_group: groups_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_group_.rowCount(); j++)
                if ( !single_group.first.compare(logging_model_group_.data( logging_model_group_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logGroup( single_group.first );
        }
        for ( const std::pair<std::string,manipulation_slot> &single_slot: manipulation_slots_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot_.rowCount(); j++)
                if ( !single_slot.first.compare(logging_model_slot_.data( logging_model_slot_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logSlot(single_slot.first);
        }
        for ( const std::pair<std::string,manipulation_slot> &single_slot: manipulation_slots_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot_modify_.rowCount(); j++)
                if ( !single_slot.first.compare(logging_model_slot_modify_.data( logging_model_slot_modify_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logSlotModify(single_slot.first);
        }
        for ( const std::pair<std::string,go_to_location> &single_location: go_to_locations_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location_.rowCount(); j++)
                if ( !single_location.first.compare( logging_model_location_.data( logging_model_location_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logLocation(single_location.first);
        }
        for ( const std::pair<std::string,go_to_location> &single_location: go_to_locations_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location_modify_.rowCount(); j++)
                if ( !single_location.first.compare( logging_model_location_modify_.data( logging_model_location_modify_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logLocationModify(single_location.first);
        }
    }
    else if ( ind == 2)
    {
        for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_go_to_.rowCount(); j++)
                if ( !single_go_to.first.compare( logging_model_go_to_.data( logging_model_go_to_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logGoTo(single_go_to.first);
        }

        for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_go_to_.rowCount(); j++)
                if ( !single_go_to.first.compare( logging_model_second_go_to_.data( logging_model_second_go_to_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logSecondGoTo(single_go_to.first);
        }

        for ( const std::pair<std::string,place> &single_place: place_actions_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_place_.rowCount(); j++)
                if ( !single_place.first.compare( logging_model_place_.data( logging_model_place_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logPlace(single_place.first);
        }

        for ( const std::pair<std::string,place> &single_place: place_actions_)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_place_.rowCount(); j++)
                if ( !single_place.first.compare(logging_model_second_place_.data( logging_model_second_place_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logSecondPlace(single_place.first);
        }

        for ( const std::pair<std::string,pick> &single_pick: pick_actions_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_pick_.rowCount(); j++)
                if ( !single_pick.first.compare(logging_model_pick_.data( logging_model_pick_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logPick(single_pick.first);
        }

        for ( const std::pair<std::string,pick> &single_pick: pick_actions_ )
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_pick_.rowCount(); j++)
                if ( !single_pick.first.compare(logging_model_second_pick_.data( logging_model_second_pick_.index( j ), 0 ).toString().toStdString() ) )
                    presence = true;
            if ( !presence )
                logSecondPick(single_pick.first);
        }
    }
    return true;
}

void QNode::writeLocations()
{
    if ( logging_model_components_.rowCount() != 0 )
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    for ( const std::pair<std::string, go_to_location> &single_location: go_to_locations_)
        logComponents(single_location.first);
}

void QNode::writeObjects()
{
    if ( logging_model_components_.rowCount() != 0 )
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    for ( const std::pair<std::string,object_type> &single_obj: objects_ )
        logComponents(single_obj.first);
}

void QNode::writeGroups()
{
    if ( logging_model_components_.rowCount() != 0 )
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    for ( const std::pair<std::string,std::string> &single_gropu: groups_ )
        logComponents(single_gropu.first);
}

bool QNode::readBoxesFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n_.getParam("/inbound/boxes",config))
    {
        ROS_ERROR("Unable to find /inboud/boxes");
        return false;
    }

    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The param is not a list of boxes" );
        return false;
    }


    ROS_INFO("There are %d boxes", config.size());

    box param_box;

    for(int i=0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue param = config[i];
        if( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%d is not a struct", i);
            continue;
        }

        if( !param.hasMember("name") )
        {
            ROS_WARN("The element #%d has not the field 'name'", i);
            continue;
        }
        param_box.name = rosparam_utilities::toString(param["name"]);

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            continue;
        }
        param_box.frame = rosparam_utilities::toString(param["frame"]);

        std::string what;
        std::vector<double> pos;
        if( !rosparam_utilities::getParam(param,"position",pos,what) )
        {
            ROS_WARN("The element #%d has not the field 'position'", i);
            continue;
        }
        assert(pos.size()==3);

        param_box.location_.pos.origin_x = pos.at(0);
        param_box.location_.pos.origin_y = pos.at(1);
        param_box.location_.pos.origin_z = pos.at(2);

        std::vector<double> quat;
        if( !rosparam_utilities::getParam(param,"quaternion",quat,what) )
        {
            ROS_WARN("The element #%d has not the field 'quaternion'", i);
            continue;
        }
        assert(quat.size()==4);

        param_box.location_.quat.rotation_x = quat.at(0);
        param_box.location_.quat.rotation_y = quat.at(1);
        param_box.location_.quat.rotation_z = quat.at(2);
        param_box.location_.quat.rotation_w = quat.at(3);

        std::vector<double> approach_distance_d;
        if( !rosparam_utilities::getParam(param,"approach_distance",approach_distance_d,what) )
        {
            ROS_WARN("The box %s has not the field 'approach_distance'",param_box.name.c_str());
            return false;
        }
        assert(approach_distance_d.size() == 3);

        param_box.approach.origin_x = approach_distance_d.at(0);
        param_box.approach.origin_y = approach_distance_d.at(1);
        param_box.approach.origin_z = approach_distance_d.at(2);

        std::vector<double> leave_distance_d;
        if( !rosparam_utilities::getParam(param,"leave_distance",leave_distance_d,what) )
        {
            ROS_WARN("The box %s has not the field 'leave_distance'",param_box.name.c_str());
            return false;
        }
        assert(leave_distance_d.size() == 3);

        param_box.leave.origin_x = leave_distance_d.at(0);
        param_box.leave.origin_y = leave_distance_d.at(1);
        param_box.leave.origin_z = leave_distance_d.at(2);

        bool presence = false;
        for ( const std::pair<std::string,box> &single_box: boxes_ )
            if ( !param_box.name.compare(single_box.first) )
                presence = true;
        if ( !presence )
        {
            if ( loadNewBox(param_box) )
                boxes_.insert(std::make_pair(param_box.name,param_box));
            boxes_compare_.insert(std::make_pair(param_box.name,param_box));
        }
    }
    return true;
}

bool QNode::readObjectFromParam()
{
    XmlRpc::XmlRpcValue param;
    if ( !n_.getParam("/manipulation_object_types", param) )
    {
        ROS_ERROR("Unable to find /manipulation_object_types");
        return false;
    }

    ROS_INFO("Reading of manipulation_object_types is finish ");

    for ( int i = 0; i < param.size(); i++ )
    {
        object_type object;
        XmlRpc::XmlRpcValue single_object = param[i];

        if( single_object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The object element #%d is not a struct", i);
            continue;
        }

        if ( !single_object.hasMember("type") )
        {
            ROS_WARN("The object element #%d has not the field 'type'", i);
            continue;
        }
        object.type = rosparam_utilities::toString(single_object["type"]);

        XmlRpc::XmlRpcValue grasp_poses_;
        if ( !single_object.hasMember("grasp_poses") )
        {
            ROS_WARN("The object element #%d has not the field 'grasp_poses'", i);
            continue;
        }
        grasp_poses_ = single_object["grasp_poses"];

        if ( grasp_poses_.getType() != XmlRpc::XmlRpcValue::TypeArray )
        {
            ROS_WARN("The field grasp_poses of object element %d is not an array", i);
            continue;
        }

        for ( int j = 0; j < grasp_poses_.size(); j++ )
        {
            XmlRpc::XmlRpcValue single_grasp = grasp_poses_[j];

            if( single_grasp.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("The grasp element %d of object element #%d is not a struct", j, i);
                continue;
            }

            location loc;

            std::string what;
            std::vector<double> pos;
            if ( !rosparam_utilities::getParam(single_grasp,"position",pos, what) )
            {
                ROS_WARN("The grasp element %d of object element #%d has not the field 'position'", j, i);
                continue;
            }

            assert(pos.size()==3);

            loc.pos.origin_x = pos.at(0);
            loc.pos.origin_y = pos.at(1);
            loc.pos.origin_z = pos.at(2);

            std::vector<double> quat;
            if( !rosparam_utilities::getParam(single_grasp,"quaternion",quat,what) )
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'quaternion'", j, i);
                continue;
            }

            assert(quat.size()==4);

            loc.quat.rotation_x = quat.at(0);
            loc.quat.rotation_y = quat.at(1);
            loc.quat.rotation_z = quat.at(2);
            loc.quat.rotation_w = quat.at(3);

            object.grasp.push_back( loc );

            std::vector<double> approach_distance_d;
            if( !rosparam_utilities::getParam(single_grasp,"approach_distance",approach_distance_d,what) )
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'approach_distance'", j, i);
                return false;
            }
            assert(approach_distance_d.size() == 3);

            position approach;
            approach.origin_x = approach_distance_d.at(0);
            approach.origin_y = approach_distance_d.at(1);
            approach.origin_z = approach_distance_d.at(2);

            object.approach.push_back( approach );

            std::vector<double> leave_distance_d;
            if( !rosparam_utilities::getParam(single_grasp,"leave_distance",leave_distance_d,what) )
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'leave_distance'", j, i);
                return false;
            }
            assert(leave_distance_d.size() == 3);

            position leave;
            leave.origin_x = leave_distance_d.at(0);
            leave.origin_y = leave_distance_d.at(1);
            leave.origin_z = leave_distance_d.at(2);

            object.leave.push_back( leave );

            if ( !single_grasp.hasMember("tool"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'tool'", j, i);
                continue;
            }

            object.tool.push_back( rosparam_utilities::toString(single_grasp["tool"]) );

            if ( !single_grasp.hasMember("pre_gripper_position"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'pre_gripper_position'", j, i);
                continue;
            }

            object.pre_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["pre_gripper_position"]) );

            if ( !single_grasp.hasMember("post_gripper_position"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'post_gripper_position'", j, i);
                continue;
            }

            object.post_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["post_gripper_position"]) );

            if ( !single_grasp.hasMember("gripper_force"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'gripper_force'", j, i);
                continue;
            }

            object.gripper_force.push_back( rosparam_utilities::toDouble(single_grasp["gripper_force"]) );

        }
        bool presence = false;
        for ( const std::pair<std::string,object_type> single_obj: objects_ )
            if ( !object.type.compare(single_obj.first) )
                presence = true;

        if ( !presence )
        {
            objects_.insert(std::make_pair(object.type,object));
            objects_compare_.insert(std::make_pair(object.type,object));
        }
    }

    return true;
}

bool QNode::readSlotsGroupFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n_.getParam("/outbound/slots_group",config))
    {
        ROS_ERROR("Unable to find /outbound/slots_group");
        return false;
    }

    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The param is not a list of boxed" );
        return false;
    }
    ROS_INFO("There are %d slots group",config.size());

    for (int i=0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue slot_group_ = config[i];
        if( slot_group_.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%d is not a struct", i);
            continue;
        }

        if( !slot_group_.hasMember("name") )
        {
            ROS_WARN("The element #%d has not the field 'name'", i);
            return false;
        }
        std::string group_name = rosparam_utilities::toString(slot_group_["name"]);
        bool presence = false;
        for ( const std::pair<std::string,std::string> &single_group: groups_ )
            if ( !group_name.compare( single_group.first ) )
                presence = true;

        if ( !presence )
        {
            if ( loadNewGroup( group_name ) )
                groups_.insert(std::make_pair(group_name,group_name));
            groups_compare_.insert(std::make_pair(group_name,group_name));
        }
    }

    return true;
}

bool QNode::readSlotsFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n_.getParam("/outbound/slots",config))
    {
        ROS_ERROR("Unable to find /outbound/slots");
        return false;
    }

    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The param is not a list of boxed" );
        return false;
    }
    ROS_INFO("There are %d slots",config.size());

    for (int i=0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue param = config[i];

        manipulation_slot manip_slot;

        if( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%d is not a struct", i);
            continue;
        }

        if( !param.hasMember("name") )
        {
            ROS_WARN("The element #%d has not the field 'name'", i);
            return false;
        }
        manip_slot.name = rosparam_utilities::toString(param["name"]);

        if( !param.hasMember("slots_group") )
        {
            ROS_WARN("The element #%d has not the field 'slots_group'", i);
            return false;
        }
        manip_slot.group = rosparam_utilities::toString(param["slots_group"]);

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            return false;
        }
        manip_slot.frame = rosparam_utilities::toString(param["frame"]);

        if( !param.hasMember("max_objects") )
        {
            ROS_WARN("The element #%d has not the field 'max_objects'", i);
            return false;
        }
        manip_slot.max_objects = rosparam_utilities::toInt(param["max_objects"]);

        std::string what;
        std::vector<double> approach_distance_d;
        if( !rosparam_utilities::getParam(param,"approach_distance",approach_distance_d,what) )
        {
            ROS_WARN("Slot %s has not the field 'approach_distance'",manip_slot.name.c_str());
            return false;
        }
        assert(approach_distance_d.size()==3);
        manip_slot.approach.origin_x = approach_distance_d.at(0);
        manip_slot.approach.origin_y = approach_distance_d.at(1);
        manip_slot.approach.origin_z = approach_distance_d.at(2);

        std::vector<double> leave_distance_d;
        if( !rosparam_utilities::getParam(param,"leave_distance",leave_distance_d,what) )
        {
            ROS_WARN("Slot %s has not the field 'leave_distance'",manip_slot.name.c_str());
            return false;
        }
        assert(leave_distance_d.size()==3);
        manip_slot.leave.origin_x = leave_distance_d.at(0);
        manip_slot.leave.origin_y = leave_distance_d.at(1);
        manip_slot.leave.origin_z = leave_distance_d.at(2);

        location loc;
        std::vector<double> position;
        if( !rosparam_utilities::getParam(param,"position",position,what) )
        {
            ROS_WARN("Slot %s has not the field 'position'",manip_slot.name.c_str());
            return false;
        }
        assert(position.size()==3);

        loc.pos.origin_x = position.at(0);
        loc.pos.origin_y = position.at(1);
        loc.pos.origin_z = position.at(2);

        std::vector<double> quaternion;
        if( !rosparam_utilities::getParam(param,"quaternion",quaternion,what) )
        {
            ROS_WARN("Slot %s has not the field 'quaternion'",manip_slot.name.c_str());
            return false;
        }
        assert(quaternion.size()==4);

        loc.quat.rotation_x = quaternion.at(0);
        loc.quat.rotation_y = quaternion.at(1);
        loc.quat.rotation_z = quaternion.at(2);
        loc.quat.rotation_w = quaternion.at(3);

        manip_slot.location_ = loc;

        bool presence = false;
        for ( const std::pair<std::string,manipulation_slot> &single_slot: manipulation_slots_ )
            if ( !manip_slot.name.compare(single_slot.first) )
                presence = true;
        if ( !presence )
        {
            if ( loadNewSlot(manip_slot) )
                manipulation_slots_.insert(std::make_pair(manip_slot.name,manip_slot));
            slots_compare_.insert(std::make_pair(manip_slot.name,manip_slot));
        }
    }

    return true;
}

bool QNode::readLocationsFromParam()
{
    XmlRpc::XmlRpcValue go_to_locations_param;
    if (!n_.getParam("/go_to_location",go_to_locations_param))
    {
        ROS_ERROR("Unable to find /go_to_location");
        return false;
    }

    if (go_to_locations_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The param is not a struct of locations" );
        return false;
    }
    ROS_INFO("There are %d go_to_locations",go_to_locations_param.size());

    for (int i=0; i < go_to_locations_param.size(); i++)
    {
        go_to_location go_to;
        XmlRpc::XmlRpcValue single_location = go_to_locations_param[i];
        if( single_location.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%d is not a struct", i);
            continue;
        }

        if( !single_location.hasMember("name") )
        {
            ROS_WARN("The element #%d has not the field 'name'", i);
            return false;
        }
        go_to.name = rosparam_utilities::toString(single_location["name"]);

        if( !single_location.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            return false;
        }
        go_to.frame = rosparam_utilities::toString(single_location["frame"]);

        std::string what;
        std::vector<double> pos;
        if( !rosparam_utilities::getParam(single_location,"position",pos,what) )
        {
            ROS_WARN("Slot %s has not the field 'position'",go_to.name.c_str());
            return false;
        }
        assert(pos.size()==3);

        go_to.location_.pos.origin_x = pos.at(0);
        go_to.location_.pos.origin_y = pos.at(1);
        go_to.location_.pos.origin_z = pos.at(2);

        std::vector<double> quat;
        if( !rosparam_utilities::getParam(single_location,"quaternion",quat,what) )
        {
            ROS_WARN("Slot %s has not the field 'quaternion'",go_to.name.c_str());
            return false;
        }
        assert(quat.size()==4);

        go_to.location_.quat.rotation_x = quat.at(0);
        go_to.location_.quat.rotation_y = quat.at(1);
        go_to.location_.quat.rotation_z = quat.at(2);
        go_to.location_.quat.rotation_w = quat.at(3);

        bool presence = false;
        for (const std::pair<std::string,go_to_location>& single_go_to: go_to_locations_)
            if ( !go_to.name.compare(single_go_to.first) )
                presence = true;

        if ( !presence )
        {
            if ( loadNewLocation(go_to) )
            {
                go_to_locations_.insert(std::make_pair(go_to.name, go_to));
                go_to_locations_compare_.insert(std::make_pair(go_to.name,go_to));
            }
        }
    }
    return true;
}

bool QNode::readGotoPickAndPlaceFromParam()
{
    XmlRpc::XmlRpcValue config;

    if ( !n_.getParam("/multi_skills/tasks",config) )
    {
        ROS_ERROR("Unable to find /multi_skills/tasks");
        return false;
    }
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The tasks param is not a list of action" );
        return false;
    }
    ROS_INFO("There are %d action",config.size());

    for ( int i = 0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue param = config[i];

        if ( !param.hasMember("type") )
        {
            ROS_WARN("The action #%d has not the field 'type'", i);
            return false;
        }
        std::string param_type = rosparam_utilities::toString(param["type"]);

        if ( !param_type.compare("goto") )
        {
            go_to_action go_to_to_read;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The action #%d has not the field 'name'", i);
                return false;
            }
            go_to_to_read.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> locations_;
            if( !rosparam_utilities::getParam(param,"goal",locations_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_to_read.locations = locations_;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_to_read.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                go_to_to_read.description = " ";
            }
            else
                go_to_to_read.description = rosparam_utilities::toString(param["description"]);

            if( !param.hasMember("job_exec_name") )
            {
                ROS_WARN("The action #%d has not the field 'job_exec_name'", i);
                go_to_to_read.job_exec_name = " ";
            }
            else
                go_to_to_read.job_exec_name = rosparam_utilities::toString(param["job_exec_name"]);

            if( !param.hasMember("property_pre_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_pre_exec_id'", i);
                go_to_to_read.pre_exec_property_id = " ";
            }
            else
                go_to_to_read.pre_exec_property_id = rosparam_utilities::toString(param["property_pre_exec_id"]);

            if( !param.hasMember("property_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_exec_id'", i);
                go_to_to_read.exec_property_id = " ";
            }
            else
                go_to_to_read.exec_property_id = rosparam_utilities::toString(param["property_exec_id"]);

            if( !param.hasMember("property_post_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_post_exec_id'", i);
                go_to_to_read.post_exec_property_id = " ";
            }
            else
                go_to_to_read.post_exec_property_id = rosparam_utilities::toString(param["property_post_exec_id"]);


            bool presence = false;
            for ( const std::pair<std::string,go_to_action> &single_go_to: go_to_actions_)
                if ( !go_to_to_read.name.compare(single_go_to.first) )
                    presence = true;

            if ( !presence )
            {
                go_to_actions_.insert(std::make_pair(go_to_to_read.name,go_to_to_read));
                go_to_actions_compare_.insert(std::make_pair(go_to_to_read.name,go_to_to_read));
            }
        }

        if ( !param_type.compare("place") )
        {
            place place_to_read;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The element place #%d has not the field 'name'", i);
                return false;
            }
            place_to_read.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> groups;
            if( !rosparam_utilities::getParam(param,"goal",groups,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_to_read.groups = groups;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_to_read.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                place_to_read.description = " ";
            }
            else
                place_to_read.description = rosparam_utilities::toString(param["description"]);

            if( !param.hasMember("job_exec_name") )
            {
                ROS_WARN("The action #%d has not the field 'job_exec_name'", i);
                place_to_read.job_exec_name = " ";
            }
            else
                place_to_read.job_exec_name = rosparam_utilities::toString(param["job_exec_name"]);

            if( !param.hasMember("property_pre_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_pre_exec_id'", i);
                place_to_read.pre_exec_property_id = " ";
            }
            else
                place_to_read.pre_exec_property_id = rosparam_utilities::toString(param["property_pre_exec_id"]);

            if( !param.hasMember("property_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_exec_id'", i);
                place_to_read.exec_property_id = " ";
            }
            else
                place_to_read.exec_property_id = rosparam_utilities::toString(param["property_exec_id"]);

            if( !param.hasMember("property_post_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_post_exec_id'", i);
                place_to_read.post_exec_property_id = " ";
            }
            else
                place_to_read.post_exec_property_id = rosparam_utilities::toString(param["property_post_exec_id"]);

            bool presence = false;
            for ( const std::pair<std::string,place> &single_place: place_actions_)
                if ( !place_to_read.name.compare(single_place.first) )
                    presence = true;
            if ( !presence )
            {
                place_actions_.insert(std::make_pair(place_to_read.name,place_to_read));
                place_actions_compare_.insert(std::make_pair(place_to_read.name,place_to_read));
            }
        }

        if ( !param_type.compare("pick") )
        {
            pick pick_to_read;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The element pick #%d has not the field 'name'", i);
                return false;
            }
            pick_to_read.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> objects;
            if( !rosparam_utilities::getParam(param,"goal",objects,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_to_read.objects = objects;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_to_read.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                pick_to_read.description = " ";
            }
            else
                pick_to_read.description = rosparam_utilities::toString(param["description"]);

            if( !param.hasMember("job_exec_name") )
            {
                ROS_WARN("The action #%d has not the field 'job_exec_name'", i);
                pick_to_read.job_exec_name = " ";
            }
            else
                pick_to_read.job_exec_name = rosparam_utilities::toString(param["job_exec_name"]);

            if( !param.hasMember("property_pre_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_pre_exec_id'", i);
                pick_to_read.pre_exec_property_id = " ";
            }
            else
                pick_to_read.pre_exec_property_id = rosparam_utilities::toString(param["property_pre_exec_id"]);

            if( !param.hasMember("property_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_exec_id'", i);
                pick_to_read.exec_property_id = " ";
            }
            else
                pick_to_read.exec_property_id = rosparam_utilities::toString(param["property_exec_id"]);

            if( !param.hasMember("property_post_exec_id") )
            {
                ROS_WARN("The action #%d has not the field 'property_post_exec_id'", i);
                pick_to_read.post_exec_property_id = " ";
            }
            else
                pick_to_read.post_exec_property_id = rosparam_utilities::toString(param["property_post_exec_id"]);

            bool presence = false;
            for ( const std::pair<std::string,pick> &single_pick: pick_actions_ )
                if ( !pick_to_read.name.compare(single_pick.first) )
                    presence = true;

            if ( !presence )
            {
                pick_actions_.insert(std::make_pair(pick_to_read.name,pick_to_read));
                pick_actions_compare_.insert(std::make_pair(pick_to_read.name,pick_to_read));
            }
        }
    }

    return true;
}

bool QNode::getGoToJobList(std::vector<std::string> &goto_job_list)
{
    manipulation_msgs::ListOfJobExecuters job_list_srv;
    if ( !go_to_job_list_client_.call(job_list_srv) )
    {
        ROS_ERROR("Unable to call /go_to_location_server/list_executers service");
        goto_job_list.clear();
        return false;
    }
    else
    {
        goto_job_list = job_list_srv.response.executers;
        return true;
    }
}

bool QNode::getPickJobList(std::vector<std::string> &pick_job_list)
{
    manipulation_msgs::ListOfJobExecuters job_list_srv;
    if ( !pick_job_list_client_.call(job_list_srv) )
    {
        ROS_ERROR("Unable to call /inbound_pick_server/list_executers");
        pick_job_list.clear();
        return false;
    }
    else
    {
        pick_job_list = job_list_srv.response.executers;
        return true;
    }
}

bool QNode::getPlaceJobList(std::vector<std::string> &place_job_list)
{
    manipulation_msgs::ListOfJobExecuters job_list_srv;
    if ( !place_job_list_client_.call(job_list_srv) )
    {
        ROS_ERROR("Unable to call /outbound_place_server/list_executers");
        place_job_list.clear();
        return false;
    }
    else
    {
        place_job_list = job_list_srv.response.executers;
        return true;
    }
}

bool QNode::getPreExecProp(const std::string &job_name, std::vector<std::string> &pre_exec_prop_list)
{
    std::string service_name;
    service_name.append("/");
    service_name.append(job_name);
    service_name.append("/list_pre_executer_properties");
    ros::ServiceClient list_pre_exec_prop_client_ = n_.serviceClient<manipulation_jobs_msgs::ListOfExecuterProperties>(service_name);
    manipulation_jobs_msgs::ListOfExecuterProperties list_pre_exec_srv;
    if ( !list_pre_exec_prop_client_.call(list_pre_exec_srv) )
    {
        ROS_ERROR("Unable to call list_pre_exec_prop_server");
        pre_exec_prop_list.clear();
        return false;
    }
    else
    {
        pre_exec_prop_list = list_pre_exec_srv.response.properties;
        return true;
    }
}

bool QNode::getExecProp(const std::string &job_name, std::vector<std::string> &exec_prop_list)
{
    std::string service_name;
    service_name.append("/");
    service_name.append(job_name);
    service_name.append("/list_executer_properties");
    ros::ServiceClient list_exec_prop_client = n_.serviceClient<manipulation_jobs_msgs::ListOfExecuterProperties>(service_name);
    manipulation_jobs_msgs::ListOfExecuterProperties list_exec_srv;
    if ( !list_exec_prop_client.call(list_exec_srv) )
    {
        ROS_ERROR("Unable to call list_exec_prop_server");
        exec_prop_list.clear();
        return false;
    }
    else
    {
        exec_prop_list = list_exec_srv.response.properties;
        return true;
    }
}

bool QNode::getPostExecProp(const std::string &job_name, std::vector<std::string> &post_exec_prop_list)
{
    std::string service_name;
    service_name.append("/");
    service_name.append(job_name);
    service_name.append("/list_post_executer_properties");
    ros::ServiceClient list_post_exec_prop_client_ = n_.serviceClient<manipulation_jobs_msgs::ListOfExecuterProperties>(service_name);
    manipulation_jobs_msgs::ListOfExecuterProperties list_post_exec_srv;
    if ( !list_post_exec_prop_client_.call(list_post_exec_srv) )
    {
        ROS_ERROR("Unable to call list_post_exec_prop_server");
        post_exec_prop_list.clear();
        return false;
    }
    else
    {
        post_exec_prop_list = list_post_exec_srv.response.properties;
        return true;
    }
}

bool QNode::compare(const std::vector<std::string> &v1, const std::vector<std::string> &v2)
{
    std::vector<std::string> v1_in = v1;
    std::vector<std::string> v2_in = v2;
    std::sort(v1_in.begin(), v1_in.end());
    std::sort(v2_in.begin(), v2_in.end());
    return v1_in == v2_in;
}
}  // namespace manipulation_interface_gui
