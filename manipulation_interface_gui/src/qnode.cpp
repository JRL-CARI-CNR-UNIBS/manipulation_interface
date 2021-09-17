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
#include <manipulation_interface_gui/recipe_test_msg.h>
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

    add_locations_client_             = n_.serviceClient<manipulation_msgs::AddLocations>     ("/go_to_location_server/add_locations");
    add_boxes_client_                 = n_.serviceClient<manipulation_msgs::AddBoxes>         ("/inbound_pick_server/add_boxes");
    add_objs_client_                  = n_.serviceClient<manipulation_msgs::AddObjects>       ("/inbound_pick_server/add_objects");
    add_slots_group_client_           = n_.serviceClient<manipulation_msgs::AddSlotsGroup>    ("/outbound_place_server/add_slots_group");
    add_slots_client_                 = n_.serviceClient<manipulation_msgs::AddSlots>         ("/outbound_place_server/add_slots");
    remove_locations_client_          = n_.serviceClient<manipulation_msgs::RemoveLocations>  ("/go_to_location_server/remove_locations");
    remove_boxes_client_              = n_.serviceClient<manipulation_msgs::RemoveBoxes>      ("/inbound_pick_server/remove_boxes");
    remove_objs_client_               = n_.serviceClient<manipulation_msgs::RemoveObjects>    ("/inbound_pick_server/remove_objects");
    remove_slots_group_client_        = n_.serviceClient<manipulation_msgs::RemoveSlotsGroup> ("/outbound_place_server/remove_slots_group");
    remove_slots_client_              = n_.serviceClient<manipulation_msgs::RemoveSlots>      ("/outbound_place_server/remove_slots");
    list_objects_client_              = n_.serviceClient<object_loader_msgs::ListObjects>     ("/list_objects");
    list_manipulation_objects_client_ = n_.serviceClient<manipulation_msgs::ListOfObjects>    ("/inbound_pick_server/list_objects");
    run_recipe_client_                = n_.serviceClient<manipulation_interface_gui::recipe_test_msg>("run_recipe");

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

    return true;
}

std::vector<std::string> QNode::loadObjectsInManipulation()
{
    object_loader_msgs::ListObjects objects_list;
    manipulation_msgs::ListOfObjects manipulation_object_list;

    if ( !list_objects_client_.call( objects_list ) )
    {
        ROS_ERROR("Unable to obtain the object list");
        return manipulation_object_list.response.object_names;
    }

    manipulation_msgs::RemoveObjects remove_objects_srv;
    manipulation_msgs::AddObjects add_objects_srv;

    if ( !objects_list.response.ids.empty() )
    {
        remove_objects_srv.request.object_names = objects_list.response.ids;

        if ( !remove_objs_client_.call(remove_objects_srv) )
        {
            ROS_ERROR("Unable to remove the objects by the manipulation");
            return manipulation_object_list.response.object_names;
        }

        for ( std::size_t i = 0; i < objects_list.response.ids.size(); i++ )
        {
            manipulation_msgs::Object obj;
            obj.name = objects_list.response.ids[i];
            obj.type = objects_list.response.types[i];
            std::string type_ = obj.type;
            int index = -1;
            for ( std::size_t j = 0; j < objects_.size(); j++ )
            {
                if ( !obj.type.compare( objects_[j].type ) )
                {
                    index = j;
                }
            }

            if ( index != -1)
            {
                for ( std::size_t j = 0; j < objects_[index].grasp.size(); j++ )
                {
                    manipulation_msgs::Grasp grasp_;
                    grasp_.tool_name = objects_[index].tool[j];
                    grasp_.location.name = obj.name+"/grasp_"+std::to_string(j)+"_"+objects_[index].tool[j];
                    grasp_.location.frame = obj.name;

                    grasp_.location.pose.position.x    = objects_[index].grasp[j].pos.origin_x;
                    grasp_.location.pose.position.y    = objects_[index].grasp[j].pos.origin_y;
                    grasp_.location.pose.position.z    = objects_[index].grasp[j].pos.origin_z;
                    grasp_.location.pose.orientation.w = objects_[index].grasp[j].quat.rotation_w;
                    grasp_.location.pose.orientation.x = objects_[index].grasp[j].quat.rotation_x;
                    grasp_.location.pose.orientation.y = objects_[index].grasp[j].quat.rotation_y;
                    grasp_.location.pose.orientation.z = objects_[index].grasp[j].quat.rotation_z;

                    grasp_.location.approach_relative_pose.position.x = objects_[index].approach[j].origin_x;
                    grasp_.location.approach_relative_pose.position.y = objects_[index].approach[j].origin_y;
                    grasp_.location.approach_relative_pose.position.z = objects_[index].approach[j].origin_z;

                    grasp_.location.leave_relative_pose.position.x = objects_[index].leave[j].origin_x;
                    grasp_.location.leave_relative_pose.position.y = objects_[index].leave[j].origin_y;
                    grasp_.location.leave_relative_pose.position.z = objects_[index].leave[j].origin_z;

                    obj.grasping_locations.push_back(grasp_);
                }
            }
            else
            {
                ROS_INFO("There isn't a grasp description of %s", type_.c_str() );
            }
            ROS_FATAL_STREAM("object\n"<< obj);
            add_objects_srv.request.add_objects.push_back( obj );
        }

        if ( !add_objs_client_.call(add_objects_srv) )
        {
            ROS_ERROR("Unable to add the objects to the manipulation");
        }
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::Success )
        {
            ROS_INFO("The objects are added");
        }
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::Error )
        {
            ROS_ERROR("Error");
        }
        if ( add_objects_srv.response.results == manipulation_msgs::AddObjects::Response::BoxNotFound )
        {
            ROS_ERROR("Box not found");
        }
    }

    if ( !list_manipulation_objects_client_.call( manipulation_object_list ) )
    {
        ROS_ERROR("Unable to obtain the manipulation object list");
        return manipulation_object_list.response.object_names;
    }
    return manipulation_object_list.response.object_names;
}

void QNode::cartMove (const std::vector<float> twist_move)
{
    geometry_msgs::TwistStamped twist_command;

    twist_command.header.frame_id=frame_id_;
    twist_command.twist.linear.x=twist_move.at(0);
    twist_command.twist.linear.y=twist_move.at(1);
    twist_command.twist.linear.z=twist_move.at(2);
    twist_command.twist.angular.x=twist_move.at(3);
    twist_command.twist.angular.y=twist_move.at(4);
    twist_command.twist.angular.z=twist_move.at(5);

    twist_command.header.stamp=ros::Time::now();
    twist_pub_.publish(twist_command);
}

void QNode::addObjectType (const int ind)
{
    if ( logging_model_action_components_.rowCount() != 0 )
    {
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());
    }
    for ( std::size_t i = 0; i < pick_actions_[ind].objects.size(); i++)
    {
        logActionComponents( pick_actions_[ind].objects[i] );
    }
}

void QNode::addSlotGroups (const int ind)
{
    if ( logging_model_action_components_.rowCount() != 0 )
    {
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());
    }
    for ( std::size_t i = 0; i < place_actions_[ind].groups.size(); i++)
    {
        logActionComponents( place_actions_[ind].groups[i] );
    }
}

void QNode::addLocationInfo (const int ind)
{
    if ( logging_model_action_components_.rowCount() != 0 )
    {
        logging_model_action_components_.removeRows(0,logging_model_action_components_.rowCount());
    }
    for ( std::size_t i = 0; i < go_to_actions_[ind].locations.size(); i++)
    {
        logActionComponents( go_to_actions_[ind].locations[i] );
    }
}

void QNode::addSecondLocationInfo( const int ind )
{
    if ( logging_model_info_action_.rowCount() != 0 )
    {
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());
    }
    for ( std::size_t i = 0; i < go_to_actions_[ind].locations.size(); i++)
    {
        logInfoAction( go_to_actions_[ind].locations[i] );
    }
}

void QNode::addSecondSlotGroups  ( const int ind )
{
    if ( logging_model_info_action_.rowCount() != 0 )
    {
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());
    }
    for ( std::size_t i = 0; i < place_actions_[ind].groups.size(); i++)
    {
        logInfoAction( place_actions_[ind].groups[i] );
    }
}

void QNode::addSecondObjectType  ( const int ind )
{
    if ( logging_model_info_action_.rowCount() != 0 )
    {
        logging_model_info_action_.removeRows(0,logging_model_info_action_.rowCount());
    }
    for ( std::size_t i = 0; i < pick_actions_[ind].objects.size(); i++)
    {
        logInfoAction( pick_actions_[ind].objects[i] );
    }
}

void QNode::addObjectCopyGrasp   ( const int index, const int index2 )
{
    objects_[index].approach.push_back( objects_[index].approach[index2] );
    objects_[index].grasp.push_back   ( objects_[index].grasp[index2] );
    objects_[index].tool.push_back    ( objects_[index].tool[index2] );
}

void QNode::writeRecipe ( const int index)
{
    for ( std::size_t i = 0; i < recipes_[index].recipe_.size(); i++ )
    {
        logRecipe( recipes_[index].recipe_[i] );
    }
}

std::vector<std::string> QNode::loadRecipesParam ()
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
        {
            ROS_WARN("The recipe #%d has not the field 'name'", i);
        }
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
        for ( std::size_t j = 0; j < recipes_.size(); j++)
        {
            if ( !single_recipe.name.compare(recipes_[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            recipes_.push_back(single_recipe);
            recipes_compare_.push_back(single_recipe);
            recipes_names.push_back(single_recipe.name);
        }
    }

    return recipes_names;
}

bool QNode::addRecipe(const std::string recipe_name)
{
    recipe recipe_;
    recipe_.name = recipe_name;

    for ( int i = 0; i < logging_model_recipe_.rowCount(); i++ )
    {
        recipe_.recipe_.push_back( logging_model_recipe_.data( logging_model_recipe_.index(i) ).toString().toStdString() );
    }

    for ( std::size_t i = 0; i < recipes_.size(); i++ )
    {
        if ( !recipe_name.compare( recipes_[i].name ) )
        {
            return false;
        }
        if ( compare( recipe_.recipe_, recipes_[i].recipe_ ) )
        {
            return false;
        }
    }

    recipes_.push_back(recipe_);

    return true;
}

bool QNode::removeRecipe(const int ind)
{
    recipes_.erase(recipes_.begin()+ind);
    return true;
}

bool QNode::saveRecipe()
{
    XmlRpc::XmlRpcValue param;

    checkRecipesParam();

    for ( std::size_t i = 0; i < recipes_.size(); i++)
    {
        param[i] = getRecipeParam(i);
    }
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
    {
        recipe.push_back( logging_model_recipe_.data( logging_model_recipe_.index(i) ).toString().toStdString() );
    }

    return callRunRecipe(recipe);
}

std::string QNode::runSelectedAction(const int index)
{
    std::vector<std::string> recipe;

    recipe.push_back( logging_model_recipe_.data( logging_model_recipe_.index(index) ).toString().toStdString() );

    return callRunRecipe(recipe);
}

std::string QNode::callRunRecipe(const std::vector<std::string> recipe)
{
    XmlRpc::XmlRpcValue param;
    param = getRecipeParam( recipe );

    n_.setParam("recipe_to_run", param);

    manipulation_interface_gui::recipe_test_msg recipe_msg;
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

XmlRpc::XmlRpcValue QNode::getRecipeParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", recipes_[index].name));
    xml_body.append(getXmlGroupString("recipe", recipes_[index].recipe_));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getRecipeParam(const std::vector<std::string> recipe)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);


    for ( std::size_t i = 0; i < recipe.size(); i++)
    {
        xml_body.append(init_value_);
        xml_body.append(init_string_);
        xml_body.append(recipe[i]);
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

XmlRpc::XmlRpcValue QNode::getActionGoToParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "goto"));
    xml_body.append(getXmlStringParam("to_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    std::vector<std::string> names;
    names.push_back(go_to_actions_[index].name);
    xml_body.append(getXmlGroupString("description", names));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getActionPlaceParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "place"));
    xml_body.append(getXmlStringParam("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    xml_body.append(getXmlGroupString("description", place_actions_[index].groups));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getActionPickParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("action", "pick"));
    xml_body.append(getXmlStringParam("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(getXmlStringParam("property_exec_id", "open"));
    xml_body.append(getXmlStringParam("tool_id", "gripper_fake"));
    xml_body.append(getXmlGroupString("description", pick_actions_[index].objects));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

bool QNode::addGoTo(const std::string go_to_name, const std::vector<std::string> &locations, const std::string description, const std::vector<std::string> agents)
{
    if ( logging_model_go_to_.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_go_to_.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !go_to_name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !go_to_name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !go_to_name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
        for ( std::size_t i = 0; i < go_to_actions_.size(); i++)
        {
            if ( compare( locations, go_to_actions_[i].locations) )
            {
                return false;
            }
        }
    }
    logGoTo(go_to_name);
    logSecondGoTo(go_to_name);

    go_to_action gt;
    gt.agents      = agents;
    gt.name = go_to_name;
    gt.locations = locations;
    gt.description = description;
    go_to_actions_.push_back(gt);

    return true;
}

bool QNode::addPlace(const std::string place_name, const std::vector<std::string> &groups, const std::string description, const std::vector<std::string> agents)
{
    if ( logging_model_place_.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_place_.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !place_name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !place_name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !place_name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
        for ( std::size_t i = 0; i < place_actions_.size(); i++)
        {
            if ( compare( groups, place_actions_[i].groups) )
            {
                return false;
            }
        }
    }
    logPlace(place_name);
    logSecondPlace(place_name);

    place plc;
    plc.agents      = agents;
    plc.name        = place_name;
    plc.groups      = groups;
    plc.description = description;
    place_actions_.push_back(plc);

    return true;
}

bool QNode::addPick(const std::string pick_name, const std::vector<std::string> &objects,const  std::string description, const std::vector<std::string> agents)
{
    if ( !pick_name.empty())
    {
        if ( logging_model_pick_.rowCount()!=0 )
        {
            for (int i=0; i<logging_model_pick_.rowCount(); i++)
            {
                ros::Duration(0.1);
                if ( !pick_name.compare( logging_model_go_to_.data( logging_model_go_to_.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
                if ( !pick_name.compare( logging_model_place_.data( logging_model_place_.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
                if ( !pick_name.compare( logging_model_pick_.data( logging_model_pick_.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
            }
            for ( std::size_t i = 0; i < pick_actions_.size(); i++)
            {
                if ( compare( objects, pick_actions_[i].objects) )
                {
                    return false;
                }
            }
        }
        logPick(pick_name);
        logSecondPick(pick_name);

        pick pck;
        pck.agents      = agents;
        pck.name        = pick_name;
        pck.objects     = objects;
        pck.description = description;
        pick_actions_.push_back(pck);
        return true;
    }

    ROS_ERROR("Empty name");
    ros::Duration(0.1);
    return true;
}

void QNode::addLocation(const go_to_location location_to_add)
{
    logLocation(location_to_add.name);
    logLocationModify(location_to_add.name);
    go_to_locations_.push_back(location_to_add);
    changed_locations_.push_back(location_to_add);
}

bool QNode::addLocationCopy(const go_to_location new_loc)
{
    for ( std::size_t i = 0; i < go_to_locations_.size(); i++)
    {
        if ( !new_loc.name.compare( go_to_locations_[i].name ) )
        {
            return false;
        }
    }
    go_to_locations_.push_back( new_loc );
    changed_locations_.push_back( new_loc );
    logLocation( new_loc.name );
    logLocationModify( new_loc.name );
    return true;
}

bool QNode::addObjectCopy(const object_type new_obj)
{
    for ( std::size_t i = 0; i < objects_.size(); i++)
    {
        if ( !new_obj.type.compare( objects_[i].type ) )
        {
            return false;
        }
    }
    objects_.push_back( new_obj );
    logObject( new_obj.type );
    logObjectModify( new_obj.type );
    return true;
}

bool QNode::addSlotCopy(const manipulation_slot new_slot)
{
    for ( std::size_t i = 0; i < manipulation_slots_.size(); i++)
    {
        if ( !new_slot.name.compare( manipulation_slots_[i].name ) )
        {
            return false;
        }
    }
    manipulation_slots_.push_back( new_slot );
    changed_slots_.push_back( new_slot );
    logSlot( new_slot.name );
    logSlotModify( new_slot.name );
    return true;
}

bool QNode::addBoxCopy(const box new_box)
{
    for ( std::size_t i = 0; i < boxes_.size(); i++)
    {
        if ( !new_box.name.compare( boxes_[i].name ) )
        {
            return false;
        }
    }
    boxes_.push_back( new_box );
    changed_boxes_.push_back( new_box );
    logBox( new_box.name );
    logBoxModify( new_box.name );
    return true;
}

location QNode::returnPosition( const std::string base_frame, const std::string target_frame )
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
    loc.pos.origin_x = transform.getOrigin().getX();
    loc.pos.origin_y = transform.getOrigin().getY();
    loc.pos.origin_z = transform.getOrigin().getZ();
    loc.quat.rotation_x = transform.getRotation().getX();
    loc.quat.rotation_y = transform.getRotation().getY();
    loc.quat.rotation_z = transform.getRotation().getZ();
    loc.quat.rotation_w = transform.getRotation().getW();
    return loc;
}

go_to_action QNode::returnGoToInfo(const int ind)
{
    return go_to_actions_[ind];
}

pick QNode::returnPickInfo(const int ind)
{
    return pick_actions_[ind];
}

place QNode::returnPlaceInfo(const int ind)
{
    return place_actions_[ind];
}

go_to_location QNode::returnLocationInfo(const int ind)
{
    return go_to_locations_[ind];
}

object_type QNode::returnObjectInfo( const int ind)
{
    return objects_[ind];
}

box QNode::returnBoxInfo( const int ind)
{
    return boxes_[ind];
}

manipulation_slot QNode::returnSlotInfo(const  int ind)
{
    return manipulation_slots_[ind];
}

std::string QNode::returnLocationListText(const int ind)
{
    return logging_model_components_.data( logging_model_components_.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::returnGroupListText(const int ind)
{
    return logging_model_components_.data( logging_model_components_.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::returnObjectListText(const int ind)
{
    return logging_model_components_.data( logging_model_components_.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::returnBoxListText(const int ind)
{
    return logging_model_box_.data( logging_model_box_.index( ind ), 0 ).toString().toStdString();
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

void QNode::logSlot( const std::string &msg)
{
    logging_model_slot_.insertRows(logging_model_slot_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot_.setData(logging_model_slot_.index(logging_model_slot_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSlotModify( const std::string &msg)
{
    logging_model_slot_modify_.insertRows(logging_model_slot_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot_modify_.setData(logging_model_slot_modify_.index(logging_model_slot_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logBox( const std::string &msg)
{
    logging_model_box_.insertRows(logging_model_box_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box_.setData(logging_model_box_.index(logging_model_box_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logBoxModify( const std::string &msg)
{
    logging_model_box_modify_.insertRows(logging_model_box_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box_modify_.setData(logging_model_box_modify_.index(logging_model_box_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logGroup( const std::string &msg)
{
    logging_model_group_.insertRows(logging_model_group_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_group_.setData(logging_model_group_.index(logging_model_group_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logLocation  ( const std::string &msg)
{
    logging_model_location_.insertRows(logging_model_location_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_.setData(logging_model_location_.index(logging_model_location_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logLocationModify  ( const std::string &msg)
{
    logging_model_location_modify_.insertRows(logging_model_location_modify_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_modify_.setData(logging_model_location_modify_.index(logging_model_location_modify_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logComponents( const std::string &msg)
{
    logging_model_components_.insertRows(logging_model_components_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_components_.setData(logging_model_components_.index(logging_model_components_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logInfoAction( const std::string &msg)
{
    logging_model_info_action_.insertRows(logging_model_info_action_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_info_action_.setData(logging_model_info_action_.index(logging_model_info_action_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondGoTo( const std::string &msg)
{
    logging_model_second_go_to_.insertRows(logging_model_second_go_to_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_go_to_.setData(logging_model_second_go_to_.index(logging_model_second_go_to_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondPlace (const std::string &msg)
{
    logging_model_second_place_.insertRows(logging_model_second_place_.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_place_.setData(logging_model_second_place_.index(logging_model_second_place_.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::logSecondPick  (const std::string &msg)
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

void QNode::removeGoTo(const int ind)
{
    go_to_actions_.erase(go_to_actions_.begin()+ind);
}

void QNode::removeLocation(const int ind)
{
    for ( std::size_t i = 0; i < changed_locations_.size(); i++ )
    {
        if ( !go_to_locations_[ind].name.compare( changed_locations_[i].name ) )
        {
            changed_locations_.erase( changed_locations_.begin() + i );
        }
    }

    manipulation_msgs::RemoveLocations remove_location_srv;
    if ( !go_to_locations_[ind].name.empty() )
    {
        ROS_WARN("Locations to remove: %s", go_to_locations_[ind].name.c_str());
        remove_location_srv.request.location_names.push_back(go_to_locations_[ind].name);
        if ( !remove_locations_client_.call(remove_location_srv) )
        {
            ROS_ERROR("Error while calling remove locations service");
        }

    }

    go_to_locations_.erase(go_to_locations_.begin()+ind);
}

void QNode::removePlace(const int ind)
{
    place_actions_.erase(place_actions_.begin()+ind);
}

void QNode::removePick(const int ind)
{
    pick_actions_.erase(pick_actions_.begin()+ind);
}

void QNode::removeObject(const int ind)
{
    objects_.erase(objects_.begin()+ind);
}

void QNode::removeSlot(const int ind)
{
    for ( std::size_t i = 0; i < changed_slots_.size(); i++ )
    {
        if ( !manipulation_slots_[ind].name.compare( changed_slots_[i].name ) )
        {
            changed_slots_.erase( changed_slots_.begin() + i );
        }
    }

    manipulation_msgs::RemoveSlots remove_slots_srv;

    if ( !manipulation_slots_[ind].name.empty() )
    {
        ROS_WARN("Slot to remove: %s", manipulation_slots_[ind].name.c_str());
        remove_slots_srv.request.slots_names.push_back(manipulation_slots_[ind].name);
        if ( !remove_slots_client_.call(remove_slots_srv) )
        {
            ROS_ERROR("Error while calling remove slots service");
        }
    }

    manipulation_slots_.erase(manipulation_slots_.begin()+ind);
}

void QNode::removeBox(const int ind)
{
    for ( std::size_t i = 0; i < changed_boxes_.size(); i++ )
    {
        if ( !boxes_[ind].name.compare( changed_boxes_[i].name ) )
        {
            changed_boxes_.erase( changed_boxes_.begin() + i );
        }
    }

    manipulation_msgs::RemoveBoxes remove_boxes_srv;
    if ( !boxes_[ind].name.empty() )
    {
        ROS_WARN("Box to remove: %s", boxes_[ind].name.c_str());
        remove_boxes_srv.request.box_names.push_back(boxes_[ind].name);
        if ( !remove_boxes_client_.call(remove_boxes_srv) )
        {
            ROS_ERROR("Error while calling remove boxes service");
        }
    }

    boxes_.erase(boxes_.begin()+ind);
}

std::vector<int> QNode::removeGroup(const int ind)
{
    for ( std::size_t i = 0; i < changed_groups_.size(); i++ )
    {
        if ( !groups_.at(ind).compare( changed_groups_.at(i) ) )
        {
            changed_groups_.erase( changed_groups_.begin() + i );
        }
    }

    std::vector<int> indexes;
    for ( std::size_t i = 0; i < manipulation_slots_.size(); i++)
    {
        if ( ! manipulation_slots_[i].group.compare( groups_.at(ind) ) )
        {
            indexes.push_back(i);
        }
    }
    for ( int i = indexes.size()-1; i >= 0; i--)
    {
        removeSlot(indexes[i]);
    }

    manipulation_msgs::RemoveSlotsGroup remove_groups_srv;

    if ( !groups_.at(ind).empty() )
    {
        ROS_WARN("Group to remove: %s", groups_.at(ind).c_str());
        remove_groups_srv.request.slots_group_names.push_back(groups_.at(ind));
        if ( !remove_slots_group_client_.call(remove_groups_srv))
        {
            ROS_ERROR("Error while calling remove groups service");
        }
    }

    groups_.erase(groups_.begin()+ind);
    return indexes;
}

void QNode::activeConfiguration(const std::string config)
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

void QNode::moveGripper( const std::string str )
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
    {
      gripper_pos_ = js_sub_->getData();
    }
}

std::string QNode::getXmlMaxNumberString(const int value )
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

std::string QNode::getXmlDoubleString( const double value )
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

std::string QNode::getXmlDoubleStringWithName(const std::string param_name, const double value)
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

std::string QNode::getXmlStringParam( const std::string param_name, const std::string value )
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

std::string QNode::getXmlPositionString( const std::string name_pos, const position pos )
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

std::string QNode::getXmlQuaternionString( const quaternion quat )
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

std::string QNode::getXmlGroupString( const std::string name, const std::vector<std::string> string_group )
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
        xml_body.append(string_group[i]);
        xml_body.append(end_string_);
        xml_body.append(end_value_);
    }

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

std::string QNode::getXmlObjectGraspString( const int index, const int index2 )
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("tool", objects_compare_[index].tool[index2]));
    xml_body.append(getXmlPositionString("position", objects_compare_[index].grasp[index2].pos));
    xml_body.append(getXmlQuaternionString(objects_compare_[index].grasp[index2].quat));
    xml_body.append(getXmlPositionString("approach_distance", objects_compare_[index].approach[index2]));
    xml_body.append(getXmlPositionString("leave_distance", objects_compare_[index].leave[index2]));
    xml_body.append(getXmlDoubleStringWithName("pre_gripper_position", objects_compare_[index].pre_gripper_position[index2]));
    xml_body.append(getXmlDoubleStringWithName("post_gripper_position", objects_compare_[index].post_gripper_position[index2]));
    xml_body.append(getXmlDoubleStringWithName("gripper_force", objects_compare_[index].gripper_force[index2]));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    return xml_body;
}

std::string QNode::getXmlObjectGraspPosesString( const int index )
{
    std::string xml_body;

    xml_body.append(init_member_);

    xml_body.append(init_name_);
    xml_body.append("grasp_poses");
    xml_body.append(end_name_);

    xml_body.append(init_value_);
    xml_body.append(init_array_);
    xml_body.append(init_data_);

    for ( std::size_t i = 0; i < objects_compare_[index].grasp.size(); i++ )
    {
        xml_body.append( getXmlObjectGraspString(index, i) );
    }

    xml_body.append(end_data_);
    xml_body.append(end_array_);
    xml_body.append(end_value_);

    xml_body.append(end_member_);

    return xml_body;
}

XmlRpc::XmlRpcValue QNode::getGoToLocationParam(const int index)
{
    //    /go_to_location
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", go_to_locations_compare_[index].name));
    xml_body.append(getXmlStringParam("frame", go_to_locations_compare_[index].frame));
    xml_body.append(getXmlPositionString("position", go_to_locations_compare_[index].location_.pos));
    xml_body.append(getXmlQuaternionString(go_to_locations_compare_[index].location_.quat));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getBoxParam(const int index)
{
    //    /inbound/boxes
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", boxes_compare_[index].name));
    xml_body.append(getXmlStringParam("frame", boxes_compare_[index].frame));
    xml_body.append(getXmlPositionString("position", boxes_compare_[index].location_.pos));
    xml_body.append(getXmlPositionString("approach_distance", boxes_compare_[index].approach));
    xml_body.append(getXmlPositionString("leave_distance", boxes_compare_[index].leave));
    xml_body.append(getXmlQuaternionString(boxes_compare_[index].location_.quat));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::getObjectGraspParam(const int index, const int index2)
{
    //    /nameObj/grasp_poses
    std::string xml_body = getXmlObjectGraspString( index, index2 );

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::getObjectParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append( getXmlStringParam("type", objects_compare_[index].type) );
    xml_body.append( getXmlObjectGraspPosesString(index) );

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getGroupParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", groups_compare_[index]));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getSlotParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", slots_compare_[index].name));
    xml_body.append(getXmlStringParam("frame", slots_compare_[index].frame));
    xml_body.append(getXmlStringParam("slots_group", slots_compare_[index].group));
    xml_body.append(getXmlMaxNumberString(slots_compare_[index].max_objects));
    xml_body.append(getXmlPositionString("position", slots_compare_[index].location_.pos));
    xml_body.append(getXmlQuaternionString( slots_compare_[index].location_.quat));
    xml_body.append(getXmlPositionString("approach_distance", slots_compare_[index].approach));
    xml_body.append(getXmlPositionString("leave_distance", slots_compare_[index].leave));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getGoToParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", go_to_actions_compare_[index].name));
    xml_body.append(getXmlStringParam("description", go_to_actions_compare_[index].description));
    xml_body.append(getXmlStringParam("type", "goto"));
    xml_body.append(getXmlGroupString("agent", go_to_actions_compare_[index].agents));
    xml_body.append(getXmlGroupString("goal", go_to_actions_compare_[index].locations));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getPickParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", pick_actions_compare_[index].name));
    xml_body.append(getXmlStringParam("description", pick_actions_compare_[index].description));
    xml_body.append(getXmlStringParam("type", "pick"));
    xml_body.append(getXmlGroupString("agent", pick_actions_compare_[index].agents));
    xml_body.append(getXmlGroupString("goal", pick_actions_compare_[index].objects));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::getPlaceParam(const int index)
{
    std::string xml_body;

    xml_body.append(init_value_);
    xml_body.append(init_struct_);

    xml_body.append(getXmlStringParam("name", place_actions_compare_[index].name));
    xml_body.append(getXmlStringParam("description", place_actions_compare_[index].description));
    xml_body.append(getXmlStringParam("type", "place"));
    xml_body.append(getXmlGroupString("agent", place_actions_compare_[index].agents));
    xml_body.append(getXmlGroupString("goal", place_actions_compare_[index].groups));

    xml_body.append(end_struct_);
    xml_body.append(end_value_);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

void QNode::setTargetFrame( const int ind )
{
    if ( robot_name_params_.size() != 0 )
    {
        n_.getParam(robot_name_params_[ind],target_frame_);
    }
}

void QNode::checkObjectsParam()
{

    bool presence;

    for ( std::size_t i = 0; i < objects_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < objects_compare_.size(); j++ )
        {
            if ( objects_[i].type == objects_compare_[j].type )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            objects_compare_[index] = objects_[i];
        }
        else
        {
            objects_compare_.push_back( objects_[i] );
        }
    }

    return;
}

void QNode::checkOtherParam()
{
    bool presence;

    for ( std::size_t i = 0; i < changed_locations_.size(); i++ )
    {
        for ( std::size_t j = 0; j < go_to_locations_compare_.size(); j++ )
        {
            if ( !changed_locations_[i].name.compare( go_to_locations_compare_[j].name ) )
            {
                locations_to_remove_.push_back( go_to_locations_compare_[j] );
            }
        }
    }

    for ( std::size_t i = 0; i < go_to_locations_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < go_to_locations_compare_.size(); j++ )
        {
            if ( go_to_locations_[i].name == go_to_locations_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            go_to_locations_compare_[index] = go_to_locations_[i];
        }
        else
        {
            go_to_locations_compare_.push_back( go_to_locations_[i] );
        }
    }

    for ( std::size_t i = 0; i < changed_slots_.size(); i++ )
    {
        for ( std::size_t j = 0; j < slots_compare_.size(); j++ )
        {
            if ( !changed_slots_[i].name.compare( slots_compare_[j].name ) )
            {
                slots_to_remove_.push_back( slots_compare_[j] );
            }
        }
    }

    for ( std::size_t i = 0; i < manipulation_slots_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < slots_compare_.size(); j++ )
        {
            if ( manipulation_slots_[i].name == slots_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            slots_compare_[index] = manipulation_slots_[i];
        }
        else
        {
            slots_compare_.push_back( manipulation_slots_[i] );
        }
    }

    for ( std::size_t i = 0; i < changed_groups_.size(); i++ )
    {
        for ( std::size_t j = 0; j < groups_compare_.size(); j++ )
        {
            if ( !changed_groups_.at(i).compare( groups_compare_.at(j) ) )
            {
                groups_to_remove_.push_back( groups_compare_.at(j) );
            }
        }
    }

    for ( std::size_t i = 0; i < groups_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < groups_compare_.size(); j++ )
        {
            if ( groups_[i] == groups_compare_[j] )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            groups_compare_[index] = groups_[i];
        }
        else
        {
            groups_compare_.push_back( groups_[i] );
        }
    }

    for ( std::size_t i = 0; i < changed_boxes_.size(); i++ )
    {
        for ( std::size_t j = 0; j < boxes_compare_.size(); j++ )
        {
            if ( !changed_boxes_[i].name.compare( boxes_compare_[j].name ) )
            {
                boxes_to_remove_.push_back( boxes_compare_[j] );
            }
        }
    }

    for ( std::size_t i = 0; i < boxes_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < boxes_compare_.size(); j++ )
        {
            if ( boxes_[i].name == boxes_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            boxes_compare_[index] = boxes_[i];
        }
        else
        {
            boxes_compare_.push_back( boxes_[i] );
        }
    }

    for ( std::size_t i = 0; i < go_to_actions_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < go_to_actions_compare_.size(); j++ )
        {
            if ( go_to_actions_[i].name == go_to_actions_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            go_to_actions_compare_[index] = go_to_actions_[i];
        }
        else
        {
            go_to_actions_compare_.push_back( go_to_actions_[i] );
        }
    }

    for ( std::size_t i = 0; i < place_actions_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < place_actions_compare_.size(); j++ )
        {
            if ( place_actions_[i].name == place_actions_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            place_actions_compare_[index] = place_actions_[i];
        }
        else
        {
            place_actions_compare_.push_back( place_actions_[i] );
        }
    }

    for ( std::size_t i = 0; i < pick_actions_.size(); i ++ )
    {
        presence = false;
        std::size_t index;
        for ( std::size_t j = 0; j < pick_actions_compare_.size(); j++ )
        {
            if ( pick_actions_[i].name == pick_actions_compare_[j].name )
            {
                presence = true;
                index = j;
            }
        }
        if ( presence )
        {
            pick_actions_compare_[index] = pick_actions_[i];
        }
        else
        {
            pick_actions_compare_.push_back( pick_actions_[i] );
        }
    }
}

void QNode::checkRecipesParam()
{
    for ( std::size_t i = 0; i < recipes_compare_.size(); i++ )
    {
        bool presence = false;
        for ( std::size_t j = 0; j < recipes_.size(); j++)
        {
            if ( recipes_[j].name == recipes_compare_[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            recipes_.push_back( recipes_compare_[i] );
        }
    }

    recipes_compare_ = recipes_;
}

bool QNode::saveComponents()
{
    XmlRpc::XmlRpcValue param;

    checkObjectsParam();
    checkOtherParam();

    for ( std::size_t i = 0; i < go_to_locations_compare_.size(); i++)
    {
        param[i] = getGoToLocationParam(i);
    }
    n_.setParam("/go_to_location", param);
    param.clear();

    for ( std::size_t i = 0; i < boxes_compare_.size(); i++)
    {
        param[i] = getBoxParam(i);
    }
    n_.setParam("/inbound/boxes", param);
    param.clear();

    for ( std::size_t i = 0; i < groups_compare_.size(); i++)
    {
        param[i] = getGroupParam(i);
    }
    n_.setParam("/outbound/slots_group", param);
    param.clear();

    for ( std::size_t i = 0; i < slots_compare_.size(); i++)
    {
        param[i] = getSlotParam( i );
    }
    n_.setParam("/outbound/slots", param);
    param.clear();

    for ( std::size_t i = 0; i < objects_compare_.size(); i++ )
    {
        param [i] = getObjectParam(i);
    }
    n_.setParam("/manipulation_object_types", param);
    param.clear();

    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::SaveParam>("/save_components_params_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    if (client.call(srv))
      return true;
    else
      return false;
}

bool QNode::loadNewLocation(const go_to_location& location_to_add)
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
        {
            ROS_INFO("Location %s added to location manager", location_to_add.name.c_str() );
        }
    }
    return true;
}

bool QNode::loadNewBox( const box &box_to_add)
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
        {
            ROS_INFO("Box %s added to location manager", box_to_add.name.c_str() );
        }

    }
    return true;
}

bool QNode::loadNewGroup( const std::string &group_to_add)
{
    if ( !group_to_add.empty() )
    {
        ROS_INFO("Group to add: %s", group_to_add.c_str());
        bool presence = false;
        for ( int i = 0; i < logging_model_group_.rowCount(); i++ )
        {
          if ( !group_to_add.compare(logging_model_group_.data( logging_model_group_.index( i ),0 ).toString().toStdString()) )
          {
            presence = true;
          }
        }
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
          {
            ROS_INFO("Added group: %s", group_to_add.c_str());
          }
        }
    }
    return true;
}

bool QNode::loadNewSlot( const manipulation_slot &slot_to_add)
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
        {
            ROS_INFO("Added slot: %s", slot_to_add.name.c_str());
        }
    }
    return true;
}

bool QNode::saveActions()
{

    XmlRpc::XmlRpcValue param;

    checkObjectsParam();
    checkOtherParam();

    for ( std::size_t i = 0; i < go_to_actions_compare_.size(); i++)
    {
        param[i] = getGoToParam(i);
    }
    for ( std::size_t i = 0; i < pick_actions_compare_.size(); i++)
    {
        param[ i + go_to_actions_compare_.size() ] = getPickParam(i);
    }
    for ( std::size_t i = 0; i < place_actions_compare_.size(); i++)
    {
        param[ i + go_to_actions_compare_.size() + pick_actions_compare_.size() ] = getPlaceParam(i);
    }
    n_.setParam("/multi_skills/tasks", param);
    param.clear();

    ros::ServiceClient client = n_.serviceClient<manipulation_interface_mongo::SaveParam>("/save_actions_param_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    if (client.call(srv))
      return true;
    else
      return false;
}

void QNode::addObject( const object_type object )
{
    logObject(object.type);
    logObjectModify(object.type);
    objects_.push_back(object);
}

void QNode::addSlot( const manipulation_slot slot )
{
    if ( logging_model_group_.rowCount()!=0 )
    {
        bool add = true;
        for ( int i=0; i<logging_model_group_.rowCount(); i++)
        {
            if ( !slot.group.compare( logging_model_group_.data( logging_model_group_.index( i ),0 ).toString().toStdString() ) )
            {
                add = false;
            }
        }
        if ( add )
        {
            logGroup(slot.group);
            groups_.push_back(slot.group);

            changed_groups_.push_back(slot.group);
        }
    }
    else
    {
        logGroup(slot.group);
        groups_.push_back(slot.group);

        changed_groups_.push_back(slot.group);
    }

    logSlot(slot.name);
    logSlotModify(slot.name);

    manipulation_slots_.push_back(slot);
    changed_slots_.push_back(slot);
}

void QNode::addBox(const box internal_box)
{
    logBox(internal_box.name);
    logBoxModify(internal_box.name);
    boxes_.push_back(internal_box);
    changed_boxes_.push_back(internal_box);
}

void QNode::addLocationChanges(const int ind, const go_to_location new_location)
{
    if ( !go_to_locations_[ind].name.compare( new_location.name ) )
    {
        go_to_locations_[ind] = new_location;
    }
    else
    {
        go_to_locations_.push_back( new_location );
        logLocation(new_location.name);
        logLocationModify(new_location.name);
    }
}

void QNode::addSlotChanges(const int ind, const manipulation_slot new_slot)
{
    if ( !manipulation_slots_[ind].name.compare( new_slot.name ) )
    {
        manipulation_slots_[ind] = new_slot;
    }
    else
    {
        manipulation_slots_.push_back(new_slot);
        logSlot(new_slot.name);
        logSlotModify(new_slot.name);
    }
}

void QNode::addBoxChanges(const int ind, const box new_box)
{
    if ( !boxes_[ind].name.compare( new_box.name ) )
    {
        boxes_[ind] = new_box;
    }
    else
    {
        boxes_.push_back( new_box );
        logBox(new_box.name);
        logBoxModify(new_box.name);
    }
}

void QNode::addObjectChanges(const int ind, const object_type new_object)
{
    if ( !objects_[ind].type.compare( new_object.type ) )
    {
        objects_[ind] = new_object;
    }
    else
    {
        objects_.push_back( new_object );
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

void QNode::loadParam( const int ind )
{
    if (!n_.getParamNames(param_names_))
    {
        ROS_ERROR("Empty robot parameter");
        return;
    }

    if ( ind == 1 )
    {
        loadRobots();

        setTargetFrame(0);

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
    }
    else if ( ind == 2 )
    {
        readGotoPickAndPlaceFromParam();
        ROS_INFO("Read actions");
    }
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
        std::size_t found  = param_names_[i].find("/go_to_location_server/groups/");
        std::size_t found2 = param_names_[i].find("/", 26);
        std::string str = param_names_[i];
        if ( found != std::string::npos && found2 != std::string::npos )
        {
            robots_.push_back( str.erase( 0, found2+1 ) );
            robot_name_params_.push_back( param_names_[i] );
        }
    }
}

void QNode::writeParam(const int ind)
{
    loadParam( ind );

    if ( ind == 1)
    {
        for ( std::size_t i = 0; i < boxes_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box_.rowCount(); j++)
            {
                if ( !boxes_[i].name.compare(logging_model_box_.data( logging_model_box_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logBox(boxes_[i].name);
            }
        }
        for ( std::size_t i = 0; i < boxes_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box_modify_.rowCount(); j++)
            {
                if ( !boxes_[i].name.compare(logging_model_box_modify_.data( logging_model_box_modify_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logBoxModify(boxes_[i].name);
            }
        }
        for ( std::size_t i = 0; i < objects_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object_.rowCount(); j++)
            {
                if ( !objects_[i].type.compare(logging_model_object_.data( logging_model_object_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logObject(objects_[i].type);
            }
        }
        for ( std::size_t i = 0; i < objects_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object_modify_.rowCount(); j++)
            {
                if ( !objects_[i].type.compare(logging_model_object_modify_.data( logging_model_object_modify_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logObjectModify(objects_[i].type);
            }
        }
        for ( std::size_t i = 0; i < groups_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_group_.rowCount(); j++)
            {
                if ( !groups_[i].compare(logging_model_group_.data( logging_model_group_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logGroup( groups_[i] );
            }
        }
        for ( std::size_t i = 0; i < manipulation_slots_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot_.rowCount(); j++)
            {
                if ( !manipulation_slots_[i].name.compare(logging_model_slot_.data( logging_model_slot_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logSlot(manipulation_slots_[i].name);
            }
        }
        for ( std::size_t i = 0; i < manipulation_slots_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot_modify_.rowCount(); j++)
            {
                if ( !manipulation_slots_[i].name.compare(logging_model_slot_modify_.data( logging_model_slot_modify_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logSlotModify(manipulation_slots_[i].name);
            }
        }
        for ( std::size_t i = 0; i < go_to_locations_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location_.rowCount(); j++)
            {
                if ( !go_to_locations_[i].name.compare(logging_model_location_.data( logging_model_location_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logLocation(go_to_locations_[i].name);
            }
        }
        for ( std::size_t i = 0; i < go_to_locations_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location_modify_.rowCount(); j++)
            {
                if ( !go_to_locations_[i].name.compare(logging_model_location_modify_.data( logging_model_location_modify_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logLocationModify(go_to_locations_[i].name);
            }
        }
    }
    else if ( ind == 2)
    {
        for ( std::size_t i = 0; i < go_to_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_go_to_.rowCount(); j++)
            {
                if ( !go_to_actions_[i].name.compare(logging_model_go_to_.data( logging_model_go_to_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logGoTo(go_to_actions_[i].name);
            }
        }
        for ( std::size_t i = 0; i < go_to_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_go_to_.rowCount(); j++)
            {
                if ( !go_to_actions_[i].name.compare(logging_model_second_go_to_.data( logging_model_second_go_to_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logSecondGoTo(go_to_actions_[i].name);
            }
        }
        for ( std::size_t i = 0; i < place_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_place_.rowCount(); j++)
            {
                if ( !place_actions_[i].name.compare(logging_model_place_.data( logging_model_place_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logPlace(place_actions_[i].name);
            }
        }
        for ( std::size_t i = 0; i < place_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_place_.rowCount(); j++)
            {
                if ( !place_actions_[i].name.compare(logging_model_second_place_.data( logging_model_second_place_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logSecondPlace(place_actions_[i].name);
            }
        }
        for ( std::size_t i = 0; i < pick_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_pick_.rowCount(); j++)
            {
                if ( !pick_actions_[i].name.compare(logging_model_pick_.data( logging_model_pick_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logPick(pick_actions_[i].name);
            }
        }
        for ( std::size_t i = 0; i < pick_actions_.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_pick_.rowCount(); j++)
            {
                if ( !pick_actions_[i].name.compare(logging_model_second_pick_.data( logging_model_second_pick_.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                logSecondPick(pick_actions_[i].name);
            }
        }
    }
}

void QNode::writeLocations()
{
    if ( logging_model_components_.rowCount() != 0 )
    {
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    }
    for ( std::size_t i = 0; i < go_to_locations_.size(); i++)
    {
        logComponents(go_to_locations_[i].name);
    }
}

void QNode::writeObjects()
{
    if ( logging_model_components_.rowCount() != 0 )
    {
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    }
    for ( std::size_t i = 0; i < objects_.size(); i++)
    {
        logComponents(objects_[i].type);
    }
}

void QNode::writeGroups()
{
    if ( logging_model_components_.rowCount() != 0 )
    {
        logging_model_components_.removeRows( 0, logging_model_components_.rowCount() );
    }
    for ( std::size_t i = 0; i < groups_.size(); i++)
    {
        logComponents(groups_[i]);
    }
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

    box box_;

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
        box_.name = rosparam_utilities::toString(param["name"]);

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            continue;
        }
        box_.frame = rosparam_utilities::toString(param["frame"]);

        std::string what;
        std::vector<double> pos;
        if( !rosparam_utilities::getParam(param,"position",pos,what) )
        {
            ROS_WARN("The element #%d has not the field 'position'", i);
            continue;
        }
        assert(pos.size()==3);

        box_.location_.pos.origin_x = pos.at(0);
        box_.location_.pos.origin_y = pos.at(1);
        box_.location_.pos.origin_z = pos.at(2);

        std::vector<double> quat;
        if( !rosparam_utilities::getParam(param,"quaternion",quat,what) )
        {
            ROS_WARN("The element #%d has not the field 'quaternion'", i);
            continue;
        }
        assert(quat.size()==4);

        box_.location_.quat.rotation_x = quat.at(0);
        box_.location_.quat.rotation_y = quat.at(1);
        box_.location_.quat.rotation_z = quat.at(2);
        box_.location_.quat.rotation_w = quat.at(3);

        std::vector<double> approach_distance_d;
        if( !rosparam_utilities::getParam(param,"approach_distance",approach_distance_d,what) )
        {
            ROS_WARN("The box %s has not the field 'approach_distance'",box_.name.c_str());
            return false;
        }
        assert(approach_distance_d.size() == 3);

        box_.approach.origin_x = approach_distance_d.at(0);
        box_.approach.origin_y = approach_distance_d.at(1);
        box_.approach.origin_z = approach_distance_d.at(2);

        std::vector<double> leave_distance_d;
        if( !rosparam_utilities::getParam(param,"leave_distance",leave_distance_d,what) )
        {
            ROS_WARN("The box %s has not the field 'leave_distance'",box_.name.c_str());
            return false;
        }
        assert(leave_distance_d.size() == 3);

        box_.leave.origin_x = leave_distance_d.at(0);
        box_.leave.origin_y = leave_distance_d.at(1);
        box_.leave.origin_z = leave_distance_d.at(2);

        bool presence = false;
        for ( std::size_t j = 0; j < boxes_.size(); j++)
        {
            if ( !box_.name.compare(boxes_[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            if ( loadNewBox(box_) )
            {
                boxes_.push_back(box_);
            }
            boxes_compare_.push_back(box_);
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
        object_type object_;
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
        object_.type = rosparam_utilities::toString(single_object["type"]);

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

            object_.grasp.push_back( loc );

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

            object_.approach.push_back( approach );

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

            object_.leave.push_back( leave );

            if ( !single_grasp.hasMember("tool"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'tool'", j, i);
                continue;
            }

            object_.tool.push_back( rosparam_utilities::toString(single_grasp["tool"]) );

            if ( !single_grasp.hasMember("pre_gripper_position"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'pre_gripper_position'", j, i);
                continue;
            }

            object_.pre_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["pre_gripper_position"]) );

            if ( !single_grasp.hasMember("post_gripper_position"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'post_gripper_position'", j, i);
                continue;
            }

            object_.post_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["post_gripper_position"]) );

            if ( !single_grasp.hasMember("gripper_force"))
            {
                ROS_WARN("The grasp element #%d of object element #%d has not the field 'gripper_force'", j, i);
                continue;
            }

            object_.gripper_force.push_back( rosparam_utilities::toDouble(single_grasp["gripper_force"]) );

        }
        bool presence = false;
        for ( std::size_t j = 0; j < objects_.size(); j++)
        {
            if ( !object_.type.compare(objects_[j].type) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            objects_.push_back( object_ );
        }
        objects_compare_.push_back( object_ );
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
        std::string group_name_ = rosparam_utilities::toString(slot_group_["name"]);
        bool presence = false;
        for ( std::size_t j = 0; j < groups_.size(); j++)
        {
            if ( !group_name_.compare( groups_[j] ) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            if ( loadNewGroup( group_name_ ) )
            {
                groups_.push_back( group_name_ );
            }
            groups_compare_.push_back( group_name_ );
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

        manipulation_slot slot_;

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
        slot_.name = rosparam_utilities::toString(param["name"]);

        if( !param.hasMember("slots_group") )
        {
            ROS_WARN("The element #%d has not the field 'slots_group'", i);
            return false;
        }
        slot_.group = rosparam_utilities::toString(param["slots_group"]);

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            return false;
        }
        slot_.frame = rosparam_utilities::toString(param["frame"]);

        if( !param.hasMember("max_objects") )
        {
            ROS_WARN("The element #%d has not the field 'max_objects'", i);
            return false;
        }
        slot_.max_objects = rosparam_utilities::toInt(param["max_objects"]);

        std::string what;
        std::vector<double> approach_distance_d;
        if( !rosparam_utilities::getParam(param,"approach_distance",approach_distance_d,what) )
        {
            ROS_WARN("Slot %s has not the field 'approach_distance'",slot_.name.c_str());
            return false;
        }
        assert(approach_distance_d.size()==3);
        slot_.approach.origin_x = approach_distance_d.at(0);
        slot_.approach.origin_y = approach_distance_d.at(1);
        slot_.approach.origin_z = approach_distance_d.at(2);

        std::vector<double> leave_distance_d;
        if( !rosparam_utilities::getParam(param,"leave_distance",leave_distance_d,what) )
        {
            ROS_WARN("Slot %s has not the field 'leave_distance'",slot_.name.c_str());
            return false;
        }
        assert(leave_distance_d.size()==3);
        slot_.leave.origin_x = leave_distance_d.at(0);
        slot_.leave.origin_y = leave_distance_d.at(1);
        slot_.leave.origin_z = leave_distance_d.at(2);

        location loc;
        std::vector<double> position;
        if( !rosparam_utilities::getParam(param,"position",position,what) )
        {
            ROS_WARN("Slot %s has not the field 'position'",slot_.name.c_str());
            return false;
        }
        assert(position.size()==3);

        loc.pos.origin_x = position.at(0);
        loc.pos.origin_y = position.at(1);
        loc.pos.origin_z = position.at(2);

        std::vector<double> quaternion;
        if( !rosparam_utilities::getParam(param,"quaternion",quaternion,what) )
        {
            ROS_WARN("Slot %s has not the field 'quaternion'",slot_.name.c_str());
            return false;
        }
        assert(quaternion.size()==4);

        loc.quat.rotation_x = quaternion.at(0);
        loc.quat.rotation_y = quaternion.at(1);
        loc.quat.rotation_z = quaternion.at(2);
        loc.quat.rotation_w = quaternion.at(3);

        slot_.location_ = loc;

        bool presence = false;
        for ( std::size_t j = 0; j < manipulation_slots_.size(); j++)
        {
            if ( !slot_.name.compare(manipulation_slots_[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            if ( loadNewSlot(slot_) )
            {
                manipulation_slots_.push_back(slot_);
            }
            slots_compare_.push_back(slot_);
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
        go_to_location go_to_;
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
        go_to_.name = rosparam_utilities::toString(single_location["name"]);

        if( !single_location.hasMember("frame") )
        {
            ROS_WARN("The element #%d has not the field 'frame'", i);
            return false;
        }
        go_to_.frame = rosparam_utilities::toString(single_location["frame"]);

        std::string what;
        std::vector<double> pos;
        if( !rosparam_utilities::getParam(single_location,"position",pos,what) )
        {
            ROS_WARN("Slot %s has not the field 'position'",go_to_.name.c_str());
            return false;
        }
        assert(pos.size()==3);

        go_to_.location_.pos.origin_x = pos.at(0);
        go_to_.location_.pos.origin_y = pos.at(1);
        go_to_.location_.pos.origin_z = pos.at(2);

        std::vector<double> quat;
        if( !rosparam_utilities::getParam(single_location,"quaternion",quat,what) )
        {
            ROS_WARN("Slot %s has not the field 'quaternion'",go_to_.name.c_str());
            return false;
        }
        assert(quat.size()==4);

        go_to_.location_.quat.rotation_x = quat.at(0);
        go_to_.location_.quat.rotation_y = quat.at(1);
        go_to_.location_.quat.rotation_z = quat.at(2);
        go_to_.location_.quat.rotation_w = quat.at(3);

        bool presence = false;
        for ( std::size_t j = 0; j < go_to_locations_.size(); j++)
        {
            if ( !go_to_.name.compare(go_to_locations_[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            if ( loadNewLocation(go_to_) )
            {
                go_to_locations_.push_back(go_to_);
            }
            go_to_locations_compare_.push_back(go_to_);
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
        std::string type_ = rosparam_utilities::toString(param["type"]);

        if ( !type_.compare("goto") )
        {
            go_to_action go_to_;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The action #%d has not the field 'name'", i);
                return false;
            }
            go_to_.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> locations_;
            if( !rosparam_utilities::getParam(param,"goal",locations_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_.locations = locations_;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                go_to_.description = " ";
            }
            else
            {
                go_to_.description = rosparam_utilities::toString(param["description"]);
            }

            bool presence = false;
            for ( std::size_t j = 0; j < go_to_actions_.size(); j++)
            {
                if ( !go_to_.name.compare(go_to_actions_[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                go_to_actions_.push_back(go_to_);
                go_to_actions_compare_.push_back(go_to_);
            }
        }

        if ( !type_.compare("place") )
        {
            place place_;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The element place #%d has not the field 'name'", i);
                return false;
            }
            place_.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> groups;
            if( !rosparam_utilities::getParam(param,"goal",groups,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_.groups = groups;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                place_.description = " ";
            }
            else
            {
                place_.description = rosparam_utilities::toString(param["description"]);
            }

            bool presence = false;
            for ( std::size_t j = 0; j < place_actions_.size(); j++)
            {
                if ( !place_.name.compare(place_actions_[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                place_actions_.push_back(place_);
                place_actions_compare_.push_back(place_);
            }
        }

        if ( !type_.compare("pick") )
        {
            pick pick_;

            if( !param.hasMember("name") )
            {
                ROS_WARN("The element pick #%d has not the field 'name'", i);
                return false;
            }
            pick_.name = rosparam_utilities::toString(param["name"]);

            std::string what;
            std::vector<std::string> objects;
            if( !rosparam_utilities::getParam(param,"goal",objects,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_.objects = objects;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_.agents = agents_;

            if( !param.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                pick_.description = " ";
            }
            else
            {
                pick_.description = rosparam_utilities::toString(param["description"]);
            }

            bool presence = false;
            for ( std::size_t j = 0; j < pick_actions_.size(); j++)
            {
                if ( !pick_.name.compare(pick_actions_[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                pick_actions_.push_back(pick_);
                pick_actions_compare_.push_back(pick_);
            }
        }
    }

    return true;
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
