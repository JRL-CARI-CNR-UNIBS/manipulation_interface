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
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_listener.h>
#include "qnode.hpp"
#include <fstream>
#include <rosparam_utilities/rosparam_utilities.h>
#include <manipulation_interface_mongo/SaveParam.h>
#include <manipulation_interface_gui/recipe_test_msg.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/ListObjects.h>
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
             ros::NodeHandle n_,
             ros::NodeHandle nh_i_,
             ros::NodeHandle nh_o_,
             ros::NodeHandle nh_g_) :
    n(n_),
    nh_i(nh_i_),
    nh_o(nh_o_),
    nh_g(nh_g_)
{}

QNode::~QNode()
{
    if ( t_component.joinable() )
        t_component.join();
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}



bool QNode::init()
{
    twist_pub= n.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);
    set_ctrl_srv = n.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    gripper_srv = n.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");

    start_ctrl_req.request.start_configuration = "watch";
    start_ctrl_req.request.strictness = 1;

    set_ctrl_srv.waitForExistence();

    if ( !set_ctrl_srv.call(start_ctrl_req) )
    {
        ROS_ERROR("Unable to call %s service to set controller %s",set_ctrl_srv.getService().c_str(),start_ctrl_req.request.start_configuration.c_str());
        return false;
    }

    if (!start_ctrl_req.response.ok)
    {
        ROS_ERROR("Error on service %s response", set_ctrl_srv.getService().c_str());
        return false;
    }

    ROS_INFO("Controller %s started.",start_ctrl_req.request.start_configuration.c_str());

    load_TF();
    load_robots();

    js_sub = std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(n,"/gripper/joint_states",10);

    add_locations_client_      = n.serviceClient<manipulation_msgs::AddLocations>     ("/go_to_location_server/add_locations");
    add_boxes_client_          = n.serviceClient<manipulation_msgs::AddBoxes>         ("/inbound_pick_server/add_boxes");
    add_objs_client_           = n.serviceClient<manipulation_msgs::AddObjects>       ("/inbound_pick_server/add_objects");
    add_slots_group_client_    = n.serviceClient<manipulation_msgs::AddSlotsGroup>    ("/outbound_place_server/add_slots_group");
    add_slots_client_          = n.serviceClient<manipulation_msgs::AddSlots>         ("/outbound_place_server/add_slots");
    remove_locations_client_   = n.serviceClient<manipulation_msgs::RemoveLocations>  ("/go_to_location_server/remove_locations");
    remove_boxes_client_       = n.serviceClient<manipulation_msgs::RemoveBoxes>      ("/inbound_pick_server/remove_boxes");
    remove_objs_client_        = n.serviceClient<manipulation_msgs::RemoveObjects>    ("/inbound_pick_server/remove_objects");
    remove_slots_group_client_ = n.serviceClient<manipulation_msgs::RemoveSlotsGroup> ("/outbound_place_server/remove_slots_group");
    remove_slots_client_       = n.serviceClient<manipulation_msgs::RemoveSlots>      ("/outbound_place_server/remove_slots");
    list_objects_client_       = n.serviceClient<object_loader_msgs::ListObjects>     ("/list_objects");


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

    return true;
}

void QNode::initial_add_components_in_manipulation()
{
    changed_boxes     = boxes;
    changed_locations = go_to_locations;
    changed_slots     = manipulation_slots;
    changed_groups    = groups;

    t_component = std::thread(&QNode::load_new_params_in_manipulation, this,
                              changed_locations,
                              changed_slots,
                              changed_boxes,
                              changed_groups,
                              locations_to_remove,
                              slots_to_remove,
                              boxes_to_remove,
                              groups_to_remove);

    changed_locations.clear();
    changed_slots.clear();
    changed_boxes.clear();
    changed_groups.clear();
}

void QNode::load_objects_in_manipulation()
{
    object_loader_msgs::ListObjects objects_list;
    if ( !list_objects_client_.call( objects_list ) )
    {
        ROS_ERROR("Unable to obtain the object list");
        return;
    }

    manipulation_msgs::RemoveObjects remove_objects_srv;
    manipulation_msgs::AddObjects add_objects_srv;

    if ( !objects_list.response.ids.empty() )
    {
        remove_objects_srv.request.object_names = objects_list.response.ids;

        if ( !remove_objs_client_.call(remove_objects_srv) )
        {
            ROS_ERROR("Unable to remove the objects by the manipulation");
            return;
        }

        for ( int i = 0; i < objects_list.response.ids.size(); i++ )
        {
            manipulation_msgs::Object obj;

            obj.name = objects_list.response.ids[i];
            obj.type = objects_list.response.types[i];

            int index;
            for ( int j = 0; j < objects.size(); j++ )
            {
                if ( !obj.type.compare( objects[j].type ) )
                {
                    index = j;
                }
            }

            for ( int j = 0; j < objects[index].grasp.size(); j++ )
            {
                manipulation_msgs::Grasp grasp_;
                grasp_.tool_name = objects[index].tool[j];
                grasp_.location.name = obj.name+"/grasp_"+std::to_string(j)+"_"+objects[index].tool[j];
                grasp_.location.frame = obj.name;

                grasp_.location.pose.position.x    = objects[index].grasp[j].pos.origin_x;
                grasp_.location.pose.position.y    = objects[index].grasp[j].pos.origin_y;
                grasp_.location.pose.position.z    = objects[index].grasp[j].pos.origin_z;
                grasp_.location.pose.orientation.w = objects[index].grasp[j].quat.rotation_w;
                grasp_.location.pose.orientation.x = objects[index].grasp[j].quat.rotation_x;
                grasp_.location.pose.orientation.y = objects[index].grasp[j].quat.rotation_y;
                grasp_.location.pose.orientation.z = objects[index].grasp[j].quat.rotation_z;

                grasp_.location.approach_relative_pose.position.x = objects[index].approach[j].origin_x;
                grasp_.location.approach_relative_pose.position.y = objects[index].approach[j].origin_y;
                grasp_.location.approach_relative_pose.position.z = objects[index].approach[j].origin_z;

                grasp_.location.leave_relative_pose.position.x = objects[index].leave[j].origin_x;
                grasp_.location.leave_relative_pose.position.y = objects[index].leave[j].origin_y;
                grasp_.location.leave_relative_pose.position.z = objects[index].leave[j].origin_z;

                obj.grasping_locations.push_back(grasp_);
            }

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
          ROS_ERROR("Box not fount");
        }
    }
}

void QNode::cartMove (std::vector<float> twist_move)
{
    geometry_msgs::TwistStamped twist_command;

    twist_command.header.frame_id=frame_id;
    twist_command.twist.linear.x=twist_move.at(0);
    twist_command.twist.linear.y=twist_move.at(1);
    twist_command.twist.linear.z=twist_move.at(2);
    twist_command.twist.angular.x=twist_move.at(3);
    twist_command.twist.angular.y=twist_move.at(4);
    twist_command.twist.angular.z=twist_move.at(5);

    twist_command.header.stamp=ros::Time::now();
    twist_pub.publish(twist_command);
}

void QNode::add_object_type (int ind)
{
    if ( logging_model_action_components.rowCount() != 0 )
    {
        logging_model_action_components.removeRows(0,logging_model_action_components.rowCount());
    }
    for ( int i = 0; i < pick_actions[ind].objects.size(); i++)
    {
        log_action_components( pick_actions[ind].objects[i] );
    }
}

void QNode::add_slot_groups (int ind)
{
    if ( logging_model_action_components.rowCount() != 0 )
    {
        logging_model_action_components.removeRows(0,logging_model_action_components.rowCount());
    }
    for ( int i = 0; i < place_actions[ind].groups.size(); i++)
    {
        log_action_components( place_actions[ind].groups[i] );
    }
}

void QNode::add_location_info (int ind)
{
    if ( logging_model_action_components.rowCount() != 0 )
    {
        logging_model_action_components.removeRows(0,logging_model_action_components.rowCount());
    }
    for ( int i = 0; i < go_to_actions[ind].locations.size(); i++)
    {
        log_action_components( go_to_actions[ind].locations[i] );
    }
}

bool QNode::add_second_location_info( int ind )
{
    if ( logging_model_info_action.rowCount() != 0 )
    {
        logging_model_info_action.removeRows(0,logging_model_info_action.rowCount());
    }
    for ( int i = 0; i < go_to_actions[ind].locations.size(); i++)
    {
        log_info_action( go_to_actions[ind].locations[i] );
    }
}

bool QNode::add_second_slot_groups  ( int ind )
{
    if ( logging_model_info_action.rowCount() != 0 )
    {
        logging_model_info_action.removeRows(0,logging_model_info_action.rowCount());
    }
    for ( int i = 0; i < place_actions[ind].groups.size(); i++)
    {
        log_info_action( place_actions[ind].groups[i] );
    }
}

bool QNode::add_second_object_type  ( int ind )
{
    if ( logging_model_info_action.rowCount() != 0 )
    {
        logging_model_info_action.removeRows(0,logging_model_info_action.rowCount());
    }
    for ( int i = 0; i < pick_actions[ind].objects.size(); i++)
    {
        log_info_action( pick_actions[ind].objects[i] );
    }
}

bool QNode::add_object_copy_grasp   ( int index, int index2 )
{
    objects[index].approach.push_back( objects[index].approach[index2] );
    objects[index].grasp.push_back   ( objects[index].grasp[index2] );
    objects[index].tool.push_back    ( objects[index].tool[index2] );
}
void QNode::write_recipe ( int index)
{
    for ( int i = 0; i < recipes[index].recipe_.size(); i++ )
    {
        log_recipe( recipes[index].recipe_[i] );
    }
}

std::vector<std::string> QNode::load_recipes_param ()
{
    XmlRpc::XmlRpcValue config;

    std::vector<std::string> recipes_names;

    if ( !n.getParam("/multi_skills/recipes", config) )
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
        for ( int j = 0; j < recipes.size(); j++)
        {
            if ( !single_recipe.name.compare(recipes[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            recipes.push_back(single_recipe);
            recipes_compare.push_back(single_recipe);
            recipes_names.push_back(single_recipe.name);
        }
    }

    return recipes_names;
}

bool QNode::add_recipe(std::string recipe_name)
{
    recipe recipe_;
    recipe_.name = recipe_name;

    for ( int i = 0; i < logging_model_recipe.rowCount(); i++ )
    {
        recipe_.recipe_.push_back( logging_model_recipe.data( logging_model_recipe.index(i) ).toString().toStdString() );
    }

    for ( int i = 0; i < recipes.size(); i++ )
    {
        if ( !recipe_name.compare( recipes[i].name ) )
        {
            return false;
        }
        if ( compare( recipe_.recipe_, recipes[i].recipe_ ) )
        {
            return false;
        }
    }

    recipes.push_back(recipe_);

    return true;
}

bool QNode::remove_recipe(int ind)
{
    recipes.erase(recipes.begin()+ind);
}

bool QNode::save_recipe()
{
    XmlRpc::XmlRpcValue param;

    check_recipes_param();

    for ( int i = 0; i < recipes.size(); i++)
    {
        param[i] = get_recipe_param(i);
    }
    n.setParam("/multi_skills/recipes", param);
    param.clear();

    ros::ServiceClient client = n.serviceClient<manipulation_interface_mongo::SaveParam>("save_recipe_param_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    client.waitForExistence();
    if ( !client.call(srv) )
    {
        ROS_ERROR("Unable to call %s service",client.getService().c_str());
        return false;
    }
    return true;
}

int QNode::run_recipe()
{
    if ( tc_finito )
    {
        t_component.join();
        tc_finito = false;
    }

    if ( t_component.joinable() )
    {
        ROS_ERROR("The previous thread has not finished");
        return 1;
    }

    std::vector<std::string> recipe_;

    for ( int i = 0; i < logging_model_recipe.rowCount(); i++ )
    {
        recipe_.push_back( logging_model_recipe.data( logging_model_recipe.index(i) ).toString().toStdString() );
    }

    XmlRpc::XmlRpcValue param;
    param = get_recipe_param( recipe_ );

    n.setParam("recipe_to_run", param);

    ros::ServiceClient run_recipe_client = n.serviceClient<manipulation_interface_gui::recipe_test_msg>("run_recipe");
    manipulation_interface_gui::recipe_test_msg recipe_msg;
    recipe_msg.request.input = "manipulator";

    if (run_recipe_client.call(recipe_msg))
    {
        ROS_INFO("Done");
    }
    else
    {
        ROS_ERROR("Failed to call service run_recipe");
        return 2;
    }
    return 0;
}

int QNode::run_selected_action( int index )
{
    if ( tc_finito )
    {
        t_component.join();
        tc_finito = false;
    }

    if ( t_component.joinable() )
    {
        ROS_ERROR("The previous thread has not finished");
        return 1;
    }

    std::vector<std::string> recipe_;

    recipe_.push_back( logging_model_recipe.data( logging_model_recipe.index(index) ).toString().toStdString() );

    XmlRpc::XmlRpcValue param;
    param = get_recipe_param( recipe_ );

    n.setParam("recipe_to_run", param);

    ros::ServiceClient run_recipe_client = n.serviceClient<manipulation_interface_gui::recipe_test_msg>("run_recipe");
    manipulation_interface_gui::recipe_test_msg recipe_msg;
    recipe_msg.request.input = "manipulator";

    if (run_recipe_client.call(recipe_msg))
    {
        ROS_INFO("Done");
    }
    else
    {
        ROS_ERROR("Failed to call service run_recipe");
        return 2;
    }
    return 0;
}

XmlRpc::XmlRpcValue QNode::get_recipe_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", recipes[index].name));
    xml_body.append(get_xml_group_string("recipe", recipes[index].recipe_));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_recipe_param(std::vector<std::string> recipe_)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);


    for ( int i = 0; i < recipe_.size(); i++)
    {
        xml_body.append(init_value);
        xml_body.append(init_string);
        xml_body.append(recipe_[i]);
        xml_body.append(end_string);
        xml_body.append(end_value);
    }

    xml_body.append(end_data);
    xml_body.append(end_array);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_action_go_to_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("action", "goto"));
    xml_body.append(get_xml_string_param("to_loc_ctrl_id", "trj_tracker"));
    xml_body.append(get_xml_string_param("property_exec_id", "open"));
    xml_body.append(get_xml_string_param("tool_id", "gripper_fake"));
    std::vector<std::string> names;
    names.push_back(go_to_actions[index].name);
    xml_body.append(get_xml_group_string("description", names));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_action_place_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("action", "place"));
    xml_body.append(get_xml_string_param("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(get_xml_string_param("property_exec_id", "open"));
    xml_body.append(get_xml_string_param("tool_id", "gripper_fake"));
    xml_body.append(get_xml_group_string("description", place_actions[index].groups));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_action_pick_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("action", "pick"));
    xml_body.append(get_xml_string_param("approach_loc_ctrl_id", "trj_tracker"));
    xml_body.append(get_xml_string_param("property_exec_id", "open"));
    xml_body.append(get_xml_string_param("tool_id", "gripper_fake"));
    xml_body.append(get_xml_group_string("description", pick_actions[index].objects));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

bool QNode::add_go_to(std::string go_to_name, std::vector<std::string> locations_, std::string description, std::vector<std::string> agents_)
{
    if ( logging_model_go_to.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_go_to.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !go_to_name.compare( logging_model_go_to.data( logging_model_go_to.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !go_to_name.compare( logging_model_place.data( logging_model_place.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !go_to_name.compare( logging_model_pick.data( logging_model_pick.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
        for ( int i = 0; i < go_to_actions.size(); i++)
        {
            if ( compare( locations_, go_to_actions[i].locations) )
            {
                return false;
            }
        }
    }
    log_go_to(go_to_name);
    log_second_go_to(go_to_name);

    go_to_action gt;
    gt.agents      = agents_;
    gt.name = go_to_name;
    gt.locations = locations_;
    gt.description = description;
    go_to_actions.push_back(gt);

    return true;
}

bool QNode::add_place(std::string place_name, std::vector<std::string> groups_, std::string description, std::vector<std::string> agents_)
{
    if ( logging_model_place.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_place.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !place_name.compare( logging_model_go_to.data( logging_model_go_to.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !place_name.compare( logging_model_place.data( logging_model_place.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
            if ( !place_name.compare( logging_model_pick.data( logging_model_pick.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
        for ( int i = 0; i < place_actions.size(); i++)
        {
            if ( compare( groups_, place_actions[i].groups) )
            {
                return false;
            }
        }
    }
    log_place(place_name);
    log_second_place(place_name);

    place plc;
    plc.agents      = agents_;
    plc.name        = place_name;
    plc.groups      = groups_;
    plc.description = description;
    place_actions.push_back(plc);

    return true;
}

bool QNode::add_pick(std::string pick_name, std::vector<std::string> objects_, std::string description, std::vector<std::string> agents_)
{
    if ( !pick_name.empty())
    {
        if ( logging_model_pick.rowCount()!=0 )
        {
            for (int i=0; i<logging_model_pick.rowCount(); i++)
            {
                ros::Duration(0.1);
                if ( !pick_name.compare( logging_model_go_to.data( logging_model_go_to.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
                if ( !pick_name.compare( logging_model_place.data( logging_model_place.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
                if ( !pick_name.compare( logging_model_pick.data( logging_model_pick.index( i ), 0 ).toString().toStdString() ) )
                {
                    return false;
                }
            }
            for ( int i = 0; i < pick_actions.size(); i++)
            {
                if ( compare( objects_, pick_actions[i].objects) )
                {
                    return false;
                }
            }
        }
        log_pick(pick_name);
        log_second_pick(pick_name);

        pick pck;
        pck.agents      = agents_;
        pck.name        = pick_name;
        pck.objects     = objects_;
        pck.description = description;
        pick_actions.push_back(pck);
        return true;
    }

    ROS_ERROR("empty line");
    ros::Duration(0.1);
    return true;
}

bool QNode::add_location(std::string location_name)
{
    if ( logging_model_location.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_location.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !location_name.compare( logging_model_location.data( logging_model_location.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
    }
    log_location(location_name);
    log_location_modify(location_name);

    location loc = return_position(base_frame, target_frame);
    go_to_location gt;
    gt.name     = location_name;
    gt.location_ = loc;
    gt.frame = base_frame;
    go_to_locations.push_back(gt);

    changed_locations.push_back(gt);

    return true;
}

bool QNode::add_location_copy(go_to_location new_loc)
{
    for ( int i = 0; i < go_to_locations.size(); i++)
    {
        if ( !new_loc.name.compare( go_to_locations[i].name ) )
        {
            return false;
        }
    }
    go_to_locations.push_back( new_loc );
    changed_locations.push_back( new_loc );
    log_location( new_loc.name );
    log_location_modify( new_loc.name );
    return true;
}

bool QNode::add_object_copy(object_type new_obj)
{
    for ( int i = 0; i < objects.size(); i++)
    {
        if ( !new_obj.type.compare( objects[i].type ) )
        {
            return false;
        }
    }
    objects.push_back( new_obj );
    //    changed_objects.push_back( new_obj );
    log_object( new_obj.type );
    log_object_modify( new_obj.type );
    return true;
}

bool QNode::add_slot_copy(manipulation_slot new_slot)
{
    for ( int i = 0; i < manipulation_slots.size(); i++)
    {
        if ( !new_slot.name.compare( manipulation_slots[i].name ) )
        {
            return false;
        }
    }
    manipulation_slots.push_back( new_slot );
    changed_slots.push_back( new_slot );
    log_slot( new_slot.name );
    log_slot_modify( new_slot.name );
    return true;
}

bool QNode::add_box_copy(box new_box)
{
    for ( int i = 0; i < boxes.size(); i++)
    {
        if ( !new_box.name.compare( boxes[i].name ) )
        {
            return false;
        }
    }
    boxes.push_back( new_box );
    changed_boxes.push_back( new_box );
    log_box( new_box.name );
    log_box_modify( new_box.name );
    return true;
}

location QNode::return_position( std::string base_frame, std::string target_frame )
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

go_to_action QNode::return_go_to_info(int ind)
{
    return go_to_actions[ind];
}

pick QNode::return_pick_info(int ind)
{
    return pick_actions[ind];
}

place QNode::return_place_info(int ind)
{
    return place_actions[ind];
}

go_to_location QNode::return_location_info(int ind)
{
    return go_to_locations[ind];
}

object_type QNode::return_object_info( int ind)
{
    return objects[ind];
}

box QNode::return_box_info( int ind)
{
    return boxes[ind];
}

manipulation_slot QNode::return_slot_info( int ind)
{
    return manipulation_slots[ind];
}

std::string QNode::return_location_list_text(int ind)
{
    return logging_model_components.data( logging_model_components.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_group_list_text(int ind)
{
    return logging_model_components.data( logging_model_components.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_object_list_text(int ind)
{
    return logging_model_components.data( logging_model_components.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_box_list_text(int ind)
{
    return logging_model_box.data( logging_model_box.index( ind ), 0 ).toString().toStdString();
}

double QNode::return_gripper_position()
{
    gripper_state = js_sub->getData();
    std::vector<double> state = gripper_state.position;
    return state[0];
}


void QNode::log_go_to(const std::string &msg)
{
    logging_model_go_to.insertRows(logging_model_go_to.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_go_to.setData(logging_model_go_to.index(logging_model_go_to.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_place(const std::string &msg)
{
    logging_model_place.insertRows(logging_model_place.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_place.setData(logging_model_place.index(logging_model_place.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_pick(const std::string &msg)
{
    logging_model_pick.insertRows(logging_model_pick.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_pick.setData(logging_model_pick.index(logging_model_pick.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_object(const std::string &msg)
{
    logging_model_object.insertRows(logging_model_object.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_object.setData(logging_model_object.index(logging_model_object.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::log_object_modify(const std::string &msg)
{
    logging_model_object_modify.insertRows(logging_model_object_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_object_modify.setData(logging_model_object_modify.index(logging_model_object_modify.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_slot( const std::string &msg)
{
    logging_model_slot.insertRows(logging_model_slot.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot.setData(logging_model_slot.index(logging_model_slot.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_slot_modify( const std::string &msg)
{
    logging_model_slot_modify.insertRows(logging_model_slot_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slot_modify.setData(logging_model_slot_modify.index(logging_model_slot_modify.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_box( const std::string &msg)
{
    logging_model_box.insertRows(logging_model_box.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box.setData(logging_model_box.index(logging_model_box.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_box_modify( const std::string &msg)
{
    logging_model_box_modify.insertRows(logging_model_box_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_box_modify.setData(logging_model_box_modify.index(logging_model_box_modify.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_group( const std::string &msg)
{
    logging_model_group.insertRows(logging_model_group.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_group.setData(logging_model_group.index(logging_model_group.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_location  ( const std::string &msg)
{
    logging_model_location.insertRows(logging_model_location.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location.setData(logging_model_location.index(logging_model_location.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_location_modify  ( const std::string &msg)
{
    logging_model_location_modify.insertRows(logging_model_location_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_modify.setData(logging_model_location_modify.index(logging_model_location_modify.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_components( const std::string &msg)
{
    logging_model_components.insertRows(logging_model_components.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_components.setData(logging_model_components.index(logging_model_components.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_info_action( const std::string &msg)
{
    logging_model_info_action.insertRows(logging_model_info_action.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_info_action.setData(logging_model_info_action.index(logging_model_info_action.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_second_go_to( const std::string &msg)
{
    logging_model_second_go_to.insertRows(logging_model_second_go_to.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_go_to.setData(logging_model_second_go_to.index(logging_model_second_go_to.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_second_place (const std::string &msg)
{
    logging_model_second_place.insertRows(logging_model_second_place.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_place.setData(logging_model_second_place.index(logging_model_second_place.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_second_pick  (const std::string &msg)
{
    logging_model_second_pick.insertRows(logging_model_second_pick.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_pick.setData(logging_model_second_pick.index(logging_model_second_pick.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_action_components(const std::string &msg)
{
    logging_model_action_components.insertRows(logging_model_action_components.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_action_components.setData(logging_model_action_components.index(logging_model_action_components.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_recipe (const std::string &msg)
{
    logging_model_recipe.insertRows(logging_model_recipe.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_recipe.setData(logging_model_recipe.index(logging_model_recipe.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::remove_go_to(int ind)
{
    go_to_actions.erase(go_to_actions.begin()+ind);
}

void QNode::remove_location(int ind)
{
    for ( int i = 0; i < changed_locations.size(); i++ )
    {
        if ( !go_to_locations[ind].name.compare( changed_locations[i].name ) )
        {
            changed_locations.erase( changed_locations.begin() + i );
        }
    }

    go_to_locations.erase(go_to_locations.begin()+ind);
}

void QNode::remove_place(int ind)
{
    place_actions.erase(place_actions.begin()+ind);
}

void QNode::remove_pick(int ind)
{
    pick_actions.erase(pick_actions.begin()+ind);
}

void QNode::remove_object(int ind)
{
    objects.erase(objects.begin()+ind);
}

void QNode::remove_slot(int ind)
{
    for ( int i = 0; i < changed_slots.size(); i++ )
    {
        if ( !manipulation_slots[ind].name.compare( changed_slots[i].name ) )
        {
            changed_slots.erase( changed_slots.begin() + i );
        }
    }

    manipulation_slots.erase(manipulation_slots.begin()+ind);
}

void QNode::remove_box(int ind)
{
    for ( int i = 0; i < changed_boxes.size(); i++ )
    {
        if ( !boxes[ind].name.compare( changed_boxes[i].name ) )
        {
            changed_boxes.erase( changed_boxes.begin() + i );
        }
    }

    boxes.erase(boxes.begin()+ind);
}

std::vector<int> QNode::remove_group(int ind)
{
    for ( int i = 0; i < changed_groups.size(); i++ )
    {
        if ( !groups.at(ind).compare( changed_groups.at(i) ) )
        {
            changed_groups.erase( changed_groups.begin() + i );
        }
    }

    std::vector<int> indexes;
    for ( int i = 0; i < manipulation_slots.size(); i++)
    {
        if ( ! manipulation_slots[i].group.compare( groups.at(ind) ) )
        {
            indexes.push_back(i);
        }
    }
    for ( int i = indexes.size()-1; i >= 0; i--)
    {
        for ( int j = 0; j < changed_slots.size(); j++ )
        {
            if ( !changed_slots[j].name.compare( manipulation_slots[indexes[i]].name ))
            {
                changed_slots.erase( changed_slots.begin()+j );
            }
        }
        manipulation_slots.erase(manipulation_slots.begin()+indexes[i]);
    }
    groups.erase(groups.begin()+ind);
    return indexes;
}

void QNode::active_configuration(std::string config)
{
    start_ctrl_req.request.start_configuration = config;
    start_ctrl_req.request.strictness = 1;

    set_ctrl_srv.waitForExistence();

    if ( !set_ctrl_srv.call(start_ctrl_req) )
    {
        ROS_ERROR("Unable to call %s service to set controller %s",set_ctrl_srv.getService().c_str(),start_ctrl_req.request.start_configuration.c_str());
        return;
    }

    if (!start_ctrl_req.response.ok)
    {
        ROS_ERROR("Error on service %s response", set_ctrl_srv.getService().c_str());
        return;
    }
    ROS_INFO("Controller %s started.",start_ctrl_req.request.start_configuration.c_str());
}

void QNode::move_gripper( std::string str )
{
    ROS_INFO("Gripper are moving");
    gripper_req.request.skill_name = " ";
    gripper_req.request.tool_id = "gripper_fake";
    gripper_req.request.property_id = str;

    gripper_srv.waitForExistence();
    if (!gripper_srv.call(gripper_req))
    {
        ROS_ERROR("Unable to move gripper t %s state",gripper_req.request.property_id.c_str());
        return;
    }

    gripper_state = js_sub->getData();
}

std::string QNode::get_xml_max_number_string(int value )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append("max_objects");
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(init_int);
    std::string str = std::to_string(value);
    xml_body.append(str);
    xml_body.append(end_int);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_double_string( double value )
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_double);
    std::string str = std::to_string(value);
    std::replace( str.begin(), str.end(), ',', '.');
    xml_body.append(str);
    xml_body.append(end_double);
    xml_body.append(end_value);

    return xml_body;
}

std::string QNode::get_xml_double_string_with_name(std::string param_name, double value)
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append(param_name);
    xml_body.append(end_name);

    xml_body.append( get_xml_double_string(value) );

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_string_param( std::string param_name, std::string value )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append(param_name);
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(value);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_position_string( std::string name_pos, position pos )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append(name_pos);
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);

    xml_body.append(get_xml_double_string(pos.origin_x));
    xml_body.append(get_xml_double_string(pos.origin_y));
    xml_body.append(get_xml_double_string(pos.origin_z));

    xml_body.append(end_data);
    xml_body.append(end_array);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_quaternion_string( quaternion quat )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append("quaternion");
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);

    xml_body.append(get_xml_double_string(quat.rotation_x));
    xml_body.append(get_xml_double_string(quat.rotation_y));
    xml_body.append(get_xml_double_string(quat.rotation_z));
    xml_body.append(get_xml_double_string(quat.rotation_w));

    xml_body.append(end_data);
    xml_body.append(end_array);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_group_string( std::string name, std::vector<std::string> string_group )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append(name);
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);


    for ( int i = 0; i < string_group.size(); i++)
    {
        xml_body.append(init_value);
        xml_body.append(init_string);
        xml_body.append(string_group[i]);
        xml_body.append(end_string);
        xml_body.append(end_value);
    }

    xml_body.append(end_data);
    xml_body.append(end_array);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

std::string QNode::get_xml_object_grasp_string( int index, int index2 )
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("tool", objects[index].tool[index2]));
    xml_body.append(get_xml_position_string("position", objects[index].grasp[index2].pos));
    xml_body.append(get_xml_quaternion_string(objects[index].grasp[index2].quat));
    xml_body.append(get_xml_position_string("approach_distance", objects[index].approach[index2]));
    xml_body.append(get_xml_position_string("leave_distance", objects[index].leave[index2]));
    xml_body.append(get_xml_double_string_with_name("pre_gripper_position", objects[index].pre_gripper_position[index2]));
    xml_body.append(get_xml_double_string_with_name("post_gripper_position", objects[index].post_gripper_position[index2]));
    xml_body.append(get_xml_double_string_with_name("gripper_force", objects[index].gripper_force[index2]));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    return xml_body;
}

std::string QNode::get_xml_object_grasp_poses_string( int index )
{
    std::string xml_body;

    xml_body.append(init_member);

    xml_body.append(init_name);
    xml_body.append("grasp_poses");
    xml_body.append(end_name);

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);

    for ( int i = 0; i < objects[index].grasp.size(); i++ )
    {
        xml_body.append( get_xml_object_grasp_string(index, i) );
    }

    xml_body.append(end_data);
    xml_body.append(end_array);
    xml_body.append(end_value);

    xml_body.append(end_member);

    return xml_body;
}

XmlRpc::XmlRpcValue QNode::get_go_to_location_param(int index)
{
    //    /go_to_location
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", go_to_locations[index].name));
    xml_body.append(get_xml_string_param("frame", go_to_locations[index].frame));
    xml_body.append(get_xml_position_string("position", go_to_locations[index].location_.pos));
    xml_body.append(get_xml_quaternion_string(go_to_locations[index].location_.quat));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_box_param(int index)
{
    //    /inbound/boxes
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", boxes[index].name));
    xml_body.append(get_xml_string_param("frame", boxes[index].frame));
    xml_body.append(get_xml_position_string("position", boxes[index].location_.pos));
    xml_body.append(get_xml_position_string("approach_distance", boxes[index].approach));
    xml_body.append(get_xml_position_string("leave_distance", boxes[index].leave));
    xml_body.append(get_xml_quaternion_string(boxes[index].location_.quat));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::get_object_grasp_param(int index, int index2)
{
    //    /nameObj/grasp_poses
    std::string xml_body = get_xml_object_grasp_string( index, index2 );

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);
    return param;
}

XmlRpc::XmlRpcValue QNode::get_object_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append( get_xml_string_param("type", objects[index].type) );
    xml_body.append( get_xml_object_grasp_poses_string(index) );

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_group_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", groups[index]));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_slot_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", manipulation_slots[index].name));
    xml_body.append(get_xml_string_param("frame", manipulation_slots[index].frame));
    xml_body.append(get_xml_string_param("slots_group", manipulation_slots[index].group));
    xml_body.append(get_xml_max_number_string(manipulation_slots[index].max_objects));
    xml_body.append(get_xml_position_string("position", manipulation_slots[index].location_.pos));
    xml_body.append(get_xml_quaternion_string( manipulation_slots[index].location_.quat));
    xml_body.append(get_xml_position_string("approach_distance", manipulation_slots[index].approach));
    xml_body.append(get_xml_position_string("leave_distance", manipulation_slots[index].leave));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_go_to_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", go_to_actions[index].name));
    xml_body.append(get_xml_string_param("description", go_to_actions[index].description));
    xml_body.append(get_xml_string_param("type", "goto"));
    xml_body.append(get_xml_group_string("agent", go_to_actions[index].agents));
    xml_body.append(get_xml_group_string("goal", go_to_actions[index].locations));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_pick_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", pick_actions[index].name));
    xml_body.append(get_xml_string_param("description", pick_actions[index].description));
    xml_body.append(get_xml_string_param("type", "pick"));
    xml_body.append(get_xml_group_string("agent", pick_actions[index].agents));
    xml_body.append(get_xml_group_string("goal", pick_actions[index].objects));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_place_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", place_actions[index].name));
    xml_body.append(get_xml_string_param("description", place_actions[index].description));
    xml_body.append(get_xml_string_param("type", "place"));
    xml_body.append(get_xml_group_string("agent", place_actions[index].agents));
    xml_body.append(get_xml_group_string("goal", place_actions[index].groups));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

void QNode::set_target_frame( int ind )
{
    if ( robot_name_params.size() != 0 )
    {
        n.getParam(robot_name_params[ind],target_frame);
    }
}

void QNode::check_objects_param()
{

    bool presence = false;

    for ( int i = 0; i < objects_compare.size(); i++ )
    {
        for ( int j = 0; j < objects.size(); j++)
        {
            if ( objects[j].type == objects_compare[i].type )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            objects.push_back( objects_compare[i] );
        }
    }

    objects_compare = objects;

    return;
}

void QNode::check_other_param()
{
    for ( int i = 0; i < go_to_locations_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < go_to_locations.size(); j++)
        {
            if ( go_to_locations[j].name == go_to_locations_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            go_to_locations.push_back( go_to_locations_compare[i] );
        }
    }

    for ( int i = 0; i < changed_locations.size(); i++ )
    {
        for ( int j = 0; j < go_to_locations_compare.size(); j++ )
        {
            if ( !changed_locations[i].name.compare( go_to_locations_compare[j].name ) )
            {
                locations_to_remove.push_back( go_to_locations_compare[j] );
            }
        }
    }

    go_to_locations_compare = go_to_locations;

    for ( int i = 0; i < go_to_actions_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < go_to_actions.size(); j++)
        {
            if ( go_to_actions[j].name == go_to_actions_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            go_to_actions.push_back( go_to_actions_compare[i] );
        }
    }

    go_to_actions_compare = go_to_actions;

    for ( int i = 0; i < place_actions_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < place_actions.size(); j++)
        {
            if ( place_actions[j].name == place_actions_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            place_actions.push_back( place_actions_compare[i] );
        }
    }

    place_actions_compare = place_actions;

    for ( int i = 0; i < pick_actions_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < pick_actions.size(); j++)
        {
            if ( pick_actions[j].name == pick_actions_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            pick_actions.push_back( pick_actions_compare[i] );
        }
    }
    pick_actions_compare = pick_actions;

    for ( int i = 0; i < slots_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < manipulation_slots.size(); j++)
        {
            if ( manipulation_slots[j].name == slots_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            manipulation_slots.push_back( slots_compare[i] );
        }
    }

    for ( int i = 0; i < changed_slots.size(); i++ )
    {
        for ( int j = 0; j < slots_compare.size(); j++ )
        {
            if ( !changed_slots[i].name.compare( slots_compare[j].name ) )
            {
                slots_to_remove.push_back( slots_compare[j] );
            }
        }
    }

    slots_compare = manipulation_slots;

    for ( int i = 0; i < groups_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < groups.size(); j++)
        {
            if ( groups[j] == groups_compare[i] )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            groups.push_back( groups_compare[i] );
        }
    }

    for ( int i = 0; i < changed_groups.size(); i++ )
    {
        for ( int j = 0; j < groups_compare.size(); j++ )
        {
            if ( !changed_groups.at(i).compare( groups_compare.at(j) ) )
            {
                groups_to_remove.push_back( groups_compare.at(j) );
            }
        }
    }

    groups_compare = groups;

    for ( int i = 0; i < boxes_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < boxes.size(); j++)
        {
            if ( boxes[j].name == boxes_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            boxes.push_back( boxes_compare[i] );
        }
    }

    for ( int i = 0; i < changed_boxes.size(); i++ )
    {
        for ( int j = 0; j < boxes_compare.size(); j++ )
        {
            if ( !changed_boxes[i].name.compare( boxes_compare[j].name ) )
            {
                boxes_to_remove.push_back( boxes_compare[j] );
            }
        }
    }

    boxes_compare = boxes;
}

void QNode::check_recipes_param()
{
    for ( int i = 0; i < recipes_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < recipes.size(); j++)
        {
            if ( recipes[j].name == recipes_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            recipes.push_back( recipes_compare[i] );
        }
    }

    recipes_compare = recipes;
}

bool QNode::save_components()
{
    XmlRpc::XmlRpcValue param;

    check_objects_param();
    check_other_param();

    for ( int i = 0; i < go_to_locations.size(); i++)
    {
        param[i] = get_go_to_location_param(i);
    }
    n.setParam("/go_to_location", param);
    param.clear();

    for ( int i = 0; i < boxes.size(); i++)
    {
        param[i] = get_box_param(i);
    }
    n.setParam("/inbound/boxes", param);
    param.clear();

    for ( int i = 0; i < groups.size(); i++)
    {
        param[i] = get_group_param(i);
    }
    n.setParam("/outbound/slots_group", param);
    param.clear();

    for ( int i = 0; i < manipulation_slots.size(); i++)
    {
        param[i] = get_slot_param( i );
    }
    n.setParam("/outbound/slots", param);
    param.clear();

    for ( int i = 0; i < objects.size(); i++ )
    {
        param [i] = get_object_param(i);
    }
    n.setParam("/manipulation_object_types", param);
    param.clear();

    ros::ServiceClient client = n.serviceClient<manipulation_interface_mongo::SaveParam>("/save_components_params_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    client.call(srv);

    write_param(1);

    if ( tc_finito )
    {
        t_component.join();
        tc_finito = false;
    }

    if ( t_component.joinable() )
    {
        ROS_ERROR("The previous thread did not finish");
        return false;
    }
    else
    {

        t_component = std::thread(&QNode::load_new_params_in_manipulation, this,
                                  changed_locations,
                                  changed_slots,
                                  changed_boxes,
                                  changed_groups,
                                  locations_to_remove,
                                  slots_to_remove,
                                  boxes_to_remove,
                                  groups_to_remove);

        changed_locations.clear();
        changed_slots.clear();
        changed_boxes.clear();
        changed_groups.clear();
        locations_to_remove.clear();
        slots_to_remove.clear();
        boxes_to_remove.clear();
        groups_to_remove.clear();
    }

    return true;
}

void QNode::load_new_params_in_manipulation(std::vector<go_to_location>    changed_locations_,
                                            std::vector<manipulation_slot> changed_slots_,
                                            std::vector<box>               changed_boxes_,
                                            std::vector<std::string>       changed_groups_,
                                            std::vector<go_to_location>    locations_to_remove_,
                                            std::vector<manipulation_slot> slots_to_remove_,
                                            std::vector<box>               boxes_to_remove_,
                                            std::vector<std::string>       groups_to_remove_)
{
    ROS_WARN("Inizio caricamento componenti nel manipulator");

    manipulation_msgs::RemoveLocations remove_location_srv;
    ROS_WARN("Locations da rimuovere: %zu", locations_to_remove_.size());
    for ( int i = 0; i < locations_to_remove_.size(); i++ )
    {
        remove_location_srv.request.location_names.push_back(locations_to_remove_[i].name);
    }
    if ( remove_location_srv.request.location_names.size() != 0 )
    {
        remove_locations_client_.call(remove_location_srv);
    }

    manipulation_msgs::RemoveSlots remove_slots_srv;
    ROS_WARN("Slot da rimuovere: %zu", slots_to_remove_.size());
    for ( int i = 0; i < slots_to_remove_.size(); i++ )
    {
        remove_slots_srv.request.slots_names.push_back(slots_to_remove_[i].name);
    }
    if ( remove_slots_srv.request.slots_names.size() != 0 )
    {
        remove_slots_client_.call(remove_slots_srv);
    }

    manipulation_msgs::RemoveBoxes remove_boxes_srv;
    ROS_WARN("Box da rimuovere: %zu", boxes_to_remove_.size());
    for ( int i = 0; i < boxes_to_remove_.size(); i++ )
    {
        remove_boxes_srv.request.box_names.push_back(boxes_to_remove_[i].name);
    }
    if ( remove_boxes_srv.request.box_names.size() != 0 )
    {
        remove_boxes_client_.call(remove_boxes_srv);
    }

    manipulation_msgs::RemoveSlotsGroup remove_groups_srv;
    ROS_WARN("Gruppi da rimuovere: %zu", groups_to_remove_.size());
    for ( int i = 0; i < groups_to_remove_.size(); i++ )
    {
        remove_groups_srv.request.slots_group_names.push_back(groups_to_remove_.at(i));
    }
    if ( remove_groups_srv.request.slots_group_names.size() != 0 )
    {
        remove_slots_group_client_.call(remove_groups_srv);
    }

    manipulation_msgs::AddLocations add_locations_srv;
    ROS_WARN("Locations da aggiungere: %zu", changed_locations_.size());
    for ( int i = 0; i < changed_locations_.size(); i++ )
    {
        manipulation_msgs::Location location_;
        location_.name               = changed_locations_[i].name;
        location_.frame              = changed_locations_[i].frame;
        location_.pose.position.x    = changed_locations_[i].location_.pos.origin_x;
        location_.pose.position.y    = changed_locations_[i].location_.pos.origin_y;
        location_.pose.position.z    = changed_locations_[i].location_.pos.origin_z;
        location_.pose.orientation.w = changed_locations_[i].location_.quat.rotation_w;
        location_.pose.orientation.x = changed_locations_[i].location_.quat.rotation_x;
        location_.pose.orientation.y = changed_locations_[i].location_.quat.rotation_y;
        location_.pose.orientation.z = changed_locations_[i].location_.quat.rotation_z;

        add_locations_srv.request.locations.push_back(location_);
    }
    if ( add_locations_srv.request.locations.size() != 0 )
    {
        add_locations_client_.call(add_locations_srv);
    }

    manipulation_msgs::AddSlotsGroup add_groups_srv;
    ROS_WARN("Gruppi da aggiungere: %zu", changed_groups_.size());
    for ( int i = 0; i < changed_groups_.size(); i++ )
    {
        manipulation_msgs::SlotsGroup group_srv;
        group_srv.name = changed_groups_.at(i);

        add_groups_srv.request.add_slots_groups.push_back(group_srv);
    }
    if ( add_groups_srv.request.add_slots_groups.size() != 0 )
    {
        add_slots_group_client_.call(add_groups_srv);
    }

    ROS_WARN("Slot da aggiungere: %zu", changed_slots_.size());
    for ( int i = 0; i < changed_slots_.size(); i++ )
    {
        manipulation_msgs::AddSlots add_slots_srv;
        std::vector<manipulation_msgs::Slot> slot_vct;
        manipulation_msgs::Slot slot_;
        slot_.name                                       = changed_slots_[i].name;
        slot_.slot_size                                  = changed_slots_[i].max_objects;
        slot_.location.name                              = slot_.name;
        slot_.location.frame                             = changed_slots_[i].frame;
        slot_.location.pose.position.x                   = changed_slots_[i].location_.pos.origin_x;
        slot_.location.pose.position.y                   = changed_slots_[i].location_.pos.origin_y;
        slot_.location.pose.position.z                   = changed_slots_[i].location_.pos.origin_z;
        slot_.location.pose.orientation.w                = changed_slots_[i].location_.quat.rotation_w;
        slot_.location.pose.orientation.x                = changed_slots_[i].location_.quat.rotation_x;
        slot_.location.pose.orientation.y                = changed_slots_[i].location_.quat.rotation_y;
        slot_.location.pose.orientation.z                = changed_slots_[i].location_.quat.rotation_z;
        slot_.location.approach_relative_pose.position.x = changed_slots_[i].approach.origin_x;
        slot_.location.approach_relative_pose.position.y = changed_slots_[i].approach.origin_y;
        slot_.location.approach_relative_pose.position.z = changed_slots_[i].approach.origin_z;
        slot_.location.leave_relative_pose.position.x    = changed_slots_[i].leave.origin_x;
        slot_.location.leave_relative_pose.position.y    = changed_slots_[i].leave.origin_y;
        slot_.location.leave_relative_pose.position.z    = changed_slots_[i].leave.origin_z;

        slot_vct.push_back(slot_);

        add_slots_srv.request.slots_group_name = changed_slots_[i].group;
        add_slots_srv.request.add_slots = slot_vct;

        if (!add_slots_client_.call(add_slots_srv))
        {
          ROS_WARN("Can't add slot to the location manager.");
        }

    }
//    for ( int i = 0; i < changed_groups.size(); i++ )
//    {
//        for ( int j = 0; j < changed_slots_.size(); j++ )
//        {
//            if ( !changed_slots_[j].group.compare( changed_groups.at(i) ) )
//            {
//                manipulation_msgs::Slot slot_;
//                slot_.name                                       = changed_slots_[j].name;
//                slot_.slot_size                                  = changed_slots_[j].max_objects;
//                slot_.location.name                              = slot_.name;
//                slot_.location.frame                             = changed_slots_[j].frame;
//                slot_.location.pose.position.x                   = changed_slots_[j].location_.pos.origin_x;
//                slot_.location.pose.position.y                   = changed_slots_[j].location_.pos.origin_y;
//                slot_.location.pose.position.z                   = changed_slots_[j].location_.pos.origin_z;
//                slot_.location.pose.orientation.w                = changed_slots_[j].location_.quat.rotation_w;
//                slot_.location.pose.orientation.x                = changed_slots_[j].location_.quat.rotation_x;
//                slot_.location.pose.orientation.y                = changed_slots_[j].location_.quat.rotation_y;
//                slot_.location.pose.orientation.z                = changed_slots_[j].location_.quat.rotation_z;
//                slot_.location.approach_relative_pose.position.x = changed_slots_[j].approach.origin_x;
//                slot_.location.approach_relative_pose.position.y = changed_slots_[j].approach.origin_y;
//                slot_.location.approach_relative_pose.position.z = changed_slots_[j].approach.origin_z;
//                slot_.location.leave_relative_pose.position.x    = changed_slots_[j].leave.origin_x;
//                slot_.location.leave_relative_pose.position.y    = changed_slots_[j].leave.origin_y;
//                slot_.location.leave_relative_pose.position.z    = changed_slots_[j].leave.origin_z;

//                add_slots_srv.request.add_slots.push_back( slot_ );
//            }
//        }
//        add_slots_srv.request.slots_group_name = changed_groups_.at(i);

//        if ( add_slots_srv.request.add_slots.size() != 0 )
//        {
//            add_slots_client_.call( add_slots_srv );
//        }
//    }

    manipulation_msgs::AddBoxes add_boxes_srv;
    ROS_WARN("Box da aggiungere: %zu", changed_boxes_.size());
    for ( int i = 0; i < changed_boxes_.size(); i++ )
    {
        manipulation_msgs::Box box_;
        box_.name = changed_boxes_[i].name;
        box_.location.name = box_.name;
        box_.location.frame = changed_boxes_[i].frame;
        box_.location.name                              = box_.name;
        box_.location.frame                             = changed_boxes_[i].frame;
        box_.location.pose.position.x                   = changed_boxes_[i].location_.pos.origin_x;
        box_.location.pose.position.y                   = changed_boxes_[i].location_.pos.origin_y;
        box_.location.pose.position.z                   = changed_boxes_[i].location_.pos.origin_z;
        box_.location.pose.orientation.w                = changed_boxes_[i].location_.quat.rotation_w;
        box_.location.pose.orientation.x                = changed_boxes_[i].location_.quat.rotation_x;
        box_.location.pose.orientation.y                = changed_boxes_[i].location_.quat.rotation_y;
        box_.location.pose.orientation.z                = changed_boxes_[i].location_.quat.rotation_z;
        box_.location.approach_relative_pose.position.x = changed_boxes_[i].approach.origin_x;
        box_.location.approach_relative_pose.position.y = changed_boxes_[i].approach.origin_y;
        box_.location.approach_relative_pose.position.z = changed_boxes_[i].approach.origin_z;
        box_.location.leave_relative_pose.position.x    = changed_boxes_[i].leave.origin_x;
        box_.location.leave_relative_pose.position.y    = changed_boxes_[i].leave.origin_y;
        box_.location.leave_relative_pose.position.z    = changed_boxes_[i].leave.origin_z;

        add_boxes_srv.request.add_boxes.push_back( box_ );
    }
    if ( add_boxes_srv.request.add_boxes.size() != 0 )
    {
        add_boxes_client_.call(add_boxes_srv);
    }

    tc_finito = true;

    ROS_WARN("Finito caricamento dei componenti sul manipulator");
}

bool QNode::save_actions()
{

    XmlRpc::XmlRpcValue param;

    check_objects_param();
    check_other_param();

    for ( int i = 0; i < go_to_actions.size(); i++)
    {
        param[i] = get_go_to_param(i);
    }
    for ( int i = 0; i < pick_actions.size(); i++)
    {
        param[ i + go_to_actions.size() ] = get_pick_param(i);
    }
    for ( int i = 0; i < place_actions.size(); i++)
    {
        param[ i + go_to_actions.size() + pick_actions.size() ] = get_place_param(i);
    }
    n.setParam("/multi_skills/tasks", param);
    param.clear();

    ros::ServiceClient client = n.serviceClient<manipulation_interface_mongo::SaveParam>("/save_actions_param_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    client.call(srv);

    write_param(2);

    return true;
}

bool QNode::add_object(std::string object_name,
                       std::vector<position> object_approach,
                       std::vector<location> object_grasp,
                       std::vector<position> object_leave,
                       std::vector<std::string> object_tools,
                       std::vector<double> pre_gripper_position,
                       std::vector<double> post_gripper_position,
                       std::vector<double> gripper_force)
{
    if ( logging_model_object.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_object.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !object_name.compare( logging_model_object.data( logging_model_object.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
    }

    log_object(object_name);
    log_object_modify(object_name);

    object_type obj;
    obj.type     = object_name;
    obj.tool     = object_tools;
    obj.approach = object_approach;
    obj.grasp    = object_grasp;
    obj.leave    = object_leave;
    obj.pre_gripper_position  = pre_gripper_position;
    obj.post_gripper_position = post_gripper_position;
    obj.gripper_force         = gripper_force;
    objects.push_back(obj);
    return true;
}

bool QNode::add_slot(std::string slot_name, location slot_approach, location slot_final_pos, location slot_leave, std::string group_name, int max_number )
{
    if ( logging_model_slot.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_slot.rowCount(); i++)
        {
            if ( !slot_name.compare( logging_model_slot.data( logging_model_slot.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
    }
    if ( logging_model_group.rowCount()!=0 )
    {
        bool add = true;
        for ( int i=0; i<logging_model_group.rowCount(); i++)
        {
            if ( !group_name.compare( logging_model_group.data( logging_model_group.index( i ),0 ).toString().toStdString() ) )
            {
                add = false;
            }
        }
        if ( add )
        {
            log_group(group_name);
            groups.push_back(group_name);

            changed_groups.push_back(group_name);
        }
    }
    else
    {
        log_group(group_name);
        groups.push_back(group_name);

        changed_groups.push_back(group_name);
    }

    position approach;
    approach.origin_x = slot_approach.pos.origin_x-slot_final_pos.pos.origin_x;
    approach.origin_y = slot_approach.pos.origin_y-slot_final_pos.pos.origin_y;
    approach.origin_z = slot_approach.pos.origin_z-slot_final_pos.pos.origin_z;
    position leave;
    leave.origin_x = slot_leave.pos.origin_x-slot_final_pos.pos.origin_x;
    leave.origin_y = slot_leave.pos.origin_y-slot_final_pos.pos.origin_y;
    leave.origin_z = slot_leave.pos.origin_z-slot_final_pos.pos.origin_z;

    log_slot(slot_name);
    log_slot_modify(slot_name);
    manipulation_slot slt;
    slt.name           = slot_name;
    slt.group          = group_name;
    slt.approach       = approach;
    slt.leave          = leave;
    slt.location_      = slot_final_pos;
    slt.max_objects    = max_number;
    slt.frame          = base_frame;
    manipulation_slots.push_back(slt);

    changed_slots.push_back(slt);

    return true;
}

bool QNode::add_box(std::string box_name, location approach_position, location final_position, location leave_position)
{
    if ( logging_model_box.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_box.rowCount(); i++)
        {
            if ( !box_name.compare( logging_model_box.data( logging_model_box.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
    }

    position approach;
    approach.origin_x = approach_position.pos.origin_x-final_position.pos.origin_x;
    approach.origin_y = approach_position.pos.origin_y-final_position.pos.origin_y;
    approach.origin_z = approach_position.pos.origin_z-final_position.pos.origin_z;
    position leave;
    leave.origin_x = leave_position.pos.origin_x-final_position.pos.origin_x;
    leave.origin_y = leave_position.pos.origin_y-final_position.pos.origin_y;
    leave.origin_z = leave_position.pos.origin_z-final_position.pos.origin_z;
    log_box(box_name);
    log_box_modify(box_name);
    box bx;
    bx.name      = box_name;
    bx.location_ = final_position;
    bx.approach  = approach;
    bx.leave     = leave;
    bx.frame     = base_frame;
    boxes.push_back(bx);

    changed_boxes.push_back(bx);

    return true;
}

bool QNode::add_location_changes(int ind, go_to_location new_location)
{
    if ( !go_to_locations[ind].name.compare( new_location.name ) )
    {
        go_to_locations[ind] = new_location;
    }
    else
    {
        go_to_locations.push_back( new_location );
        log_location(new_location.name);
        log_location_modify(new_location.name);
    }
    return true;
}

bool QNode::add_slot_changes(int ind, manipulation_slot new_slot)
{
    if ( !manipulation_slots[ind].name.compare( new_slot.name ) )
    {
        manipulation_slots[ind] = new_slot;
    }
    else
    {
        manipulation_slots.push_back(new_slot);
        log_slot(new_slot.name);
        log_slot_modify(new_slot.name);
    }
    return true;
}

bool QNode::add_box_changes(int ind, box new_box)
{
    if ( !boxes[ind].name.compare( new_box.name ) )
    {
        boxes[ind] = new_box;
    }
    else
    {
        boxes.push_back( new_box );
        log_box(new_box.name);
        log_box_modify(new_box.name);
    }
    return true;
}

bool QNode::add_object_changes(int ind, object_type new_object)
{
    if ( !objects[ind].type.compare( new_object.type ) )
    {
        objects[ind] = new_object;
    }
    else
    {
        objects.push_back( new_object );
        log_object_modify(new_object.type);
        log_object(new_object.type);
    }
    return true;
}

void QNode::load_TF()
{
    tf::TransformListener listener;
    ros::Duration(0.3).sleep();
    listener.getFrameStrings(TFs);
}

void QNode::load_param( int ind )
{
    if (!n.getParamNames(param_names))
    {
        ROS_ERROR("Empty robot parameter");
        return;
    }

    if ( ind == 1 )
    {
        load_robots();

        set_target_frame(0);

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
    }
}

void QNode::load_robots()
{
    if (!n.getParamNames(param_names))
    {
        ROS_ERROR("Empty robot parameter");
        return;
    }

    robots.clear();
    robot_name_params.clear();
    for ( int i = 0; i < param_names.size(); i++)
    {
        std::size_t found  = param_names[i].find("/go_to_location_server/groups/");
        std::size_t found2 = param_names[i].find("/", 26);
        std::string str = param_names[i];
        if ( found != std::string::npos && found2 != std::string::npos )
        {
            robots.push_back( str.erase( 0, found2+1 ) );
            robot_name_params.push_back( param_names[i] );
        }
    }
}

void QNode::write_param(int ind)
{
    load_param( ind );

    if ( ind == 1)
    {
        for ( int i = 0; i < boxes.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box.rowCount(); j++)
            {
                if ( !boxes[i].name.compare(logging_model_box.data( logging_model_box.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_box(boxes[i].name);
            }
        }
        for ( int i = 0; i < boxes.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_box_modify.rowCount(); j++)
            {
                if ( !boxes[i].name.compare(logging_model_box_modify.data( logging_model_box_modify.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_box_modify(boxes[i].name);
            }
        }
        for ( int i = 0; i < objects.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object.rowCount(); j++)
            {
                if ( !objects[i].type.compare(logging_model_object.data( logging_model_object.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_object(objects[i].type);
            }
        }
        for ( int i = 0; i < objects.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_object_modify.rowCount(); j++)
            {
                if ( !objects[i].type.compare(logging_model_object_modify.data( logging_model_object_modify.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_object_modify(objects[i].type);
            }
        }
        for ( int i = 0; i < groups.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_group.rowCount(); j++)
            {
                if ( !groups[i].compare(logging_model_group.data( logging_model_group.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_group( groups[i] );
            }
        }
        for ( int i = 0; i < manipulation_slots.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot.rowCount(); j++)
            {
                if ( !manipulation_slots[i].name.compare(logging_model_slot.data( logging_model_slot.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_slot(manipulation_slots[i].name);
            }
        }
        for ( int i = 0; i < manipulation_slots.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_slot_modify.rowCount(); j++)
            {
                if ( !manipulation_slots[i].name.compare(logging_model_slot_modify.data( logging_model_slot_modify.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_slot_modify(manipulation_slots[i].name);
            }
        }
        for ( int i = 0; i < go_to_locations.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location.rowCount(); j++)
            {
                if ( !go_to_locations[i].name.compare(logging_model_location.data( logging_model_location.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_location(go_to_locations[i].name);
            }
        }
        for ( int i = 0; i < go_to_locations.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_location_modify.rowCount(); j++)
            {
                if ( !go_to_locations[i].name.compare(logging_model_location_modify.data( logging_model_location_modify.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_location_modify(go_to_locations[i].name);
            }
        }
    }
    else if ( ind == 2)
    {
        for ( int i = 0; i < go_to_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_go_to.rowCount(); j++)
            {
                if ( !go_to_actions[i].name.compare(logging_model_go_to.data( logging_model_go_to.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_go_to(go_to_actions[i].name);
            }
        }
        for ( int i = 0; i < go_to_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_go_to.rowCount(); j++)
            {
                if ( !go_to_actions[i].name.compare(logging_model_second_go_to.data( logging_model_second_go_to.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_second_go_to(go_to_actions[i].name);
            }
        }
        for ( int i = 0; i < place_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_place.rowCount(); j++)
            {
                if ( !place_actions[i].name.compare(logging_model_place.data( logging_model_place.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_place(place_actions[i].name);
            }
        }
        for ( int i = 0; i < place_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_place.rowCount(); j++)
            {
                if ( !place_actions[i].name.compare(logging_model_second_place.data( logging_model_second_place.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_second_place(place_actions[i].name);
            }
        }
        for ( int i = 0; i < pick_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_pick.rowCount(); j++)
            {
                if ( !pick_actions[i].name.compare(logging_model_pick.data( logging_model_pick.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_pick(pick_actions[i].name);
            }
        }
        for ( int i = 0; i < pick_actions.size(); i++)
        {
            bool presence = false;
            for ( int j = 0; j < logging_model_second_pick.rowCount(); j++)
            {
                if ( !pick_actions[i].name.compare(logging_model_second_pick.data( logging_model_second_pick.index( j ), 0 ).toString().toStdString() ) )
                {
                    presence = true;
                }
            }
            if ( !presence )
            {
                log_second_pick(pick_actions[i].name);
            }
        }
    }
}

void QNode::write_locations()
{
    if ( logging_model_components.rowCount() != 0 )
    {
        logging_model_components.removeRows( 0, logging_model_components.rowCount() );
    }
    for ( int i = 0; i < go_to_locations.size(); i++)
    {
        log_components(go_to_locations[i].name);
    }
}

void QNode::write_objects()
{
    if ( logging_model_components.rowCount() != 0 )
    {
        logging_model_components.removeRows( 0, logging_model_components.rowCount() );
    }
    for ( int i = 0; i < objects.size(); i++)
    {
        log_components(objects[i].type);
    }
}

void QNode::write_groups()
{
    if ( logging_model_components.rowCount() != 0 )
    {
        logging_model_components.removeRows( 0, logging_model_components.rowCount() );
    }
    for ( int i = 0; i < groups.size(); i++)
    {
        log_components(groups[i]);
    }
}

bool QNode::readBoxesFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n.getParam("/inbound/boxes",config))
    {
        ROS_ERROR("Unable to find /inboud/boxes");
        return false;
    }

    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The param is not a list of boxes" );
        return false;
    }


    ROS_INFO("There are %u boxes",config.size());

    box box_;

    for(size_t i=0; i < config.size(); i++)
    {
        XmlRpc::XmlRpcValue param = config[i];
        if( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%zu is not a struct", i);
            continue;
        }

        if( !param.hasMember("name") )
        {
            ROS_WARN("The element #%zu has not the field 'name'", i);
            continue;
        }
        box_.name = rosparam_utilities::toString(param["name"]);

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%zu has not the field 'frame'", i);
            continue;
        }
        box_.frame = rosparam_utilities::toString(param["frame"]);

        std::string what;
        std::vector<double> pos;
        if( !rosparam_utilities::getParam(param,"position",pos,what) )
        {
            ROS_WARN("The element #%zu has not the field 'position'", i);
            continue;
        }
        assert(pos.size()==3);

        box_.location_.pos.origin_x = pos.at(0);
        box_.location_.pos.origin_y = pos.at(1);
        box_.location_.pos.origin_z = pos.at(2);

        std::vector<double> quat;
        if( !rosparam_utilities::getParam(param,"quaternion",quat,what) )
        {
            ROS_WARN("The element #%zu has not the field 'quaternion'", i);
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
        for ( int j = 0; j < boxes.size(); j++)
        {
            if ( !box_.name.compare(boxes[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            boxes.push_back(box_);
            boxes_compare.push_back(box_);
        }
    }
    return true;
}

bool QNode::readObjectFromParam()
{
    XmlRpc::XmlRpcValue param;
    if ( !n.getParam("/manipulation_object_types", param) )
    {
        ROS_ERROR("Unable to find /manipulation_object_types");
        return false;
    }

    ROS_INFO("Reading of manipulation_object_types is finish ");

    for ( std::size_t i = 0; i < param.size(); i++ )
    {
        object_type object_;
        XmlRpc::XmlRpcValue single_object = param[i];

        if( single_object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The object element #%zu is not a struct", i);
            continue;
        }

        if ( !single_object.hasMember("type") )
        {
            ROS_WARN("The object element #%zu has not the field 'type'", i);
            continue;
        }
        object_.type = rosparam_utilities::toString(single_object["type"]);

        XmlRpc::XmlRpcValue grasp_poses_;
        if ( !single_object.hasMember("grasp_poses") )
        {
            ROS_WARN("The object element #%zu has not the field 'grasp_poses'", i);
            continue;
        }
        grasp_poses_ = single_object["grasp_poses"];

        if ( grasp_poses_.getType() != XmlRpc::XmlRpcValue::TypeArray )
        {
            ROS_WARN("The field grasp_poses of object element %zu is not an array", i);
            continue;
        }

        for ( std::size_t j = 0; j < grasp_poses_.size(); j++ )
        {
            XmlRpc::XmlRpcValue single_grasp = grasp_poses_[j];

            if( single_grasp.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("The grasp element %zu of object element #%zu is not a struct", j, i);
                continue;
            }

            location loc;

            std::string what;
            std::vector<double> pos;
            if ( !rosparam_utilities::getParam(single_grasp,"position",pos, what) )
            {
                ROS_WARN("The grasp element %zu of object element #%zu has not the field 'position'", j, i);
                continue;
            }

            assert(pos.size()==3);

            loc.pos.origin_x = pos.at(0);
            loc.pos.origin_y = pos.at(1);
            loc.pos.origin_z = pos.at(2);

            std::vector<double> quat;
            if( !rosparam_utilities::getParam(single_grasp,"quaternion",quat,what) )
            {
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'quaternion'", j, i);
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
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'approach_distance'", j, i);
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
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'leave_distance'", j, i);
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
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'tool'", j, i);
                continue;
            }

            object_.tool.push_back( rosparam_utilities::toString(single_grasp["tool"]) );

            if ( !single_grasp.hasMember("pre_gripper_position"))
            {
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'pre_gripper_position'", j, i);
                continue;
            }

            object_.pre_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["pre_gripper_position"]) );

            if ( !single_grasp.hasMember("post_gripper_position"))
            {
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'post_gripper_position'", j, i);
                continue;
            }

            object_.post_gripper_position.push_back( rosparam_utilities::toDouble(single_grasp["post_gripper_position"]) );

            if ( !single_grasp.hasMember("gripper_force"))
            {
                ROS_WARN("The grasp element #%zu of object element #%zu has not the field 'gripper_force'", j, i);
                continue;
            }

            object_.gripper_force.push_back( rosparam_utilities::toDouble(single_grasp["gripper_force"]) );

        }
        objects.push_back( object_ );
    }

    return true;
}

bool QNode::readSlotsGroupFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n.getParam("/outbound/slots_group",config))
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
        XmlRpc::XmlRpcValue slot = config[i];
        if( slot.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("The element #%d is not a struct", i);
            continue;
        }

        if( !slot.hasMember("name") )
        {
            ROS_WARN("The element #%d has not the field 'name'", i);
            return false;
        }

        bool presence = false;
        for ( int j = 0; j < groups.size(); j++)
        {
            if ( !rosparam_utilities::toString(slot["name"]).compare( groups[j] ) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            groups.push_back( rosparam_utilities::toString(slot["name"]) );
            groups_compare.push_back( rosparam_utilities::toString(slot["name"]) );
        }
    }

    return true;
}

bool QNode::readSlotsFromParam()
{
    XmlRpc::XmlRpcValue config;
    if (!n.getParam("/outbound/slots",config))
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
        for ( int j = 0; j < manipulation_slots.size(); j++)
        {
            if ( !slot_.name.compare(manipulation_slots[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            manipulation_slots.push_back(slot_);
            slots_compare.push_back(slot_);
        }
    }

    return true;
}

bool QNode::readLocationsFromParam()
{
    XmlRpc::XmlRpcValue go_to_locations_param;
    if (!n.getParam("/go_to_location",go_to_locations_param))
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
        for ( int j = 0; j < go_to_locations.size(); j++)
        {
            if ( !go_to_.name.compare(go_to_locations[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            go_to_locations.push_back(go_to_);
            go_to_locations_compare.push_back(go_to_);
        }
    }
    return true;
}

bool QNode::readGotoPickAndPlaceFromParam()
{
    XmlRpc::XmlRpcValue config;

    if ( !n.getParam("/multi_skills/tasks",config) )
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
            for ( int j = 0; j < go_to_actions.size(); j++)
            {
                if ( !go_to_.name.compare(go_to_actions[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                go_to_actions.push_back(go_to_);
                go_to_actions_compare.push_back(go_to_);
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
            std::vector<std::string> groups_;
            if( !rosparam_utilities::getParam(param,"goal",groups_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_.groups = groups_;

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
            for ( int j = 0; j < place_actions.size(); j++)
            {
                if ( !place_.name.compare(place_actions[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                place_actions.push_back(place_);
                place_actions_compare.push_back(place_);
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
            std::vector<std::string> objects_;
            if( !rosparam_utilities::getParam(param,"goal",objects_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_.objects = objects_;

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
            for ( int j = 0; j < pick_actions.size(); j++)
            {
                if ( !pick_.name.compare(pick_actions[j].name) )
                {
                    presence = true;
                }
            }

            if ( !presence )
            {
                pick_actions.push_back(pick_);
                pick_actions_compare.push_back(pick_);
            }
        }
    }

    return true;
}

bool QNode::compare(std::vector<std::string> &v1, std::vector<std::string> &v2)
{
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    return v1 == v2;
}

}  // namespace manipulation_interface_gui
