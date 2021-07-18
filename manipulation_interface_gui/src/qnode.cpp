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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulation_interface_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}



bool QNode::init() {
    ros::init(init_argc,init_argv,"manipulation_interface_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

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
    load_param();

	return true;
}

void QNode::load_actions()
{
    logging_model_second_go_to.removeRows (0, logging_model_second_go_to.rowCount());
    logging_model_second_place.removeRows (0, logging_model_second_place.rowCount());
    logging_model_second_pick.removeRows  (0, logging_model_second_pick.rowCount());

    load_param();

    for ( int i = 0; i < go_to_actions.size(); i++ )
    {
        log_second_go_to( go_to_actions[i].name );
    }
    for ( int i = 0; i < pick_actions.size(); i++ )
    {
        log_second_pick( pick_actions[i].name );
    }
    for ( int i = 0; i < place_actions.size(); i++ )
    {
        log_second_place( place_actions[i].name );
    }
}

void QNode::add_object_type (int ind)
{
    if ( logging_model_second_objects.rowCount() != 0 )
    {
        logging_model_second_objects.removeRows(0,logging_model_second_objects.rowCount());
    }
    if ( ind != 1000)
    {
        for ( int i = 0; i < pick_actions[ind].objects.size(); i++)
        {
            log_second_objects( pick_actions[ind].objects[i] );
        }
    }
}

void QNode::add_slot_groups (int ind)
{
    if ( logging_model_second_slots.rowCount() != 0 )
    {
        logging_model_second_slots.removeRows(0,logging_model_second_slots.rowCount());
    }
    if ( ind != 1000)
    {
        for ( int i = 0; i < place_actions[ind].groups.size(); i++)
        {
            log_second_slots( place_actions[ind].groups[i] );
        }
    }
}

std::vector<std::string> QNode::load_recipe ( bool init, std::string name_recipe)
{
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue config;

    std::vector<std::string> recipes_names;

    if ( !n.getParam("/multi_skills/recipes", config) )
    {
        ROS_ERROR("Unable to find /multi_skills/recipes");
    }
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The recipes param is not an array" );
    }
    ROS_INFO("There are %d recipes",config.size());

    recipes.clear();

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
        std::vector<std::string> description;
        if( !rosparam_utilities::getParam(param,"recipe",description,what) )
        {
          ROS_WARN("The element #%zu has not the field 'description'", i);
          continue;
        }
        single_recipe.recipe_ = description;
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
        if ( init )
        {
            if ( !name_recipe.compare( single_recipe.name ) )
            {
                for ( int j = 0; j < description.size(); j++ )
                {
                    log_recipe( description[j] );
                }
            }
        }

    }

    return recipes_names;
}

bool QNode::save_recipe( std::string recipe_name )
{
    ros::NodeHandle n;
    XmlRpc::XmlRpcValue param;

    ROS_ERROR("in ingresso %zu", recipes.size());

    for ( int i = 0; i < recipes.size(); i++ )
    {
        if ( !recipe_name.compare( recipes[i].name ) )
        {
            recipes.erase( recipes.begin()+i );
        }
    }

    ROS_ERROR("dopo eliminazione %zu", recipes.size());

    recipe recipe_;
    recipe_.name = recipe_name;

    for ( int i = 0; i < logging_model_recipe.rowCount(); i++ )
    {
        recipe_.recipe_.push_back( logging_model_recipe.data( logging_model_recipe.index(i) ).toString().toStdString() );
    }

    recipes.push_back(recipe_);

    ROS_ERROR("dopo aggiunta %zu", recipes.size());

    for ( int i = 0; i < recipes.size(); i++)
    {
        param[i] = get_recipe_param(i);
    }
    n.setParam("/multi_skills/recipes", param);
    param.clear();

    ROS_ERROR("dopo pubblicazione %zu", recipes.size());

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

bool QNode::set_recipe()
{
    ros::NodeHandle n;
    std::vector<std::string> actions_names;

    for ( int i = 0; i < logging_model_recipe.rowCount(); i++ )
    {
        bool presence = false;

        for ( int j = 0; j < logging_model_second_go_to.rowCount(); j++ )
        {
            if ( !logging_model_second_go_to.data( logging_model_second_go_to.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                presence = true;
            }
        }
        for ( int j = 0; j < logging_model_second_place.rowCount(); j++ )
        {
            if ( !logging_model_second_place.data( logging_model_second_place.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                presence = true;
            }
        }
        for ( int j = 0; j < logging_model_second_pick.rowCount(); j++ )
        {
            if ( !logging_model_second_pick.data( logging_model_second_pick.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                presence = true;
            }
        }

        if ( presence )
        {
            actions_names.push_back( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() );
        }
        else
        {
            return false;
        }
    }
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_array);
    xml_body.append(init_data);


    for ( int i = 0; i < actions_names.size(); i++)
    {
        xml_body.append(init_value);
        xml_body.append(init_string);
        xml_body.append(actions_names[i]);
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

    n.setParam("/multi_skills/recipe", param);
    return true;
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

bool QNode::add_go_to(std::string go_to_name, std::vector<std::string> locations_)
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

    go_to gt;
    gt.name = go_to_name;
    gt.locations = locations_;
    go_to_actions.push_back(gt);

    return true;

}

bool QNode::add_place(std::string place_name, std::vector<std::string> groups_)
{
    if ( logging_model_place.rowCount()!=0 )
    {
        for (int i=0; i<logging_model_place.rowCount(); i++)
        {
            ros::Duration(0.1);
            if ( !place_name.compare( logging_model_place.data( logging_model_place.index( i ), 0 ).toString().toStdString() ) )
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
    place plc;
    plc.name   = place_name;
    plc.groups = groups_;
    place_actions.push_back(plc);

    return true;
}

bool QNode::add_pick(std::string position_name, std::vector<std::string> objects_)
{
    if ( !position_name.empty())
    {
        if ( logging_model_pick.rowCount()!=0 )
        {
//            ROS_ERROR("sei entrato nell if");
            for (int i=0; i<logging_model_pick.rowCount(); i++)
            {
//                ROS_ERROR("sei entrato nell for");
                ros::Duration(0.1);
                if ( !position_name.compare( logging_model_pick.data( logging_model_pick.index( i ), 0 ).toString().toStdString() ) )
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
        log_pick(position_name);
        pick pck;
        pck.name    = position_name;
        pck.objects = objects_;
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
        //            ROS_ERROR("sei entrato nell if");
        for (int i=0; i<logging_model_location.rowCount(); i++)
        {
            //                ROS_ERROR("sei entrato nell for");
            ros::Duration(0.1);
            if ( !location_name.compare( logging_model_location.data( logging_model_location.index( i ), 0 ).toString().toStdString() ) )
            {
                return false;
            }
        }
    }
    log_location(location_name);

    location loc = return_position(base_frame, target_frame);
    go_to_location gt;
    gt.name     = location_name;
    gt.location_ = loc;
    gt.frame = base_frame;
    go_to_locations.push_back(gt);

    return true;
}

bool QNode::add_objects_pick ( int ind )
{
    for ( int i = 0; i < pick_actions[ind].objects.size(); i++ )
    {
        log_obj_pick(pick_actions[ind].objects[i]);
    }
}

bool QNode::add_groups_place ( int ind )
{
    for ( int i = 0; i < place_actions[ind].groups.size(); i++ )
    {
        log_grp_place(place_actions[ind].groups[i]);
    }
}

bool QNode::add_locations_go_to ( int ind )
{
    for ( int i = 0; i < go_to_actions[ind].locations.size(); i++ )
    {
        log_location_goto(go_to_actions[ind].locations[i]);
    }
}

location QNode::return_position( std::string base_frame, std::string target_frame )
{
    tf::TransformListener listener;
    ros::Duration(0.3).sleep();
    tf::StampedTransform transform;
    location loc;
    try
    {
//        ROS_ERROR("The base frame is: %s. The target frame is: %s",base_frame.c_str(),target_frame.c_str());
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

slot QNode::return_slot_info( int ind)
{
    return manipulation_slots[ind];
}

std::string QNode::return_location_list_text(int ind)
{
    return logging_model_location.data( logging_model_location.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_group_list_text(int ind)
{
    return logging_model_group.data( logging_model_group.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_object_list_text(int ind)
{
    return logging_model_object.data( logging_model_object.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_obj_dist_list_text(int ind)
{
    return logging_model_obj_dist.data( logging_model_obj_dist.index( ind ), 0 ).toString().toStdString();
}

std::string QNode::return_box_list_text(int ind)
{
    return logging_model_box.data( logging_model_box.index( ind ), 0 ).toString().toStdString();
}

void QNode::log_go_to(const std::string &msg) {
    logging_model_go_to.insertRows(logging_model_go_to.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_go_to.setData(logging_model_go_to.index(logging_model_go_to.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_place(const std::string &msg) {
    logging_model_place.insertRows(logging_model_place.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_place.setData(logging_model_place.index(logging_model_place.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_pick(const std::string &msg) {
    logging_model_pick.insertRows(logging_model_pick.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_pick.setData(logging_model_pick.index(logging_model_pick.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_object(const std::string &msg) {
    logging_model_object.insertRows(logging_model_object.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_object.setData(logging_model_object.index(logging_model_object.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar

    logging_model_object_modify.insertRows(logging_model_object_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
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

    logging_model_slot_modify.insertRows(logging_model_slot_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
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

    logging_model_box_modify.insertRows(logging_model_box_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
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

void QNode::log_obj_dist( const std::string &msg)
{
    logging_model_obj_dist.insertRows(logging_model_obj_dist.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_obj_dist.setData(logging_model_obj_dist.index(logging_model_obj_dist.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_grp_place ( const std::string &msg)
{
    logging_model_grp_place.insertRows(logging_model_grp_place.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_grp_place.setData(logging_model_grp_place.index(logging_model_grp_place.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_obj_pick  ( const std::string &msg)
{
    logging_model_obj_pick.insertRows(logging_model_obj_pick.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_obj_pick.setData(logging_model_obj_pick.index(logging_model_obj_pick.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_location  ( const std::string &msg)
{
    logging_model_location.insertRows(logging_model_location.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location.setData(logging_model_location.index(logging_model_location.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar

    logging_model_location_modify.insertRows(logging_model_location_modify.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    logging_model_location_modify.setData(logging_model_location_modify.index(logging_model_location_modify.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_location_goto  ( const std::string &msg)
{
    logging_model_location_go_to.insertRows(logging_model_location_go_to.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_location_go_to.setData(logging_model_location_go_to.index(logging_model_location_go_to.rowCount()-1),new_row);
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

void QNode::log_second_objects(const std::string &msg)
{
    logging_model_second_objects.insertRows(logging_model_second_objects.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_objects.setData(logging_model_second_objects.index(logging_model_second_objects.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_second_slots (const std::string &msg)
{
    logging_model_second_slots.insertRows(logging_model_second_slots.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_second_slots.setData(logging_model_second_slots.index(logging_model_second_slots.rowCount()-1),new_row);
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
    go_to_actions.erase (go_to_actions.begin()+ind);
}

void QNode::remove_location(int ind)
{
    go_to_locations.erase (go_to_locations.begin()+ind);
}

void QNode::remove_place(int ind)
{
    place_actions.erase (place_actions.begin()+ind);
}

void QNode::remove_pick(int ind)
{
    pick_actions.erase (pick_actions.begin()+ind);
}

void QNode::remove_object(int ind)
{
    objects.erase      (objects.begin()+ind);
}

void QNode::remove_slot(int ind)
{
    manipulation_slots.erase           (manipulation_slots.begin()+ind);
}

void QNode::remove_box(int ind)
{
    boxes.erase     (boxes.begin()+ind);
}

std::vector<int> QNode::remove_group(int ind)
{
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
        manipulation_slots.erase(manipulation_slots.begin()+indexes[i]);
    }
    groups.erase(groups.begin()+ind);
    return indexes;
}

void QNode::active_manual_guidance( bool action )
{
    if ( action )
    {
        start_ctrl_req.request.start_configuration = "rot_xyz_manual_guidance";
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
    else
    {
        start_ctrl_req.request.start_configuration = "watch";
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
}

void QNode::close_gripper( bool action )
{
    if ( action )
    {
        ROS_INFO("Gripper are closing");
        gripper_req.request.skill_name = " ";
        gripper_req.request.tool_id = "gripper_fake";
        gripper_req.request.property_id = "close_min_force_min_vel";

        gripper_srv.waitForExistence();
        if (!gripper_srv.call(gripper_req))
        {
            ROS_ERROR("Unable to move gripper t %s state",gripper_req.request.property_id.c_str());
            return;
        }
    }
    else
    {
        ROS_INFO("Gripper are opening");
        gripper_req.request.skill_name = " ";
        gripper_req.request.tool_id = "gripper_fake";
        gripper_req.request.property_id = "open";

        gripper_srv.waitForExistence();
        if (!gripper_srv.call(gripper_req))
        {
            ROS_ERROR("Unable to move gripper t %s state",gripper_req.request.property_id.c_str());
            return;
        }
    }
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
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("tool", objects[index].tool[index2]));
    xml_body.append(get_xml_position_string("position", objects[index].grasp[index2].pos));
    xml_body.append(get_xml_quaternion_string(objects[index].grasp[index2].quat));
    xml_body.append(get_xml_position_string("approach_distance", objects[index].approach[index2]));

    xml_body.append(end_struct);
    xml_body.append(end_value);

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

    xml_body.append( get_xml_string_param("name", objects[index].name) );
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

XmlRpc::XmlRpcValue QNode::get_object_name_param(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", objects[index].name));

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
    xml_body.append(get_xml_group_string("objects", pick_actions[index].objects));

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
    xml_body.append(get_xml_group_string("groups", place_actions[index].groups));

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

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_go_to_param_(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", go_to_actions[index].name));
    xml_body.append(get_xml_string_param("description", go_to_actions[index].description));
    xml_body.append(get_xml_string_param("type", "goto"));
    xml_body.append(get_xml_string_param("agent", "robot"));
    xml_body.append(get_xml_group_string("goal", go_to_actions[index].locations));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_pick_param_(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", pick_actions[index].name));
    xml_body.append(get_xml_string_param("description", pick_actions[index].description));
    xml_body.append(get_xml_string_param("type", "pick"));
    xml_body.append(get_xml_string_param("agent", "robot"));
    xml_body.append(get_xml_group_string("goal", pick_actions[index].objects));

    xml_body.append(end_struct);
    xml_body.append(end_value);

    int offset = 0;
    int* offset_ptr = &offset;
    XmlRpc::XmlRpcValue param;
    param.fromXml(xml_body,offset_ptr);

    return param;
}

XmlRpc::XmlRpcValue QNode::get_place_param_(int index)
{
    std::string xml_body;

    xml_body.append(init_value);
    xml_body.append(init_struct);

    xml_body.append(get_xml_string_param("name", place_actions[index].name));
    xml_body.append(get_xml_string_param("description", place_actions[index].description));
    xml_body.append(get_xml_string_param("type", "place"));
    xml_body.append(get_xml_string_param("agent", "robot"));
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
    ros::NodeHandle n;
    if ( robot_name_params.size() != 0 )
    {
        n.getParam(robot_name_params[ind],target_frame);
    }
}

void QNode::check_objects_param()
{
    for ( int i = 0; i < objects_compare.size(); i++ )
    {
        bool presence = false;
        for ( int j = 0; j < objects.size(); j++)
        {
            if ( objects[j].name == objects_compare[i].name )
            {
                presence = true;
            }
        }
        if ( presence == false )
        {
            objects.push_back( objects_compare[i] );
        }
    }
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
}

bool QNode::save_components()
{
    ros::NodeHandle n;

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

    for ( int i = 0; i < objects.size(); i++)
    {
        for ( int j = 0; j < objects[i].grasp.size(); j++)
        {
            param[j] = get_object_grasp_param( i, j );
        }
        std::string str;
        str.append("/manipulation_objects/");
        str.append(objects[i].name);
        str.append("/grasp_poses");
        n.setParam( str, param);
        param.clear();
    }

    for ( int i = 0; i < objects.size(); i++ )
    {
        param [i] = get_object_param(i);
    }
    n.setParam("/manipulation/objects", param);
    param.clear();

    for ( int i = 0; i < go_to_actions.size(); i++)
    {
        param[i] = get_go_to_param_(i);
    }
    for ( int i = 0; i < pick_actions.size(); i++)
    {
        param[ i + go_to_actions.size() ] = get_pick_param_(i);
    }
    for ( int i = 0; i < place_actions.size(); i++)
    {
        param[ i + go_to_actions.size() + pick_actions.size() ] = get_place_param_(i);
    }
    n.setParam("/multi_skills/tasks", param);
    param.clear();

    ros::ServiceClient client = n.serviceClient<manipulation_interface_mongo::SaveParam>("/save_components_params_on_mongo");
    manipulation_interface_mongo::SaveParam srv;
    client.call(srv);
    return true;
}

bool QNode::save_object(std::string object_name, std::vector<position> object_approach, std::vector<location> object_grasp, std::vector<std::string> object_tools)
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
    object_type obj;
    obj.name     = object_name;
    obj.tool     = object_tools;
    obj.approach = object_approach;
    obj.grasp    = object_grasp;
    objects.push_back(obj);

    return true;
}

bool QNode::save_slot(std::string slot_name, location slot_approach, location slot_final_pos, std::string group_name, int max_number )
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
        }
    }
    else
    {
        log_group(group_name);
        groups.push_back(group_name);
    }
    position approach;
    approach.origin_x = slot_approach.pos.origin_x-slot_final_pos.pos.origin_x;
    approach.origin_y = slot_approach.pos.origin_y-slot_final_pos.pos.origin_y;
    approach.origin_z = slot_approach.pos.origin_z-slot_final_pos.pos.origin_z;

    log_slot(slot_name);
    slot slt;
    slt.name           = slot_name;
    slt.group          = group_name;
    slt.approach       = approach;
    slt.location_      = slot_final_pos;
    slt.max_objects    = max_number;
    slt.frame          = base_frame;
    manipulation_slots.push_back(slt);
    return true;
}

bool QNode::save_box(std::string box_name, location approach_position, location final_position)
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
    log_box(box_name);
    box bx;
    bx.name      = box_name;
    bx.location_ = final_position;
    bx.approach  = approach;
    bx.frame     = base_frame;
    boxes.push_back(bx);
    return true;
}

bool QNode::save_location_changes(int ind, location new_location)
{
    go_to_locations[ind].location_ = new_location;
    return save_components();
}

bool QNode::save_slot_changes(int ind, slot new_slot)
{
    manipulation_slots[ind].location_   = new_slot.location_;
    manipulation_slots[ind].approach    = new_slot.approach;
    manipulation_slots[ind].max_objects = new_slot.max_objects;
    return save_components();
}

bool QNode::save_box_changes(int ind, box new_box)
{
    boxes[ind].approach  = new_box.approach;
    boxes[ind].location_ = new_box.location_;
    return save_components();
}

bool QNode::save_object_changes(int ind, int ind2, location new_loc, position new_appr)
{
    objects[ind].approach[ind2] = new_appr;
    objects[ind].grasp[ind2]    = new_loc;
    return save_components();
}

void QNode::load_TF()
{
    tf::TransformListener listener;
    ros::Duration(0.3).sleep();
    listener.getFrameStrings(TFs);
}

void QNode::load_param()
{
    ros::NodeHandle n;

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
    set_target_frame(0);

    readBoxesFromParam();
    readObjectFromParam();
    readSlotsGroupFromParam();
    readSlotsFromParam();
    readLocationsFromParam();
    readGotoPickAndPlaceFromParam();
}

void QNode::write_param()
{
    load_param();

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
    for ( int i = 0; i < objects.size(); i++)
    {
        bool presence = false;
        for ( int j = 0; j < logging_model_object.rowCount(); j++)
        {
            if ( !objects[i].name.compare(logging_model_object.data( logging_model_object.index( j ), 0 ).toString().toStdString() ) )
            {
                presence = true;
            }
        }
        if ( !presence )
        {
            log_object(objects[i].name);
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
}

bool QNode::readBoxesFromParam()
{
    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue config;
    if (!nh_.getParam("/inbound/boxes",config))
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
        ROS_INFO("Found box named: %s",box_.name.c_str());

        if( !param.hasMember("frame") )
        {
            ROS_WARN("The element #%zu has not the field 'frame'", i);
            continue;
        }
        box_.frame = rosparam_utilities::toString(param["frame"]);
        ROS_INFO("Found box frame name: %s",box_.frame.c_str());

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
    ros::NodeHandle nh_;

    std::vector<std::string> objects_param, objects_names;
    for ( int i = 0; i < param_names.size(); i++)
    {
        std::size_t found  = param_names[i].find("/manipulation_objects/");
        if ( found != std::string::npos )
        {
            found = param_names[i].find("grasp_poses");
            if ( found != std::string::npos )
            {
                std::size_t found2  = param_names[i].find( "/", 5 );
                std::size_t found3  = param_names[i].find( "/", found2+1 );
                std::string str = param_names[i];
                str.erase( found3, str.size() );
                objects_names.push_back( str.erase( 0, found2+1 ) );
                objects_param.push_back( param_names[i] );
            }
        }
    }

    ROS_INFO("There are %d objects",objects_param.size());

    for ( int i = 0; i < objects_param.size(); i++ )
    {
        XmlRpc::XmlRpcValue config;
        if (!nh_.getParam(objects_param[i],config))
        {
            ROS_ERROR("Unable to find %s",objects_param[i].c_str());
            return false;
        }

        if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("The param is not a list of object information" );
            return false;
        }

        object_type object_;
        object_.name = objects_names[i];

        for(size_t i=0; i < config.size(); i++)
        {
            XmlRpc::XmlRpcValue object = config[i];
            if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("The element #%zu is not a struct", i);
                continue;
            }
            if( !object.hasMember("tool") )
            {
                ROS_WARN("The element #%zu has not the field 'tool'", i);
                continue;
            }
            object_.tool.push_back( rosparam_utilities::toString(object["tool"]) );

            location loc;

            std::string what;
            std::vector<double> pos;
            if( !rosparam_utilities::getParam(object,"position",pos, what) )
            {
                ROS_WARN("Pose has not the field 'position'");
                continue;
            }

            assert(pos.size()==3);

            loc.pos.origin_x = pos.at(0);
            loc.pos.origin_y = pos.at(1);
            loc.pos.origin_z = pos.at(2);

            std::vector<double> quat;
            if( !rosparam_utilities::getParam(object,"quaternion",quat,what) )
            {
                ROS_WARN("pose has not the field 'quaternion'");
                continue;
            }

            assert(quat.size()==4);

            loc.quat.rotation_x = quat.at(0);
            loc.quat.rotation_y = quat.at(1);
            loc.quat.rotation_z = quat.at(2);
            loc.quat.rotation_w = quat.at(3);

            object_.grasp.push_back(loc);

            std::vector<double> approach_distance_d;
            if( !rosparam_utilities::getParam(object,"approach_distance",approach_distance_d,what) )
            {
                ROS_WARN("The box %s has not the field 'approach_distance'",object_.name.c_str());
                return false;
            }
            assert(approach_distance_d.size() == 3);

            position approach;
            approach.origin_x = approach_distance_d.at(0);
            approach.origin_y = approach_distance_d.at(1);
            approach.origin_z = approach_distance_d.at(2);

            object_.approach.push_back(approach);
        }

        bool presence = false;
        for ( int j = 0; j < objects.size(); j++)
        {
            if ( !object_.name.compare(objects[j].name) )
            {
                presence = true;
            }
        }

        if ( !presence )
        {
            objects.push_back(object_);
            objects_compare.push_back(object_);
        }
    }


    return true;
}

bool QNode::readSlotsGroupFromParam()
{
    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue config;
    if (!nh_.getParam("/outbound/slots_group",config))
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
    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue config;
    if (!nh_.getParam("/outbound/slots",config))
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

        slot slot_;

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
    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue go_to_locations_param;
    if (!nh_.getParam("/go_to_location",go_to_locations_param))
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
    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue config;

    if ( !nh_.getParam("/multi_skills/tasks",config) )
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
            go_to go_to_;

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
