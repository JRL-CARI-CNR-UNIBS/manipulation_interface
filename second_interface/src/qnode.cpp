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
#include <std_msgs/String.h>
#include <sstream>
#include "../include/second_interface/qnode.hpp"
#include <rosparam_utilities/rosparam_utilities.h>
#include <mongo_interactions/SaveParam.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace second_interface {

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
	ros::init(init_argc,init_argv,"second_interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    
    return true;
}

void QNode::load_actions()
{
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue go_to_param;
    XmlRpc::XmlRpcValue places_param;
    XmlRpc::XmlRpcValue picks_param;

    logging_model_go_to.removeRows (0, logging_model_go_to.rowCount());
    logging_model_place.removeRows (0, logging_model_place.rowCount());
    logging_model_pick.removeRows  (0, logging_model_pick.rowCount());

    if ( !n.getParam("/go_to_location",go_to_param) )
    {
        ROS_ERROR("go_to_param don't find");
    }
    else if( go_to_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Go_to is not a list");
    }
    else
    {
        for ( size_t i = 0; i < go_to_param.size(); i++ )
        {
            go_to single_go_to;
            XmlRpc::XmlRpcValue param = go_to_param[i];

            if ( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("The element #%zu is not a struct", i);
            }
            if ( !param.hasMember("frame") )
            {
                ROS_ERROR("The element #%zu has not the field 'frame'", i);
                single_go_to.frame=" ";
            }
            else
            {
                single_go_to.frame = rosparam_utilities::toString(param["frame"]);
            }
            if ( !param.hasMember("name") )
            {
                ROS_ERROR("The element #%zu has not the field 'name'", i);
                single_go_to.name=" ";
            }
            else
            {
                single_go_to.name = rosparam_utilities::toString(param["name"]);
            }
            go_to_actions.push_back(single_go_to);
            log_go_to(single_go_to.name);
        }
    }

    if ( !n.getParam("/places_param",places_param) )
    {
        ROS_ERROR("places_param don't find");
    }
    else if ( places_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Places is not a list");
    }
    else
    {
        for ( size_t i = 0; i < places_param.size(); i++ )
        {
            place single_place;
            XmlRpc::XmlRpcValue param = places_param[i];

            if ( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("The element #%zu is not a struct", i);
            }
            if ( !param.hasMember("name") )
            {
                ROS_ERROR("The element #%zu has not the field 'name'", i);
                single_place.name=" ";
            }
            else
            {
                single_place.name = rosparam_utilities::toString(param["name"]);
            }
            if ( !param.hasMember("groups") )
            {
                ROS_ERROR("The element #%zu has not the field 'groups'", i);
            }
            else
            {
                std::string what;
                if( !rosparam_utilities::getParam(param,"groups",single_place.groups,what) )
                {
                  ROS_WARN("The element #%zu has not the field 'groups'", i);
                  continue;
                }
            }
            place_actions.push_back(single_place);
            log_place(single_place.name);
        }
    }

    if ( !n.getParam("/picks_param",picks_param) )
    {
        ROS_ERROR("picks_param don't find");
    }
    else if ( picks_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Picks is not a list");
    }
    else
    {
        for ( size_t i = 0; i < picks_param.size(); i++ )
        {
            pick single_pick;
            XmlRpc::XmlRpcValue param = picks_param[i];

            if ( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("The element #%zu is not a struct", i);
            }
            if ( !param.hasMember("name") )
            {
                ROS_ERROR("The element #%zu has not the field 'name'", i);
                single_pick.name=" ";
            }
            else
            {
                single_pick.name = rosparam_utilities::toString(param["name"]);
            }
            if ( !param.hasMember("objects") )
            {
                ROS_ERROR("The element #%zu has not the field 'objects'", i);
            }
            else
            {
                std::string what;
                if( !rosparam_utilities::getParam(param,"objects",single_pick.objects,what) )
                {
                  ROS_WARN("The element #%zu has not the field 'groups'", i);
                  continue;
                }
            }
            pick_actions.push_back(single_pick);
            log_pick(single_pick.name);
        }
    }

}

void QNode::add_object_type (int ind)
{
    if ( logging_model_objects.rowCount() != 0 )
    {
        logging_model_objects.removeRows(0,logging_model_objects.rowCount());
    }
    if ( ind != 1000)
    {
        for ( int i = 0; i < pick_actions[ind].objects.size(); i++)
        {
            log_objects( pick_actions[ind].objects[i] );
        }
    }
}

void QNode::add_slot_groups (int ind)
{
    if ( logging_model_slots.rowCount() != 0 )
    {
        logging_model_slots.removeRows(0,logging_model_slots.rowCount());
    }
    if ( ind != 1000)
    {
        for ( int i = 0; i < place_actions[ind].groups.size(); i++)
        {
            log_slots( place_actions[ind].groups[i] );
        }
    }
}

void QNode::add_go_to ( int ind )
{
    log_recipe( go_to_actions[ind].name );
}

void QNode::add_place ( int ind )
{
    log_recipe( place_actions[ind].name );
}

void QNode::add_pick  ( int ind )
{
    log_recipe( pick_actions[ind].name );
}

std::vector<std::string> QNode::load_recipe ( bool init, std::string name_recipe)
{
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue recipe_param;

    std::vector<std::string> recipes_param, recipes_names;

    if (!n.getParamNames(param_names))
    {
        ROS_ERROR("Empty robot parameter");
        return recipes_names;
    }

    for ( int i = 0; i < param_names.size(); i++)
    {
        std::size_t found  = param_names[i].find("/multi_skills/recipes");
        if ( found != std::string::npos )
        {
            std::size_t found2  = param_names[i].find( "/", 5 );
            std::size_t found3  = param_names[i].find( "/", found2+1 );
            std::string str = param_names[i];
            recipes_names.push_back( str.erase( 0, found3+1 ) );
            recipes_param.push_back( param_names[i] );
        }
    }

    if ( !init )
    {
        return recipes_names;
    }
    else
    {
        int num_action = 0;
        std::string str = "/multi_skills/recipes/";
        str.append(name_recipe);
        ROS_ERROR("%s", str.c_str());
        if ( !n.getParam(str,recipe_param) )
        {
            ROS_ERROR("recipe_param don't find");
        }
        else if ( recipe_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("recipe is not a list");
        }
        else
        {
            for ( size_t i = 0; i < recipe_param.size(); i++ )
            {
                std::string type;
                XmlRpc::XmlRpcValue param = recipe_param[i];

                if ( param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("The element #%zu is not a struct", i);
                }
                if ( !param.hasMember("action") )
                {
                    ROS_ERROR("The element #%zu has not the field 'action'", i);
                    type = " ";
                }
                else
                {
                    type = rosparam_utilities::toString(param["action"]);
                }
                if ( !param.hasMember("description") )
                {
                    ROS_ERROR("The element #%zu has not the field 'description'", i);
                }
                else
                {
                    if ( !type.compare("goto") )
                    {
                        std::string what;
                        std::vector<std::string> description;
                        if( !rosparam_utilities::getParam(param,"description",description,what) )
                        {
                          ROS_WARN("The element #%zu has not the field 'description'", i);
                          continue;
                        }
                        std::string name;
                        for (int j = 0; j < go_to_actions.size(); j++ )
                        {

                            if ( !description[0].compare( go_to_actions[j].name ) )
                            {
                                name = go_to_actions[j].name;
                            }
                        }
                        if ( !name.empty() )
                        {
                            log_recipe( name );
                        }
                        else
                        {
                            num_action++;
                            name="go_to";
                            name.append(std::to_string(num_action));
                            log_recipe( name );
                        }
                    }
                    else if ( !type.compare("place") )
                    {
                        std::string what;
                        std::vector<std::string> description;
                        if( !rosparam_utilities::getParam(param,"description",description,what) )
                        {
                          ROS_WARN("The element #%zu has not the field 'description'", i);
                          continue;
                        }
                        std::string name;
                        for ( int j = 0; j < place_actions.size(); j++ )
                        {
                            if ( compare( description, place_actions[j].groups ) )
                            {
                                name = place_actions[j].name;
                            }
                        }
                        if ( name.empty() )
                        {
                            num_action++;
                            name="place";
                            name.append(std::to_string(num_action));
                        }
                        log_recipe( name );
                    }
                    else if ( !type.compare("pick") )
                    {
                        std::string what;
                        std::vector<std::string> description;
                        if( !rosparam_utilities::getParam(param,"description",description,what) )
                        {
                          ROS_WARN("The element #%zu has not the field 'description'", i);
                          continue;
                        }
                        std::string name;
                        for ( int j = 0; j < pick_actions.size(); j++ )
                        {
                            if ( compare( description, pick_actions[j].objects ) )
                            {
                                name = pick_actions[j].name;
                            }
                        }
                        if ( name.empty() )
                        {
                            num_action++;
                            name="pick";
                            name.append(std::to_string(num_action));
                        }
                        log_recipe( name );
                    }
                }
            }
        }
        return recipes_names;
    }


}

bool QNode::save_all( std::string recipe_name )
{
    ROS_ERROR("recipe_name = %s", recipe_name.c_str());
    ros::NodeHandle n;
    action_list.clear();
    for ( int i = 0; i < logging_model_recipe.rowCount(); i++ )
    {
        action action_;
        for ( int j = 0; j < logging_model_go_to.rowCount(); j++ )
        {
            if ( !logging_model_go_to.data( logging_model_go_to.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                action_.type  = "go_to";
                action_.index = j;
            }
        }
        for ( int j = 0; j < logging_model_place.rowCount(); j++ )
        {
            if ( !logging_model_place.data( logging_model_place.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                action_.type  = "place";
                action_.index = j;
            }
        }
        for ( int j = 0; j < logging_model_pick.rowCount(); j++ )
        {
            if ( !logging_model_pick.data( logging_model_pick.index( j ), 0 ).toString().toStdString().compare( logging_model_recipe.data( logging_model_recipe.index( i ), 0 ).toString().toStdString() ) )
            {
                action_.type  = "pick";
                action_.index = j;
            }
        }
        action_list.push_back(action_);
    }

    XmlRpc::XmlRpcValue param;

    for ( int i = 0; i <  action_list.size(); i++)
    {
        if ( !action_list[i].type.compare("go_to") )
        {
            param[i] = get_action_go_to_param(action_list[i].index);
        }
        else if ( !action_list[i].type.compare("place") )
        {
            param[i] = get_action_place_param(action_list[i].index);
        }
        else if ( !action_list[i].type.compare("pick") )
        {
            param[i] = get_action_pick_param(action_list[i].index);
        }
        else
        {
            ROS_ERROR("The type of action is wrong");
        }

    }
    n.setParam("/multi_skills/recipe", param);
    std::string str = "/multi_skills/recipes/";
    str.append(recipe_name);
    n.setParam( str, param);

    ros::ServiceClient client = n.serviceClient<mongo_interactions::SaveParam>("save_recipe_param_on_mongo");
    mongo_interactions::SaveParam srv;
    client.waitForExistence();
     if ( !client.call(srv) )
    {
        ROS_ERROR("Unable to call %s service",client.getService().c_str());
        return false;
    }
    return true;
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

void QNode::log_go_to( const std::string &msg)
{
    logging_model_go_to.insertRows(logging_model_go_to.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_go_to.setData(logging_model_go_to.index(logging_model_go_to.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_place (const std::string &msg)
{
    logging_model_place.insertRows(logging_model_place.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_place.setData(logging_model_place.index(logging_model_place.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_pick  (const std::string &msg)
{
    logging_model_pick.insertRows(logging_model_pick.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_pick.setData(logging_model_pick.index(logging_model_pick.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_objects(const std::string &msg)
{
    logging_model_objects.insertRows(logging_model_objects.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_objects.setData(logging_model_objects.index(logging_model_objects.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_slots (const std::string &msg)
{
    logging_model_slots.insertRows(logging_model_slots.rowCount(),1);
    ROS_DEBUG_STREAM(msg);
    QVariant new_row(QString(msg.c_str()));
    logging_model_slots.setData(logging_model_slots.index(logging_model_slots.rowCount()-1),new_row);
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

bool QNode::compare(std::vector<std::string> &v1, std::vector<std::string> &v2)
{
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    return v1 == v2;
}

}  // namespace second_interface
