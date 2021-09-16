#include <tuple>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/GoToAction.h>
#include <manipulation_msgs/RemoveObjectFromSlot.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <manipulation_interface_gui/recipe_test_msg.h>

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

struct action
{
    std::string type; // action
    std::vector<std::string> goal; // description
    std::string approach_loc; // approach_loc_ctrl_id
    std::string to_loc; // to_loc_ctrl_id
    std::string leave_loc; // leave_loc_ctrl_id
    std::string tool_id; // tool_id
    std::string job_exec; // job_exec_name
    std::string pre_exec_id; // property_pre_exec_id
    std::string exec_id; // property_exec_id
    std::string post_exec_id; // property_post_exec_id
};

bool run_recipe( manipulation_interface_gui::recipe_test_msg::Request& req,
                 manipulation_interface_gui::recipe_test_msg::Response& res)
{
    ros::NodeHandle nh_;

    manipulation_msgs::RemoveObjectFromSlot remove_object_from_slot;

    actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction>  pick_ac ("/inbound_pick_server/"+req.robot_name+"/pick");
    actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac("/outbound_place_server/"+req.robot_name+"/place");
    actionlib::SimpleActionClient<manipulation_msgs::GoToAction>         go_to_ac("/go_to_location_server/"+req.robot_name+"/go_to");

    std::string grasped_object = req.grasped_object_in;

    ROS_WARN("Waiting for pick server");
    pick_ac.waitForServer();
    ROS_WARN("Connection ok");
    ROS_WARN("Waiting for place server");
    place_ac.waitForServer();
    ROS_WARN("Connection ok");
    ROS_WARN("Waiting for goto server");
    go_to_ac.waitForServer();
    ROS_WARN("Connection ok");

    manipulation_msgs::PickObjectsResult pick_result;
    manipulation_msgs::PlaceObjectsResult place_result;
    manipulation_msgs::GoToResult goto_result;

    ros::ServiceClient remove_object_from_slot_clnt = nh_.serviceClient<manipulation_msgs::RemoveObjectFromSlot>("/outbound_place_server/remove_obj_from_slot");
    remove_object_from_slot_clnt.waitForExistence();

    XmlRpc::XmlRpcValue param_;

    if (!nh_.getParam("/recipe_to_run",param_))
    {
      ROS_ERROR("There isn't the recipe to run");
      return false;
    }

    if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Recipe is not a list" );
      return false;
    }

    std::vector<std::string> recipe;

    for ( int i = 0; i < param_.size(); i++ )
    {
        recipe.push_back(static_cast<std::string>(param_[i]));
    }



    if ( !nh_.getParam("/multi_skills/tasks",param_) )
    {
        ROS_ERROR("Unable to find /multi_skills/tasks");
        return false;
    }
    if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("The tasks param is not a list of action" );
        return false;
    }
    ROS_INFO("There are %d action",param_.size());

    std::vector<go_to> go_to_actions;
    std::vector<pick>  pick_actions;
    std::vector<place> place_actions;

    for ( int i = 0; i < param_.size(); i++)
    {
        XmlRpc::XmlRpcValue param__ = param_[i];
        if ( !param__.hasMember("type") )
        {
            ROS_WARN("The action #%d has not the field 'type'", i);
            return false;
        }
        std::string type_ = rosparam_utilities::toString(param__["type"]);

        if ( !type_.compare("goto") )
        {
            go_to go_to_;

            if( !param__.hasMember("name") )
            {
                ROS_WARN("The action #%d has not the field 'name'", i);
                return false;
            }
            go_to_.name = rosparam_utilities::toString(param__["name"]);

            std::string what;
            std::vector<std::string> locations_;
            if( !rosparam_utilities::getParam(param__,"goal",locations_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_.locations = locations_;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param__,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            go_to_.agents = agents_;

            if( !param__.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                go_to_.description = " ";
            }
            else
            {
                go_to_.description = rosparam_utilities::toString(param__["description"]);
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
            }
        }

        if ( !type_.compare("place") )
        {
            place place_;

            if( !param__.hasMember("name") )
            {
                ROS_WARN("The element place #%d has not the field 'name'", i);
                return false;
            }
            place_.name = rosparam_utilities::toString(param__["name"]);

            std::string what;
            std::vector<std::string> groups_;
            if( !rosparam_utilities::getParam(param__,"goal",groups_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_.groups = groups_;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param__,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            place_.agents = agents_;

            if( !param__.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                place_.description = " ";
            }
            else
            {
                place_.description = rosparam_utilities::toString(param__["description"]);
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
            }
        }

        if ( !type_.compare("pick") )
        {
            pick pick_;

            if( !param__.hasMember("name") )
            {
                ROS_WARN("The element pick #%d has not the field 'name'", i);
                return false;
            }
            pick_.name = rosparam_utilities::toString(param__["name"]);

            std::string what;
            std::vector<std::string> objects_;
            if( !rosparam_utilities::getParam(param__,"goal",objects_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_.objects = objects_;

            std::vector<std::string> agents_;
            if( !rosparam_utilities::getParam(param__,"agent",agents_,what) )
            {
                ROS_WARN("Action %d  has not the field 'goal'", i);
                return false;
            }
            pick_.agents = agents_;

            if( !param__.hasMember("description") )
            {
                ROS_WARN("The action #%d has not the field 'description'", i);
                pick_.description = " ";
            }
            else
            {
                pick_.description = rosparam_utilities::toString(param__["description"]);
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
            }
        }
    }

    action single_action;
    std::vector<action> actions;
    for ( int i = 0; i < recipe.size(); i++ )
    {
        for ( int j = 0; j < go_to_actions.size(); j++ )
        {
            if ( !recipe[i].compare(go_to_actions[j].name) )
            {
                single_action.type         = "goto";
                single_action.goal         = go_to_actions[j].locations;
                single_action.approach_loc = "trajectory_tracking";
                single_action.to_loc       = "trajectory_tracking";
                single_action.leave_loc    = "trajectory_tracking";
                single_action.tool_id      = "gripper_fake";
                single_action.job_exec     = "go_to";
                single_action.pre_exec_id  = "open";
                single_action.exec_id      = "open";
                single_action.post_exec_id = "open";
            }
        }
        for ( int j = 0; j < pick_actions.size(); j++ )
        {
            if ( !recipe[i].compare(pick_actions[j].name) )
            {
                single_action.type         = "pick";
                single_action.goal         = pick_actions[j].objects;
                single_action.approach_loc = "trajectory_tracking";
                single_action.to_loc       = "trajectory_tracking";
                single_action.leave_loc    = "trajectory_tracking";
                single_action.tool_id      = "gripper_fake";
                single_action.job_exec     = "pick";
                single_action.pre_exec_id  = "open";
                single_action.exec_id      = "close";
                single_action.post_exec_id = "close";
            }
        }
        for ( int j = 0; j < place_actions.size(); j++ )
        {
            if ( !recipe[i].compare(place_actions[j].name) )
            {
                single_action.type         = "place";
                single_action.goal         = place_actions[j].groups;
                single_action.approach_loc = "trajectory_tracking";
                single_action.to_loc       = "trajectory_tracking";
                single_action.leave_loc    = "trajectory_tracking";
                single_action.tool_id      = "gripper_fake";
                single_action.job_exec     = "place";
                single_action.pre_exec_id  = "close";
                single_action.exec_id      = "open";
                single_action.post_exec_id = "open";
            }
        }
        actions.push_back(single_action);
    }

    for ( const action action_: actions)
    {
        if ( !action_.type.compare("pick") )
        {
            if ( !grasped_object.empty() )
            {
                res.result = -1;
                res.result_string = "Pick: object already grasped";
                return true;
            }

            manipulation_msgs::PickObjectsGoal pick_goal;

            pick_goal.object_types          = action_.goal;
            pick_goal.approach_loc_ctrl_id  = action_.approach_loc;
            pick_goal.to_loc_ctrl_id        = action_.to_loc      ;
            pick_goal.leave_loc_ctrl_id     = action_.leave_loc   ;
            pick_goal.job_exec_name         = action_.job_exec    ;
            pick_goal.tool_id               = action_.tool_id     ;
            pick_goal.property_pre_exec_id  = action_.pre_exec_id ;
            pick_goal.property_exec_id      = action_.exec_id     ;
            pick_goal.property_post_exec_id = action_.post_exec_id;

            pick_ac.sendGoalAndWait(pick_goal);
            pick_result = *pick_ac.getResult();
            res.result = pick_result.result;

            switch (pick_result.result)
            {
                case manipulation_msgs::PickObjectsResult::Success :
                    res.result_string = "Pick: Success";
                    break;
                case manipulation_msgs::PickObjectsResult::NoInboundBoxFound :
                    res.result_string = "Pick: NoInboundBoxFound";
                    break;
                case manipulation_msgs::PickObjectsResult::NoAvailableTrajectories :
                    res.result_string = "Pick: NoAvailableTrajectories";
                    break;
                case manipulation_msgs::PickObjectsResult::GraspFailure :
                    res.result_string = "Pick: GraspFailure";
                    break;
                case manipulation_msgs::PickObjectsResult::TrajectoryError :
                    res.result_string = "Pick: TrajectoryError";
                    break;
                case manipulation_msgs::PickObjectsResult::SceneError :
                    res.result_string = "Pick: SceneError";
                    break;
                case manipulation_msgs::PickObjectsResult::ControllerError :
                    res.result_string = "Pick: ControllerError";
                    break;
                case manipulation_msgs::PickObjectsResult::UnexpectedError :
                    res.result_string = "Pick: UnexpectedError";
                    break;
            }

            if (pick_result.result < 0)
            {
                ROS_ERROR("Unable to pick -> object type = %s",pick_result.object_name.c_str());
                grasped_object.clear();
                res.grasped_object_out = grasped_object;
                return true;
            }
            ROS_INFO("Well done! I picked it, name = %s",pick_result.object_name.c_str());
            grasped_object = pick_result.object_name;
            res.grasped_object_out = grasped_object;
        }
        else if ( !action_.type.compare("place") )
        {
            manipulation_msgs::PlaceObjectsGoal place_goal;

            if ( !grasped_object.empty() )
                place_goal.object_name = grasped_object;
            else
            {
                ROS_ERROR("No object name");
                res.result = -1;
                res.result_string = "No grasped object";
                return true;
            }

            place_goal.slots_group_names     = action_.goal;
            place_goal.approach_loc_ctrl_id  = action_.approach_loc;
            place_goal.to_loc_ctrl_id        = action_.to_loc      ;
            place_goal.leave_loc_ctrl_id     = action_.leave_loc   ;
            place_goal.job_exec_name         = action_.job_exec    ;
            place_goal.tool_id               = action_.tool_id     ;
            place_goal.property_pre_exec_id  = action_.pre_exec_id ;
            place_goal.property_exec_id      = action_.exec_id     ;
            place_goal.property_post_exec_id = action_.post_exec_id;

            place_ac.sendGoalAndWait(place_goal);
            place_result = *place_ac.getResult();
            res.result = place_result.result;

            switch (place_result.result)
            {
            case manipulation_msgs::PlaceObjectsResult::Success :
                res.result_string = "Place: Success";
                break;
            case manipulation_msgs::PlaceObjectsResult::NotInitialized :
                res.result_string = "Place: NotInitialized";
                break;
            case manipulation_msgs::PlaceObjectsResult::Full :
                res.result_string = "Place: Full";
                break;
            case manipulation_msgs::PlaceObjectsResult::ReleaseError :
                res.result_string = "Place: ReleaseError";
                break;
            case manipulation_msgs::PlaceObjectsResult::SceneError :
                res.result_string = "Place: SceneError";
                break;
            case manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories :
                res.result_string = "Place: NoAvailableTrajectories";
                break;
            case manipulation_msgs::PlaceObjectsResult::TrajectoryError :
                res.result_string = "Place: TrajectoryError";
                break;
            case manipulation_msgs::PlaceObjectsResult::ReturnError :
                res.result_string = "Place: ReturnError";
                break;
            case manipulation_msgs::PlaceObjectsResult::ControllerError :
                res.result_string = "Place: ControllerError";
                break;
            case manipulation_msgs::PlaceObjectsResult::UnexpectedError :
                res.result_string = "Place: UnexpectedError";
                break;
            }

            if (place_result.result < 0)
            {
                ROS_ERROR("Unable to place -> object name = %s", place_goal.object_name.c_str());
                return true;
            }

            remove_object_from_slot.request.object_to_remove_name = place_goal.object_name;
            remove_object_from_slot.request.slot_name = place_result.slot_name;

            if (!remove_object_from_slot_clnt.call(remove_object_from_slot))
            {
                ROS_ERROR("Unespected error calling %s service",remove_object_from_slot_clnt.getService().c_str());
                return 0;
            }

            ROS_INFO("Well done! ");
            grasped_object.clear();
            res.grasped_object_out = grasped_object;
        }
        else if ( !action_.type.compare("goto") )
        {
            manipulation_msgs::GoToGoal go_to_goal;


            go_to_goal.location_names.push_back( action_.goal[0] );
            go_to_goal.to_loc_ctrl_id   = action_.to_loc      ;
            go_to_goal.job_exec_name    = action_.job_exec    ;
            go_to_goal.tool_id          = action_.tool_id     ;
            go_to_goal.property_exec_id = action_.exec_id     ;

            go_to_ac.sendGoalAndWait(go_to_goal);
            goto_result = *go_to_ac.getResult();
            res.result = goto_result.result;

            switch (goto_result.result)
            {
            case manipulation_msgs::GoToResult::Success :
                res.result_string = "Goto: Success";
                break;
            case manipulation_msgs::GoToResult::SceneError :
                res.result_string = "Goto: SceneError";
                break;
            case manipulation_msgs::GoToResult::NoAvailableTrajectories :
                res.result_string = "Goto: NoAvailableTrajectories";
                break;
            case manipulation_msgs::GoToResult::TrajectoryError :
                res.result_string = "Goto: TrajectoryError";
                break;
            case manipulation_msgs::GoToResult::NotInitialized :
                res.result_string = "Goto: NotInitialized";
                break;
            case manipulation_msgs::GoToResult::ToolError :
                res.result_string = "Goto: ToolError";
                break;
            case manipulation_msgs::GoToResult::ControllerError :
                res.result_string = "Goto: ControllerError";
                break;
            }

            if (goto_result.result < 0)
            {
                ROS_ERROR("[Unable to go to -> location name = %s", go_to_goal.location_names.at(0).c_str());
                return true;
            }
            res.grasped_object_out = grasped_object;
            ROS_INFO("Well done! ");
        }
        else
        {
            ROS_ERROR("Unable to execute the action because its type is wrong");
            return 0;
        }


    }

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recipe_test_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService( "run_recipe", run_recipe);
  ROS_INFO("Ready to run recipe.");
  ros::spin();

  return 0;
}
