#include "manual_controller/visualrobot_interface/visualrobot_interface.h"

using namespace moveit_commander;


/// @brief  this function to set the joint position by degrees
/// @param joint_position 
/// @return true if everything is Ok
bool Commander::set_joint_position(const std::vector<double> &joint_position)
{   
    try
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;


        std::vector<double> joint_group_positions;
        arm_current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


        if ((joint_position.size() != 0 && joint_group_positions.size() != 0) && joint_group_positions.size() >= joint_position.size())
        {
            bool success_plan;
            
            for (int i = 0; i < joint_position.size(); i++)
            {
                joint_group_positions.at(i) = degrees_to_radians(joint_position.at(i));
            }

            arm_move_group->setJointValueTarget(joint_group_positions);
    
            success_plan = (arm_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success_plan)
                arm_move_group->execute(my_plan);
            else
                ROS_ERROR("Error To Plan");
        }

    }
    catch (std::exception &e)
    {
        ROS_ERROR("%s", e.what());
        return false;
    }

    return true;
}