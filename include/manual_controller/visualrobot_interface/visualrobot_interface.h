#ifndef VISUALROBOT_INTERFACE_H
#define VISUALROBOT_INTERFACE_H

#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "manual_controller/math/math.h"

namespace moveit_commander
{
    class Commander
    {   
        private:
            const std::string PLANNING_ARM_NAME;
            const std::string PLANNING_GRIPPER_NAME;

            moveit::planning_interface::MoveGroupInterface* arm_move_group;
            moveit::planning_interface::MoveGroupInterface* hand_move_group;
            const moveit::core::JointModelGroup* joint_model_group;
    
            moveit::core::RobotStatePtr arm_current_state;
            
            ros::AsyncSpinner *spinner;

        public:

            Commander(const std::string &planning_arm, const std::string &planning_gripper): 
            PLANNING_ARM_NAME(planning_arm) , PLANNING_GRIPPER_NAME(planning_gripper)
            {   
                spinner = new ros::AsyncSpinner(1);
                spinner->start();

                arm_move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_ARM_NAME);
                hand_move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GRIPPER_NAME);
                arm_current_state = arm_move_group->getCurrentState();

                joint_model_group = arm_move_group->getCurrentState()->getJointModelGroup(PLANNING_ARM_NAME);
            }

            bool set_joint_position(const std::vector<double> &joint_position);

    };
}

#endif