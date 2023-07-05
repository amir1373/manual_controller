#include <vector>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/console.h"


#include "manual_controller/hardware_interface/asda_hardware_interface.h"
#include "manual_controller/manager/controller_manager.h"
#include "manual_controller/visualrobot_interface/visualrobot_interface.h"

using namespace std;
using namespace delta::asda;
using namespace moveit_commander;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_controller");
    ROS_INFO("Manaul Is Up");
    
    ros::NodeHandle node_handle("~");
    
    ros::CallbackQueue callback_queue;
    node_handle.setCallbackQueue(&callback_queue);

    // parameters
    double loop_hz;
    if (!node_handle.getParam("/rail/hardware_interface/loop_hz", loop_hz))
    {
      std::string param_name = node_handle.resolveName("/rail/hardware_interface/loop_hz");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return 1;
    }

    string arm_planning_name;
    if (!node_handle.getParam("/manual_controller/moveit_interface/arm_planning_name", arm_planning_name))
    {
      ROS_ERROR("Failed to get arm Planning Name");
      return 1;
    }

    string hand_planning_name;
    if (!node_handle.getParam("/manual_controller/moveit_interface/hand_planning_name", hand_planning_name))
    {
      ROS_ERROR("Failed to get hand Planing Name");
      return 1;
    }

    // // init spinner
    ros::AsyncSpinner spinner(2, &callback_queue);
    spinner.start();

    // // create hardeare class
    ServoHW servo_hw(node_handle, loop_hz);

    // // create moveit_interface class
    Commander moveit_commander(arm_planning_name, hand_planning_name);
    

    // // Manger Control
    return controller_manager::manage_control(servo_hw, moveit_commander, node_handle);
}