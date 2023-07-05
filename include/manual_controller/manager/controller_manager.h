#include <manual_controller/hardware_interface/asda_hardware_interface.h>
#include <manual_controller/visualrobot_interface/visualrobot_interface.h>

// the message for reporting slaves
#include "manual_controller/SlaveStatus.h"

#include <std_msgs/Time.h>
#include "ros/ros.h"
#include "ros/time.h"

namespace controller_manager {

    inline manual_controller::SlaveStatus generate_slave_status_msg(delta::asda::ServoHW &servo_hw);

    int manage_control(delta::asda::ServoHW &servo_hw, moveit_commander::Commander &moveit_commander, ros::NodeHandle &node_handle)
    {   
        auto freq = node_handle.param<double>("publish_frequency", 10);


        if (servo_hw.init())
        {
            ROS_INFO("Hardware Interface initialized correctly");
        }
        else
        {
            ROS_FATAL("Failed to initialize Hardware Interface");
            return 1;
        }


        // Advertised Services
        auto fault_reset_srv = node_handle.advertiseService("fault_reset", &delta::asda::ServoHW::fault_reset, &servo_hw);
        auto ready_to_switch_on_srv = node_handle.advertiseService("ready_to_switch_on", &delta::asda::ServoHW::ready_to_switch_on, &servo_hw);
        auto switch_on_srv = node_handle.advertiseService("switch_on", &delta::asda::ServoHW::switch_on, &servo_hw);
        auto enable_operation_srv = node_handle.advertiseService("enable_operation", &delta::asda::ServoHW::enable_operation, &servo_hw);
        auto halt_srv = node_handle.advertiseService("halt", &delta::asda::ServoHW::halt, &servo_hw);
        auto quick_stop_srv = node_handle.advertiseService("quick_stop", &delta::asda::ServoHW::quick_stop, &servo_hw);

        // Advertised Topics
        auto hardware_status_publisher = node_handle.advertise<manual_controller::SlaveStatus>("hardware_status", 10);

        // Start
        if (servo_hw.start())
        {
            ROS_INFO("Hardware Interface started.");

            if (moveit_commander.set_joint_position(servo_hw.pulse_to_position()))
            {
                ROS_INFO("Plan And Execute Visual Robot to Current Position!");
            }
            else
            {
                ROS_ERROR("Failed To Plan And Execute Visual Robot to Current Position!");
                return 1;
            }

        }
        else
        {
            ROS_FATAL("Failed to start Hardware Interface");
            return 1;
        }

        // Loop
        ros::Rate rate(freq);
        while (ros::ok())
        {
            rate.sleep();

            manual_controller::SlaveStatus hardware_status_msg = generate_slave_status_msg(servo_hw);

            // industrial_msgs::RobotStatus status_msg;
            // status_msg.header.stamp = time;
            // status_msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
            // status_msg.e_stopped.val = (!servo_hw.status.quick_stop) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
            // status_msg.drives_powered.val = (servo_hw.status.switched_on) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
            // status_msg.motion_possible.val = (servo_hw.status.operation_enabled) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
            // status_msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
            // status_msg.in_error.val = (servo_hw.status.fault) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
            // status_msg.error_code = 0;

            // if (servo_hw.status.fault)
            // {
            // for (int i = 0; i < servo_hw.slave_count(); i++)
            // {
            //     const uint16 slave_idx = 1 + i;
            //     uint16 error_code;
            //     servo_hw.get_error_code(slave_idx, error_code);
            // }
            // }

            // if (servo_hw.status.warning)
            // {
            // for (int i = 0; i < servo_hw.slave_count(); i++)
            // {
            //     const uint16 slave_idx = 1 + i;
            //     uint16 error_code;
            //     servo_hw.get_error_code(slave_idx, error_code);
            //     status_msg.error_code = error_code;
            // }
            // }

            hardware_status_publisher.publish(hardware_status_msg);
        }

        servo_hw.close();
        return 0;
    }

    inline manual_controller::SlaveStatus generate_slave_status_msg(delta::asda::ServoHW &servo_hw)
    {
        manual_controller::SlaveStatus slave_status;

        std_msgs::Time time_stamp;
        time_stamp.data = ros::Time::now();

        slave_status.stamp = time_stamp;
        slave_status.stopped = !servo_hw.status.quick_stop;
        slave_status.drives_powered = servo_hw.status.switched_on;
        slave_status.motion_possible = servo_hw.status.operation_enabled;
        slave_status.in_error = servo_hw.status.fault;
    

        if (slave_status.in_error || servo_hw.status.warning)
        {
            for (int i = 0; i < servo_hw.slave_count(); i++)
            {
                const uint16 slave_idx = 1 + i;
                uint16 error_code;
                servo_hw.get_error_code(slave_idx, error_code);
                slave_status.error_code = error_code;


                if (servo_hw.get_error_message(slave_status.error_code, slave_status.error_message))
                {
                    ROS_WARN("Deleta Servo Error: %s", slave_status.error_message.c_str());
                }
                else
                {
                    ROS_ERROR("Deleta Servo Error Message Not Found!: %s", slave_status.error_message.c_str());
                    slave_status.error_message = "Deleta Servo Error Message Not Found!";
                }
            }
        }

        

        return slave_status;
    }
}