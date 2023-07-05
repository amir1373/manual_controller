#ifndef DELTA_ASDA_HARDWARE_INTERFACE_H
#define DELTA_ASDA_HARDWARE_INTERFACE_H
#include <string>
#include <vector>
#include <cmath>
#include <map>
// UNIX
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
// xmlrpcpp
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
// delta_servo
#include "manual_controller/ethercat/master.h"
#include "manual_controller/ethercat/time.h"
#include "manual_controller/ethercat/common.h"
// ros standard message
#include "sensor_msgs/JointState.h" 

#include "manual_controller/math/math.h"

#define POSITION_STEP_FACTOR  100000.0
#define VELOCITY_STEP_FACTOR  100000.0


namespace delta { namespace asda {

void* control_loop(void* arg);

class ServoHW{
private:

  bool init_ethercat(unsigned long cycletime, const std::string &ifname, const std::vector<std::string> &slaves)
  {
    ec_master = delta::asda::ethercat::Master(cycletime, ifname, slaves);

    if (ec_master.init())
    {
      ROS_INFO("EtherCAT Master interface: %s", ifname.c_str());
    }
    else
    {
      ROS_FATAL("Failed to initialize EtherCAT master.");
      return false;
    }

    for (int i = 0; i < slaves.size(); i++)
    {
      ROS_INFO("EtherCAT Slave[%d]: %s", 1 + i, slaves[i].c_str());
    }

    return true;
  }

  void joint_state_updater(const sensor_msgs::JointState::ConstPtr &joint_state)
  {
      ROS_INFO("In Joint State Updater");

      if (joint_positions.size() != joint_state->position.size())
      {
          joint_positions.resize(joint_state->position.size(), 0.0);
          pulse.resize(joint_state->position.size(), 0.0);
      }

      for (size_t i = 0; i < joint_state->position.size(); ++i)
      {
          joint_positions[i] = joint_state->position[i];

          // ROS_ERROR("joint[%ld] = %lf", i, joint_positions[i]);
      }

      position_to_pulse();
  }

  void position_to_pulse()
  {
    for (int i = 0; i < joint_positions.size(); i++)
    {
      pulse[i] = (radians_to_degrees( joint_positions[i] ) / 360) * (pulse_per_rotation * gear_ratio_list[i]);

      // ROS_ERROR("pulse[%d] = %d", i, pulse[i]);

    }
  }



  bool config_slaves(XmlRpc::XmlRpcValue &slaves_param)
  {
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        int position_window = slaves_param[i]["position_window"];
        int position_window_time = slaves_param[i]["position_window_time"];
        int interpolation_sub_mode = slaves_param[i]["interpolation_sub_mode"];
        int interpolation_time_period = slaves_param[i]["interpolation_time_period"];
        int following_error_window = slaves_param[i]["following_error_window"];
        int position_offset = slaves_param[i]["position_offset"];
        int position_factor[2] = { slaves_param[i]["position_factor"][0], slaves_param[i]["position_factor"][1] };
        int quickstop_deceleration = slaves_param[i]["quickstop_deceleration"];

        const uint16 slave_idx = 1 + i;
        ROS_DEBUG("EtherCAT Slave[%d] Following Error Window: %u", slave_idx, following_error_window);
        ROS_DEBUG("EtherCAT Slave[%d] Position Offset: %u", slave_idx, position_offset);
        ROS_DEBUG("EtherCAT Slave[%d] Position Factor: %u : %u", slave_idx, position_factor[0], position_factor[1]);
        ROS_DEBUG("EtherCAT Slave[%d] QuickStop Deceleration: %u", slave_idx, quickstop_deceleration);

        ec_master.config_position_interpolation(slave_idx, delta::asda::ethercat::interpolation_sub_mode_t::LINEAR_INTERPOLATION, interpolation_time_period);
        ec_master.set_following_error_window(slave_idx, following_error_window);
        ec_master.set_position_offset(slave_idx, position_offset);
        ec_master.set_position_factor(slave_idx, position_factor[0], position_factor[1]);
        ec_master.set_quickstop_deceleration(slave_idx, quickstop_deceleration);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
        return false;
      }
    }

    return true;
  }

  bool get_gear_ration(XmlRpc::XmlRpcValue &gear_ratio_data)
  {
    for (int i = 0; i < gear_ratio_data.size(); i++)
    {
      try
      {
        double gear_ratio = gear_ratio_data[i];
        gear_ratio_list.push_back((float) gear_ratio);
        ROS_INFO("gear_ratio[%d] = %f", i, gear_ratio_list[i]);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
        return false;
      }
    }

    return true;
  }

  ros::Subscriber joint_state_subscriber;
  std::vector<float> gear_ratio_list;
  int pulse_per_rotation;

  ros::NodeHandle node;

  std::vector<double> joint_positions;
  std::vector<int> pulse;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

public:

  double loop_hz;
  delta::asda::ethercat::Master ec_master;
  bool reset_controllers = true;

  struct {
    bool ready_to_switch_on;
    bool switched_on;
    bool operation_enabled;
    bool fault;
    bool voltage_enabled;
    bool quick_stop;
    bool switch_on_disabled;
    bool warning;
    bool remote;
    bool target_reached;
    bool internal_limit_active;
    bool homing_attained;
    bool homing_error;
    bool following_error;
  } status;


  ServoHW(ros::NodeHandle &node) :node(node)
  {}

  ServoHW(ros::NodeHandle &node, double &loop_hz) :node(node) , loop_hz(loop_hz)
  {}

  std::vector<double> pulse_to_position()
  { 

    std::vector<double> joint_pos;

    for (int i = 0; i < ec_slavecount; i++)
    {
      int slave_idx = i+1;
      double position = (ec_master.tx_pdo[slave_idx].actual_position * 360) / (pulse_per_rotation * gear_ratio_list[i]);
      joint_pos.push_back(position);
    }

    return joint_pos;
  }

  bool init(double loop_hz)
  {
    this->loop_hz = loop_hz;


    // EtherCAT
    unsigned long cycletime;
    cycletime = (1.0 / loop_hz) * 1000000000;

    std::string ifname;
    if (!node.getParam("ethercat/ifname", ifname))
    {
      std::string param_name = node.resolveName("ethercat/ifname");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue slaves_param;
    if (!node.getParam("ethercat/slaves", slaves_param))
    {
      std::string param_name = node.resolveName("ethercat/slaves");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    std::vector<std::string> slaves;
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        std::string device_name = slaves_param[i]["device_name"];
        slaves.push_back(device_name);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
      }
    }

    if (!init_ethercat(cycletime, ifname, slaves))
    {
      ROS_ERROR("Failed to initialize EtherCAT master");
      close();
      return false;
    }

    if (!config_slaves(slaves_param))
    {
      ROS_ERROR("Failed to configure EtherCAT slaves");
      close();
      return false;
    }

    if (!node.getParam("pulse_calculator/pulse_per_rotation", pulse_per_rotation))
    {
      ROS_ERROR("Failed to Get pulse per rotation");
      close();
      return false;
    }

    XmlRpc::XmlRpcValue gear_ratio_data;
    if (!node.getParam("pulse_calculator/gear_ratio", gear_ratio_data))
    {
      std::string param_name = node.resolveName("pulse_calculator/gear_ratio");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      close();
      return false;
    }

    if (!get_gear_ration(gear_ratio_data))
    {
      ROS_ERROR("Failed to configure gear ration");
      close();
      return false;
    }

    
    return true;  
  }

  bool init()
  {
    // EtherCAT
    unsigned long cycletime;
    cycletime = (1.0 / loop_hz) * 1000000000;

    std::string ifname;
    if (!node.getParam("ethercat/ifname", ifname))
    {
      std::string param_name = node.resolveName("ethercat/ifname");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue slaves_param;
    if (!node.getParam("ethercat/slaves", slaves_param))
    {
      std::string param_name = node.resolveName("ethercat/slaves");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    std::vector<std::string> slaves;
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        std::string device_name = slaves_param[i]["device_name"];
        slaves.push_back(device_name);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
      }
    }

    if (!init_ethercat(cycletime, ifname, slaves))
    {
      ROS_ERROR("Failed to initialize EtherCAT master");
      close();
      return false;
    }

    if (!config_slaves(slaves_param))
    {
      ROS_ERROR("Failed to configure EtherCAT slaves");
      close();
      return false;
    }

    if (!node.getParam("pulse_calculator/pulse_per_rotation", pulse_per_rotation))
    {
      ROS_ERROR("Failed to Get pulse per rotation");
      close();
      return false;
    }

    XmlRpc::XmlRpcValue gear_ratio_data;
    if (!node.getParam("pulse_calculator/gear_ratio", gear_ratio_data))
    {
      std::string param_name = node.resolveName("pulse_calculator/gear_ratio");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      close();
      return false;
    }

    if (!get_gear_ration(gear_ratio_data))
    {
      ROS_ERROR("Failed to configure gear ration");
      close();
      return false;
    }

    
    return true;  
  }


  int slave_count()
  {
    return ec_slavecount;
  }

  bool start()
  {

    joint_state_subscriber  = node.subscribe("/joint_states", 1000, &ServoHW::joint_state_updater, this);
  
    if (!ec_master.start())
    {
      return false;
    }

    pthread_t pthread;
    pthread_attr_t pthread_attr;

    errno = pthread_attr_init(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_init");
      return false;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);
    errno = pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set), &cpu_set);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setaffinity_np");
      return 1;
    }

    errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    sched_param sched_param
    {
      .sched_priority = 80
    };
    errno = pthread_attr_setschedparam(&pthread_attr, &sched_param);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedparam");
      return false;
    }

    errno = pthread_create(&pthread, &pthread_attr, &control_loop, this);
    if (errno != 0)
    {
      ROS_FATAL("pthread_create");
      return false;
    }

    errno = pthread_attr_destroy(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_destroy");
      return false;
    }

    return true;
  }

  /* */
  bool fault_reset();
  bool fault_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool ready_to_switch_on();
  bool ready_to_switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_on();
  bool switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_off();
  bool switch_off(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /**/
  bool enable_operation();
  bool enable_operation(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool start_homing();
  bool start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool halt();
  bool halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool quick_stop();
  bool quick_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool get_error_code(const uint16 slave_idx, uint16 &error_code);
  bool get_error_message(const uint16 &error_code, std::string &error_message);


  void read(const ros::Time &time, const ros::Duration &period)
  {

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      uint16 status_word = ec_master.tx_pdo[slave_idx].status_word;
      int32 actual_position = ec_master.tx_pdo[slave_idx].actual_position;

      status.ready_to_switch_on = (status_word >> 0) & 0x01;
      status.switched_on = (status_word >> 1) & 0x01;
      status.operation_enabled = (status_word >> 2) & 0x01;
      status.fault = (status_word >> 3) & 0x01;
      status.voltage_enabled = (status_word >> 4) & 0x01;
      status.quick_stop = (status_word >> 5) & 0x01;
      status.switch_on_disabled = (status_word >> 6) & 0x01;
      status.warning = (status_word >> 7) & 0x01;
      status.remote = (status_word >> 9) & 0x01;
      status.target_reached = (status_word >> 10) & 0x01;
      status.internal_limit_active = (status_word >> 11) & 0x01;
      
       if (!status.operation_enabled)
        {
            reset_controllers = true;
        }

      // switch (mode_of_operation_display)
      // {
      //   case delta::asda::ethercat::mode_of_operation_t::HOMING:
      //     status.homing_attained = (status_word >> 12) & 0x01;
      //     status.homing_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_TORQUE:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      // }
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    if (pulse.size() > 0) 
    {
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint32 target_position = pulse[i];
        ec_master.rx_pdo[slave_idx].target_position = target_position;
        ROS_ERROR("target_position = %d", pulse[i]);
      }
    }
  }


  void close()
  {
    ec_master.close();
    ROS_INFO("EtherCAT socket closed.");
  }

};


inline void* control_loop(void* arg)
{
  delta::asda::ServoHW* servo_hw = (delta::asda::ServoHW*)arg;
  servo_hw->reset_controllers = true;
  
  struct timespec t, t_1, t0_cmd;
  clock_gettime(CLOCK_MONOTONIC, &t);
  int step_position = 1000;

  int step_execution_time = 0;

  while (ros::ok())
  {
    delta::asda::ethercat::add_timespec(&t, servo_hw->ec_master.t_cycle + servo_hw->ec_master.t_off);

    struct timespec t_left;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);

    struct timespec t_period;
    delta::asda::ethercat::diff_timespec(t, t_1, &t_period);

    if (delta::asda::ethercat::to_nsec(t_period) < (servo_hw->ec_master.t_cycle - 100000) || (servo_hw->ec_master.t_cycle + 100000) < delta::asda::ethercat::to_nsec(t_period))
    {
      ROS_WARN("EtherCAT Master period: %lu [ns]", delta::asda::ethercat::to_nsec(t_period));
    }

    if (step_execution_time == 50)
    {
      if (servo_hw->ec_master.fault_reset())
      {
        ROS_INFO("Fault Reset SUCCESS");
      }
      else
      {
        ROS_ERROR("Fault Reset FAILURE");
        return 0;
      }
    }

    if (step_execution_time == 100)
    {
      std::cout << "Ready to Switch On ... ";
      if (servo_hw->ec_master.ready_to_switch_on())
      {
        ROS_INFO("Ready to Switch On SUCCESS");
      }
      else
      {
        ROS_ERROR("Ready to Switch On FAILURE");
        return 0;
      }
    }

    if (step_execution_time == 200)
    {
      std::cout << "Switch On ... ";
      if (servo_hw->ec_master.switch_on())
      {
        ROS_INFO("Switch On SUCCESS");
      }
      else
      {
        
        ROS_ERROR("Switch On FAILURE");
        return 0;
      }
    }

    if (step_execution_time == 400)
    {
      std::cout << "Enable Motion ... ";
      if (servo_hw->ec_master.enable_operation())
      {
        ROS_INFO("Enable Motion SUCCESS");
      }
      else
      {
        ROS_ERROR("Enable Motion FAILURE");
        return 0;
      }
    }

    if (step_execution_time == 500) t0_cmd = t;
    if (step_execution_time >= 500)
    {
      struct timespec t_cmd;
      delta::asda::ethercat::diff_timespec(t, t0_cmd, &t_cmd);

      const ros::Time now = ros::Time::now();
      const ros::Duration period(delta::asda::ethercat::to_sec(t_period));
      // servo_hw->read(now, period);
      // servo_hw->write(now, period);
    }

    if (step_execution_time <= 500) step_execution_time++;
    servo_hw->ec_master.update();
  
    t_1 = t;
  }

  return NULL;
}


} }  // namespace
#endif