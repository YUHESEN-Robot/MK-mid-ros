#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

namespace yhs_tool
{

  CanControl::CanControl()
  {
    ros::NodeHandle private_node("~");

    std::string ultrasonic_numbers_str;
    private_node.getParam("ultrasonic_number", ultrasonic_numbers_str);
    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
  	private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
  	private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
  	private_node.param("/yhs_can_control/wheel_base", wheel_base_, 0.6);
    private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));

    private_node.param("/yhs_can_control/io_cmd/enable", io_param_enable_, false);
    private_node.param("/yhs_can_control/io_cmd/lower_beam", io_param_lower_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/upper_beam", io_param_upper_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/turn_lamp", io_param_turn_lamp_, 0); // 0:off, 1:left, 2:right, 3:hazard
    private_node.param("/yhs_can_control/io_cmd/braking_lamp", io_param_braking_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/clearance_lamp", io_param_clearance_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/fog_lamp", io_param_fog_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/speaker", io_param_speaker_, false);
    private_node.param("/yhs_can_control/io_cmd/discharge", io_param_discharge_, false);

    current_io_cmd_.io_cmd_enable = io_param_enable_;
    current_io_cmd_.io_cmd_lower_beam_headlamp = io_param_lower_beam_;
    current_io_cmd_.io_cmd_upper_beam_headlamp = io_param_upper_beam_;
    current_io_cmd_.io_cmd_turn_lamp = io_param_turn_lamp_;
    current_io_cmd_.io_cmd_braking_lamp = io_param_braking_lamp_;
    current_io_cmd_.io_cmd_clearance_lamp = io_param_clearance_lamp_;
    current_io_cmd_.io_cmd_fog_lamp = io_param_fog_lamp_;
    current_io_cmd_.io_cmd_speaker = io_param_speaker_;
    current_io_cmd_.io_cmd_disCharge = io_param_discharge_;

    last_imu_time_ = ros::Time(0);

    std::istringstream iss(ultrasonic_numbers_str);
    int number;
    while (iss >> number)
    {
      ultrasonic_number_.push_back(number);
    }
  }

  CanControl::~CanControl()
  {
  }

  // io控制回调函数
  void CanControl::sendIoCommand()
  {
    static unsigned char count_1 = 0;
    unsigned char sendData_u_io[8];

    memset(sendData_u_io, 0, 8);

    sendData_u_io[0] = current_io_cmd_.io_cmd_enable;

    // 近光灯开关（预留）
    sendData_u_io[1] = 0;
    if (current_io_cmd_.io_cmd_lower_beam_headlamp)
      sendData_u_io[1] |= 0x01;

    // 远光灯开关（预留）
    if (current_io_cmd_.io_cmd_upper_beam_headlamp)
      sendData_u_io[1] |= 0x02;

    sendData_u_io[1] |= (current_io_cmd_.io_cmd_turn_lamp & 0x03) << 2; 

    // 制动灯开关（预留）
    if (current_io_cmd_.io_cmd_braking_lamp)
      sendData_u_io[1] |= 0x10;

    // 示廓灯开关（预留）
    if (current_io_cmd_.io_cmd_clearance_lamp) 
      sendData_u_io[1] |= 0x20;

    // 雾灯开关（预留） 
    if (current_io_cmd_.io_cmd_fog_lamp) 
      sendData_u_io[1] |= 0x40;

    // 扬声器开关（预留） 
    if (current_io_cmd_.io_cmd_speaker)
      sendData_u_io[2] |= 0x01;

    sendData_u_io[3] = 0;
    sendData_u_io[4] = 0;
    
    // 放电控制
    if (current_io_cmd_.io_cmd_disCharge)
      sendData_u_io[5] |= 0x01;

    count_1++;
    if (count_1 == 16)
      count_1 = 0;

    sendData_u_io[6] = count_1 << 4;

    sendData_u_io[7] = sendData_u_io[0] ^ sendData_u_io[1] ^ sendData_u_io[2] ^ sendData_u_io[3] ^ sendData_u_io[4] ^ sendData_u_io[5] ^ sendData_u_io[6];

    send_frames_[0].can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;

    memcpy(send_frames_[0].data, sendData_u_io, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0)
    {
      ROS_ERROR("send message failed, error code: %d", ret);
    }

    cmd_mutex_.unlock();
  }

  void CanControl::sendCtrlCommand()
  {
    static unsigned char count_ctrl = 0;
    unsigned char sendData_u_vel[8];
    memset(sendData_u_vel, 0, 8);

    unsigned short vel = 0;
    // 协议: 0.001 m/s -> *1000
    if (current_ctrl_cmd_.ctrl_cmd_velocity < 0) 
      vel = 0;
    else 
      vel = (unsigned short)(current_ctrl_cmd_.ctrl_cmd_velocity * 1000);

    // 协议: 0.01 deg -> *100
    short angular = (short)(current_ctrl_cmd_.ctrl_cmd_steering * 100);

    // Byte 0
    sendData_u_vel[0] = (0x0f & current_ctrl_cmd_.ctrl_cmd_gear);
    sendData_u_vel[0] |= (unsigned char)((vel & 0x0f) << 4);

    // Byte 1
    sendData_u_vel[1] = (unsigned char)((vel >> 4) & 0xff);

    // Byte 2
    sendData_u_vel[2] = (unsigned char)((vel >> 12) & 0x0f);
    sendData_u_vel[2] |= (unsigned char)((angular & 0x0f) << 4);

    // Byte 3
    sendData_u_vel[3] = (unsigned char)((angular >> 4) & 0xff);

    // Byte 4
    sendData_u_vel[4] = (unsigned char)((angular >> 12) & 0x0f);
    sendData_u_vel[4] |= (unsigned char)((current_ctrl_cmd_.ctrl_cmd_Brake & 0x0f) << 4);

    // Byte 5
    sendData_u_vel[5] = (unsigned char)((current_ctrl_cmd_.ctrl_cmd_Brake >> 4) & 0x0f);

    // Byte 6: Counter
    count_ctrl++;
    if (count_ctrl > 15) 
      count_ctrl = 0;

    sendData_u_vel[6] = count_ctrl << 4;

    // Byte 7: BCC
    sendData_u_vel[7] = sendData_u_vel[0] ^ sendData_u_vel[1] ^ sendData_u_vel[2] ^ 
                        sendData_u_vel[3] ^ sendData_u_vel[4] ^ sendData_u_vel[5] ^ sendData_u_vel[6];

    send_frames_[0].can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) ROS_ERROR_THROTTLE(1, "Send Ctrl ID failed: %d", ret);
  }

  // IO回调：更新状态并立即发送
  void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_io_cmd_ = *msg; // 更新当前状态
    sendIoCommand();
  }

  // 速度控制回调函数
  void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_ctrl_cmd_ = *msg;
  }

  // 数据接收解析线程
  void CanControl::recvData()
  {

    while (ros::ok())
    {

      if (read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
      {
        for (int j = 0; j < 1; j++)
        {

          switch (recv_frames_[0].can_id)
          {
          // 速度控制反馈
          case 0x18C4D2EF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::ctrl_fb msg;
            msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];

            msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;

            msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

            // 当前车辆制动状态反馈（预留）
            msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;

            msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

            msg.ctrl_fb_RemoteSt = (recv_frames_[0].data[5] & 0x01) >> 7;

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {
              if (msg.ctrl_fb_gear == 2)
							msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
				  		OdomPub(msg.ctrl_fb_velocity, msg.ctrl_fb_steering / 180 * 3.1415);
              ctrl_fb_pub_.publish(msg);
            }

            break;
          }

          // 左轮反馈
          case 0x18C4D7EF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::lr_wheel_fb msg;
            msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

            msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              lr_wheel_fb_pub_.publish(msg);
            }

            break;
          }

          // 右轮反馈
          case 0x18C4D8EF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::rr_wheel_fb msg;
            msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

            msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              rr_wheel_fb_pub_.publish(msg);
            }

            break;
          }

          // io反馈
          case 0x18C4DAEF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::io_fb msg;
            if (0x01 & recv_frames_[0].data[0])
              msg.io_fb_enable = true;
            else
              msg.io_fb_enable = false;

            // 近光灯开关状态反馈（预留）
            if (0x01 & recv_frames_[0].data[1])
              msg.io_fb_lower_beam_headlamp = true;
            else
              msg.io_fb_lower_beam_headlamp = false;

            // 远光灯开关状态反馈（预留）
            if (0x02 & recv_frames_[0].data[1])
              msg.io_fb_upper_beam_headlamp = true;
            else
              msg.io_fb_upper_beam_headlamp = false;

            msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

            if (0x10 & recv_frames_[0].data[1])  
              msg.io_fb_braking_lamp = true;
            else
              msg.io_fb_braking_lamp = false;

            // 示廓灯开关状态反馈（预留）  
            if (0x20 & recv_frames_[0].data[1])
              msg.io_fb_clearance_lamp = true;
            else
              msg.io_fb_clearance_lamp = false;

            // 雾灯开关状态反馈（预留）  
            if (0x40 & recv_frames_[0].data[1])
              msg.io_fb_fog_lamp = true;
            else
              msg.io_fb_fog_lamp = false;

            // 扬声器开关状态反馈（预留）  
            if (0x01 & recv_frames_[0].data[2])
              msg.io_fb_speaker = true;
            else
              msg.io_fb_speaker = false;

            // 前左防撞条开关状态反馈(预留)  
            if (0x01 & recv_frames_[0].data[3])
              msg.io_fb_fl_impact_sensor = true;
            else
              msg.io_fb_fl_impact_sensor = false;

            if (0x02 & recv_frames_[0].data[3])
              msg.io_fb_fm_impact_sensor = true;
            else
              msg.io_fb_fm_impact_sensor = false;

            // 前右防撞条开关状态反馈（预留）  
            if (0x04 & recv_frames_[0].data[3])
              msg.io_fb_fr_impact_sensor = true;
            else
              msg.io_fb_fr_impact_sensor = false;

            // 后左防撞条开关状态反馈（预留）  
            if (0x08 & recv_frames_[0].data[3])
              msg.io_fb_rl_impact_sensor = true;
            else
              msg.io_fb_rl_impact_sensor = false;

            if (0x10 & recv_frames_[0].data[3])
              msg.io_fb_rm_impact_sensor = true;
            else
              msg.io_fb_rm_impact_sensor = false;

            // 后右防撞条开关状态反馈（预留）
            if (0x20 & recv_frames_[0].data[3])
              msg.io_fb_rr_impact_sensor = true;
            else
              msg.io_fb_rr_impact_sensor = false;              

            // 前左跌落传感器状态反馈（预留）  
            if (0x01 & recv_frames_[0].data[4])
              msg.io_fb_fl_drop_sensor = true;
            else
              msg.io_fb_fl_drop_sensor = false;

            // 前中跌落传感器状态反馈（预留）
            if (0x02 & recv_frames_[0].data[4])
              msg.io_fb_fm_drop_sensor = true;
            else
              msg.io_fb_fm_drop_sensor = false;

            // 前右跌落传感器状态反馈（预留）
            if (0x04 & recv_frames_[0].data[4])
              msg.io_fb_fr_drop_sensor = true;
            else
              msg.io_fb_fr_drop_sensor = false;

            // 后左跌落传感器状态反馈（预留）
            if (0x08 & recv_frames_[0].data[4])
              msg.io_fb_rl_drop_sensor = true;
            else
              msg.io_fb_rl_drop_sensor = false;

            // 后中跌落传感器状态反馈（预留）
            if (0x10 & recv_frames_[0].data[4])
              msg.io_fb_rm_drop_sensor = true;
            else
              msg.io_fb_rm_drop_sensor = false;

            // 后右跌落传感器状态反馈（预留）
            if (0x20 & recv_frames_[0].data[4])
              msg.io_fb_rr_drop_sensor = true;
            else
              msg.io_fb_rr_drop_sensor = false; 

            msg.io_fb_disCharge = 0x01 & recv_frames_[0].data[5];

            msg.io_fb_chargeEn = 0x02 & recv_frames_[0].data[1];

            if (0x10 & recv_frames_[0].data[5])
              msg.io_fb_ScramSt = true;
            else
              msg.io_fb_ScramSt = false;

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              io_fb_pub_.publish(msg);
            }

            break;
          }

          // 里程计反馈
          case 0x18C4DEEF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::odo_fb msg;
            msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

            // 累计角度（预留）
            msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

            odo_fb_pub_.publish(msg);

            break;
          }

          // bms_Infor反馈
          case 0x18C4E1EF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::bms_Infor msg;
            msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

            msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

            msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              bms_Infor_pub_.publish(msg);
            }

            break;
          }

          // bms_flag_Infor反馈
          case 0x18C4E2EF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::bms_flag_Infor msg;
            msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

            if (0x01 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_single_ov = true;
            else
              msg.bms_flag_Infor_single_ov = false;

            if (0x02 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_single_uv = true;
            else
              msg.bms_flag_Infor_single_uv = false;

            if (0x04 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_ov = true;
            else
              msg.bms_flag_Infor_ov = false;

            if (0x08 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_uv = true;
            else
              msg.bms_flag_Infor_uv = false;

            if (0x10 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_charge_ot = true;
            else
              msg.bms_flag_Infor_charge_ot = false;

            if (0x20 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_charge_ut = true;
            else
              msg.bms_flag_Infor_charge_ut = false;

            if (0x40 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_discharge_ot = true;
            else
              msg.bms_flag_Infor_discharge_ot = false;

            if (0x80 & recv_frames_[0].data[1])
              msg.bms_flag_Infor_discharge_ut = true;
            else
              msg.bms_flag_Infor_discharge_ut = false;

            if (0x01 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_charge_oc = true;
            else
              msg.bms_flag_Infor_charge_oc = false;

            if (0x02 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_discharge_oc = true;
            else
              msg.bms_flag_Infor_discharge_oc = false;

            if (0x04 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_short = true;
            else
              msg.bms_flag_Infor_short = false;

            if (0x08 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_ic_error = true;
            else
              msg.bms_flag_Infor_ic_error = false;

            if (0x10 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_lock_mos = true;
            else
              msg.bms_flag_Infor_lock_mos = false;

            msg.bms_flag_Infor_charge_st = (recv_frames_[0].data[2] >> 5) & 0x03;

            if (0x80 & recv_frames_[0].data[2])
              msg.bms_flag_Infor_SOCWarning = true;
            else
              msg.bms_flag_Infor_SOCWarning = false;

            if (0x01 & recv_frames_[0].data[3])
              msg.bms_flag_Infor_SOCLowProtection = true;
            else
              msg.bms_flag_Infor_SOCLowProtection = false;

            msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

            msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              bms_flag_Infor_pub_.publish(msg);
            }

            break;
          }

          // Veh_fb_Diag反馈
          case 0x18C4EAEF | CAN_EFF_FLAG:
          {
            yhs_can_msgs::Veh_Diag_fb msg;
            msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

            if (0x10 & recv_frames_[0].data[0])
              msg.Veh_fb_AutoCANCtrlCmd = true;
            else
              msg.Veh_fb_AutoCANCtrlCmd = false;

            if (0x20 & recv_frames_[0].data[0])
              msg.Veh_fb_AutoIOCANCmd = true;
            else
              msg.Veh_fb_AutoIOCANCmd = false;

            if (0x01 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSDisOnline = true;
            else
              msg.Veh_fb_EPSDisOnline = false;

            if (0x02 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSfault = true;
            else
              msg.Veh_fb_EPSfault = false;

            if (0x04 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSMosfetOT = true;
            else
              msg.Veh_fb_EPSMosfetOT = false;

            if (0x08 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSWarning = true;
            else
              msg.Veh_fb_EPSWarning = false;

            if (0x10 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSDisWork = true;
            else
              msg.Veh_fb_EPSDisWork = false;

            if (0x20 & recv_frames_[0].data[1])
              msg.Veh_fb_EPSOverCurrent = true;
            else
              msg.Veh_fb_EPSOverCurrent = false;

            // 转向系统故障预留
            msg.Veh_fb_STReserve = ( (recv_frames_[0].data[1] & 0x03 ) << 4) | (recv_frames_[0].data[2] & 0x0f);

            if (0x10 & recv_frames_[0].data[2])
              msg.Veh_fb_EHBecuFault = true;
            else
              msg.Veh_fb_EHBecuFault = false;

            if (0x20 & recv_frames_[0].data[2])
              msg.Veh_fb_EHBDisOnline = true;
            else
              msg.Veh_fb_EHBDisOnline = false;

            if (0x40 & recv_frames_[0].data[2])
              msg.Veh_fb_EHBWorkModelFault = true;
            else
              msg.Veh_fb_EHBWorkModelFault = false;

            if (0x80 & recv_frames_[0].data[2])
              msg.Veh_fb_EHBDisEn = true;
            else
              msg.Veh_fb_EHBDisEn = false;

            if (0x01 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBAnguleFault = true;
            else
              msg.Veh_fb_EHBAnguleFault = false;

            if (0x02 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBOT = true;
            else
              msg.Veh_fb_EHBOT = false;

            if (0x04 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBPowerFault = true;
            else
              msg.Veh_fb_EHBPowerFault = false;

            if (0x08 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBsensorAbnomal = true;
            else
              msg.Veh_fb_EHBsensorAbnomal = false;

            if (0x10 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBMotorFault = true;
            else
              msg.Veh_fb_EHBMotorFault = false;

            if (0x20 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBOilPressSensorFault = true;
            else
              msg.Veh_fb_EHBOilPressSensorFault = false;

            if (0x40 & recv_frames_[0].data[3])
              msg.Veh_fb_EHBOilFault = true;
            else
              msg.Veh_fb_EHBOilFault = false;

            // 制动系统故障预留
            if (0x80 & recv_frames_[0].data[3])
              msg.Veh_fb_BraReserve = true;
            else
              msg.Veh_fb_BraReserve = false;

            msg.Veh_fb_LDrvMCUFault = 0x3f & recv_frames_[0].data[4];
            msg.Veh_fb_RDrvMCUFault = (recv_frames_[0].data[5] & 0x0f << 2) | (recv_frames_[0].data[4] >> 6);

            if (0x10 & recv_frames_[0].data[5])
              msg.Veh_fb_AUXBMSDisOnline = true;
            else
              msg.Veh_fb_AUXBMSDisOnline = false;

            if (0x80 & recv_frames_[0].data[5])
              msg.Veh_fb_AuxRemoteDisOnline = true;
            else
              msg.Veh_fb_AuxRemoteDisOnline = false;

            // 辅件故障预留
            msg.Veh_fb_AuxReserve = recv_frames_[0].data[6] & 0x0f;

            unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

            if (crc == recv_frames_[0].data[7])
            {

              Veh_Diag_fb_pub_.publish(msg);
            }

            break;
          }

            // ultrasonic预留
            static unsigned short ultra_data[8] = {0};
          case 0x18C4E8EF | CAN_EFF_FLAG:
          {
            ultra_data[0] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
            ultra_data[1] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));

            ultra_data[2] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
            ultra_data[3] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));
            break;
          }

          case 0x18C4E9EF | CAN_EFF_FLAG:
          {
            ultra_data[4] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
            ultra_data[5] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));

            ultra_data[6] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
            ultra_data[7] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));

            yhs_can_msgs::ultrasonic ultra_msg;

            ultra_msg.ultrasonic_fb_01 = ultra_data[ultrasonic_number_[0]];
            ultra_msg.ultrasonic_fb_02 = ultra_data[ultrasonic_number_[1]];
            ultra_msg.ultrasonic_fb_03 = ultra_data[ultrasonic_number_[2]];
            ultra_msg.ultrasonic_fb_04 = ultra_data[ultrasonic_number_[3]];

            ultra_msg.ultrasonic_fb_05 = ultra_data[ultrasonic_number_[4]];
            ultra_msg.ultrasonic_fb_06 = ultra_data[ultrasonic_number_[5]];
            ultra_msg.ultrasonic_fb_07 = ultra_data[ultrasonic_number_[6]];
            ultra_msg.ultrasonic_fb_08 = ultra_data[ultrasonic_number_[7]];

            ultrasonic_pub_.publish(ultra_msg);
          }

          default:
            break;
          }
        }
      }
    }
  }

  void CanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
      std::lock_guard<std::mutex> lock(mutex_);

      last_imu_time_ = ros::Time::now(); 

      tf2::Quaternion quaternion;
      tf2::fromMsg(imu_data_msg->orientation, quaternion);
      tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void CanControl::OdomPub(const float velocity,const float steering)
  {
  	static double x = 0.0;
  	static double y = 0.0;
  	static double th = 0.0;
  
  	double x_mid = 0.0;
  	double y_mid = 0.0;
  
  	static tf2_ros::TransformBroadcaster odom_broadcaster;
  
  	static ros::Time last_time = ros::Time::now();
  	ros::Time current_time;
  
  	double vx = velocity;
  	double vth = vx * tan(steering) / wheel_base_;
  
  	current_time = ros::Time::now();
  
  	double dt = (current_time - last_time).toSec();
    bool is_imu_active = (current_time - last_imu_time_).toSec() < 0.2;
    if (is_imu_active)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_; 
    }
    else
    {
        th += vth * dt; 
    }
  
  	double delta_x = (vx * cos(th)) * dt;
  	double delta_y = (vx * sin(th)) * dt;
  
  	x += delta_x;
  	y += delta_y;
  
  	x_mid = x + wheel_base_ / 2 * cos(th);
  	y_mid = y + wheel_base_ / 2 * sin(th);

    tf2::Quaternion quat;
    if (is_imu_active) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat.setRPY(imu_roll_, imu_pitch_, th);
    } else {
        quat.setRPY(0, 0, th);
    }
  
  	geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrame_;
    odom_trans.child_frame_id = baseFrame_;

    odom_trans.transform.translation.x = x_mid;
    odom_trans.transform.translation.y = y_mid;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
  
  	//是否发布tf转换
  	if(tfUsed_)
  		odom_broadcaster.sendTransform(odom_trans);
  
  	//next, we'll publish the odometry message over ROS
  	nav_msgs::Odometry odom;
  	odom.header.stamp = current_time;
  	odom.header.frame_id = odomFrame_;
  
  	//set the position
  	odom.pose.pose.position.x = x_mid;
  	odom.pose.pose.position.y = y_mid;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = odom_quat;
  
  	//set the velocity
  	odom.child_frame_id = baseFrame_;
  	odom.twist.twist.linear.x = vx;
  	odom.twist.twist.linear.y = 0.0;
  	odom.twist.twist.angular.z = vth;
    
    // 协方差矩阵设置
  	odom.pose.covariance[0]  = 0.1;   	
  	odom.pose.covariance[7]  = 0.1;		
  	odom.pose.covariance[35] = 0.2;   	
  
  	odom.pose.covariance[14] = 1e10; 	
  	odom.pose.covariance[21] = 1e10; 	
  	odom.pose.covariance[28] = 1e10; 	
  
  	//publish the message
  	odom_pub_.publish(odom);
  
  	last_time = current_time;
  
  }

  void CanControl::timerCallBack(const ros::TimerEvent& event)
  {
      static int loop_count = 0;
      boost::mutex::scoped_lock lock(cmd_mutex_);
      sendCtrlCommand(); 
      if (loop_count % 5 == 0) {
          sendIoCommand();
      }

      loop_count++;
  }

  void CanControl::run()
  {

    ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &CanControl::ImuDataCallBack, this);

    ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5);
    lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb", 5);
    rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb", 5);
    io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb", 5);
    odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb", 5);
    bms_Infor_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor>("bms_Infor", 5);
    bms_flag_Infor_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor>("bms_flag_Infor", 5);
    Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

    // 打开设备
    dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (dev_handler_ < 0)
    {
      ROS_ERROR(">>open can deivce error!");
      return;
    }
    else
    {
      ROS_INFO(">>open can deivce success!");
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, if_name_.c_str());

    ioctl(dev_handler_, SIOCGIFINDEX, &ifr);

    // bind socket to network interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (ret < 0)
    {
      ROS_ERROR(">>bind dev_handler error!\r\n");
      return;
    }

    boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));

    timer_ = nh_.createTimer(ros::Duration(0.01), &CanControl::timerCallBack, this);

    ros::spin();

    close(dev_handler_);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yhs_can_control_node");

  yhs_tool::CanControl cancontrol;
  cancontrol.run();

  return 0;
}
