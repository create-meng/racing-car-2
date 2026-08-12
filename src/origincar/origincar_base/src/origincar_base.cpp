#include "origincar_base/origincar_base.h"
#include "rclcpp/rclcpp.hpp"
#include "origincar_base/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "origincar_msg/msg/data.hpp"
#include <thread>

using std::placeholders::_1;
using namespace std;
sensor_msgs::msg::Imu Mpu6050;
rclcpp::Node::SharedPtr node_handle = nullptr;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot_control = std::make_shared<origincar_base>();
    std::thread control_thread([robot_control]() {
      robot_control->Control();
    });

    rclcpp::spin(robot_control);

    if (control_thread.joinable()) {
      control_thread.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
}

short origincar_base::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    const uint16_t raw = (static_cast<uint16_t>(Data_High) << 8) |
                         static_cast<uint16_t>(Data_Low);
    return static_cast<short>(static_cast<int16_t>(raw));
}

float origincar_base::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    const uint16_t raw = (static_cast<uint16_t>(Data_High) << 8) |
                         static_cast<uint16_t>(Data_Low);
    return static_cast<float>(static_cast<int16_t>(raw)) *
           static_cast<float>(odom_velocity_scale_mps_per_lsb_);
  }

void origincar_base::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
    {
      std::lock_guard<std::mutex> lock(cmd_state_mutex_);
      last_cmd_received_at_ = std::chrono::steady_clock::now();
      last_cmd_nonzero_ = std::abs(akm_ctl->drive.speed) > 1e-4 ||
                          std::abs(akm_ctl->drive.steering_angle) > 1e-4;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    short  transition;
  
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = akm_ctl->drive.speed*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = akm_ctl->drive.steering_angle*1000/2;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); 
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void origincar_base::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    {
      std::lock_guard<std::mutex> lock(cmd_state_mutex_);
      last_cmd_received_at_ = std::chrono::steady_clock::now();
      last_cmd_nonzero_ = std::abs(twist_aux->linear.x) > 1e-4 ||
                          std::abs(twist_aux->linear.y) > 1e-4 ||
                          std::abs(twist_aux->angular.z) > 1e-4;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    short  transition;
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = twist_aux->linear.x*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = twist_aux->linear.y*1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition>>8;

    transition=0;
    transition = (twist_aux->angular.z)*1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      if (akm_cmd_vel == "none") {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
      } 
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void origincar_base::Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch)
{
  (void)sign_switch;
    /* if (sign_switch->data == -1) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 0.5;
         Robot_Pos.Y = 0.2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }
     else if (sign_switch->data == 6) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 2;
         Robot_Pos.Y = 2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }*/
}

void origincar_base::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 
                                                  
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6; 
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e6;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    // Do not make one raw gyro sample infinitely authoritative; a serial
    // glitch must be rejected by robot_localization rather than becoming a
    // visible yaw jump.
    Imu_Data_Pub.angular_velocity_covariance[8] = 0.01;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    Imu_Data_Pub.linear_acceleration_covariance[0] = 0.04;
    Imu_Data_Pub.linear_acceleration_covariance[4] = 0.04;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 0.04;

    imu_publisher->publish(Imu_Data_Pub);

}

void origincar_base::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    origincar_msg::msg::Data robotpose;
    origincar_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;

    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;


    odom.twist.twist.linear.x =  Robot_Vel.X;
    odom.twist.twist.linear.y =  Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z; 
    // The EKF consumes only vx/vy from this message.  Keep covariance
    // explicit so a future config cannot accidentally treat the raw wheel
    // pose or wheel yaw as a high-confidence world-frame measurement.
    std::copy(std::begin(odom_pose_covariance), std::end(odom_pose_covariance),
              odom.pose.covariance.begin());
    std::copy(std::begin(odom_twist_covariance), std::end(odom_twist_covariance),
              odom.twist.covariance.begin());

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    odom_publisher->publish(odom);
    robotpose_publisher->publish(robotpose);
    robotvel_publisher->publish(robotvel); 
}

void origincar_base::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }
}

unsigned char origincar_base::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0) {
      for(k=0; k < Count_Number; k++) {
        check_sum = check_sum^Receive_Data.rx[k];
      }
    } else if (mode == 1) {
      for (k=0; k < Count_Number; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    }

    return check_sum;
}

bool origincar_base::Get_Sensor_Data()
{
    // Command callbacks and this reader share one physical UART.  Serial
    // reads/writes must not overlap once S1 begins publishing /cmd_vel.
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!Stm32_Serial.isOpen()) {
      return false;
    }

    // Serial reads may return a short chunk at any byte boundary.  Keep the
    // stream between calls and extract only complete, checked 24-byte frames.
    // This prevents one short read from shifting vx/vy/gyro fields forever.
    try {
      const size_t available = Stm32_Serial.available();
      if (available == 0) {
        return false;
      }
      const size_t request = std::min<size_t>(available, 256);
      uint8_t chunk[256];
      const size_t received = Stm32_Serial.read(chunk, request);
      if (received == 0) {
        return false;
      }
      serial_rx_buffer_.insert(serial_rx_buffer_.end(), chunk, chunk + received);
    } catch (const std::exception&) {
      return false;
    }

    bool frame_valid = false;
    bool invalid_frame = false;
    while (serial_rx_buffer_.size() >= RECEIVE_DATA_SIZE) {
      const auto header = std::find(serial_rx_buffer_.begin(), serial_rx_buffer_.end(), FRAME_HEADER);
      if (header == serial_rx_buffer_.end()) {
        // Retain only a possible partial frame suffix; no valid header was
        // present before it, so older bytes cannot form a frame later.
        if (serial_rx_buffer_.size() > RECEIVE_DATA_SIZE - 1) {
          invalid_frame = true;
          serial_rx_buffer_.erase(
              serial_rx_buffer_.begin(),
              serial_rx_buffer_.end() - static_cast<std::ptrdiff_t>(RECEIVE_DATA_SIZE - 1));
        }
        break;
      }
      if (header != serial_rx_buffer_.begin()) {
        invalid_frame = true;
        serial_rx_buffer_.erase(serial_rx_buffer_.begin(), header);
      }
      if (serial_rx_buffer_.size() < RECEIVE_DATA_SIZE) {
        break;
      }
      if (serial_rx_buffer_[RECEIVE_DATA_SIZE - 1] != FRAME_TAIL) {
        invalid_frame = true;
        serial_rx_buffer_.erase(serial_rx_buffer_.begin());
        continue;
      }
      uint8_t checksum = 0;
      for (size_t i = 0; i < RECEIVE_DATA_SIZE - 2; ++i) {
        checksum ^= serial_rx_buffer_[i];
      }
      if (serial_rx_buffer_[RECEIVE_DATA_SIZE - 2] != checksum) {
        invalid_frame = true;
        serial_rx_buffer_.erase(serial_rx_buffer_.begin());
        continue;
      }

      std::copy_n(serial_rx_buffer_.begin(), RECEIVE_DATA_SIZE, Receive_Data.rx);
      serial_rx_buffer_.erase(
          serial_rx_buffer_.begin(),
          serial_rx_buffer_.begin() + static_cast<std::ptrdiff_t>(RECEIVE_DATA_SIZE));
      frame_valid = true;
      // Drain any additional complete frames, but decode/publish only the
      // newest one so a read backlog does not integrate old samples at dt=0.
    }
    if (!frame_valid) {
      if (!invalid_frame) {
        return false;
      }
      ++invalid_frame_count_;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "serial sensor frame unavailable: buffered_bytes=%zu invalid_frames=%zu",
          serial_rx_buffer_.size(), invalid_frame_count_);
      return false;
    }

    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];
    if (Receive_Data.Frame_Header == FRAME_HEADER &&
        Receive_Data.Frame_Tail == FRAME_TAIL) {
          Receive_Data.Flag_Stop=Receive_Data.rx[1];
          Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);
        
          Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);
                                                                          
          Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); 

          const double planar_speed = std::hypot(Robot_Vel.X, Robot_Vel.Y);
          if (!std::isfinite(planar_speed) || planar_speed > odom_max_valid_speed_mps_) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "rejecting impossible raw wheel frame vx=%.3f vy=%.3f speed=%.3f limit=%.3f m/s",
                Robot_Vel.X, Robot_Vel.Y, planar_speed, odom_max_valid_speed_mps_);
            Robot_Vel.X = 0.0f;
            Robot_Vel.Y = 0.0f;
            Robot_Vel.Z = 0.0f;
            return false;
          }

          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);
          Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);
          Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);
          Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);

          Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data /
                                          imu_accel_scale_lsb_per_mps2_;
          Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data /
                                          imu_accel_scale_lsb_per_mps2_;
          Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data /
                                          imu_accel_scale_lsb_per_mps2_;

          Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * imu_gyro_scale_rad_s_per_lsb_;
          Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * imu_gyro_scale_rad_s_per_lsb_;
          Update_Imu_Gyro_Calibration();
          Mpu6050.angular_velocity.z =
              Mpu6050_Data.gyros_z_data * imu_gyro_scale_rad_s_per_lsb_ - imu_gyro_bias_z_rad_s_;

          const uint16_t voltage_raw = (static_cast<uint16_t>(Receive_Data.rx[20]) << 8) |
                                       static_cast<uint16_t>(Receive_Data.rx[21]);
          Power_voltage = static_cast<float>(voltage_raw) *
                          static_cast<float>(voltage_scale_v_per_lsb_);

          // Log the first valid frame as well as low-rate samples.  Waiting
          // for frame 200 hid a dead/contended serial port during startup.
          ++count_;
          if (count_ == 1 || count_ % 200 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "sensor mapping sample #%zu raw_odom=(%d,%d,%d) decoded=(%.3f,%.3f,%.3f) "
                        "raw_gyro=(%d,%d,%d) decoded_z=%.5f raw_accel=(%d,%d,%d)",
                        count_,
                        static_cast<int>(static_cast<int16_t>(
                            (static_cast<uint16_t>(Receive_Data.rx[2]) << 8) | Receive_Data.rx[3])),
                        static_cast<int>(static_cast<int16_t>(
                            (static_cast<uint16_t>(Receive_Data.rx[4]) << 8) | Receive_Data.rx[5])),
                        static_cast<int>(static_cast<int16_t>(
                            (static_cast<uint16_t>(Receive_Data.rx[6]) << 8) | Receive_Data.rx[7])),
                        Robot_Vel.X, Robot_Vel.Y, Robot_Vel.Z,
                        Mpu6050_Data.gyros_x_data, Mpu6050_Data.gyros_y_data,
                        Mpu6050_Data.gyros_z_data, Mpu6050.angular_velocity.z,
                        Mpu6050_Data.accele_x_data, Mpu6050_Data.accele_y_data,
                        Mpu6050_Data.accele_z_data);
          }

          return true;
    }

    return false;
}

void origincar_base::Control()
{
    rclcpp::Time current_time, last_time;
    current_time = rclcpp::Node::now();
    last_time = rclcpp::Node::now();
    while(rclcpp::ok()) {
      current_time = rclcpp::Node::now();
      Sampling_Time = (current_time - last_time).seconds();
      const bool sensor_updated = Get_Sensor_Data();
      if (sensor_updated) {
        // The serial read can block while SIGINT invalidates the ROS context.
        if (!rclcpp::ok()) {
          break;
        }
        if (Sampling_Time > odom_max_integration_dt_sec_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "dropping stale odom integration gap %.3fs", Sampling_Time);
          Robot_Vel.X = 0.0f;
          Robot_Vel.Y = 0.0f;
          Robot_Vel.Z = 0.0f;
        } else {
          Robot_Pos.X += odom_world_x_scale_ *
              (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
          Robot_Pos.Y += odom_world_y_scale_ *
              (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
          Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;
        }

        Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                  Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
        Publish_ImuSensor();
        Publish_Voltage();
        Publish_Odom();
        last_time = current_time;
      }
      Check_Cmd_Vel_Watchdog();
      if (!sensor_updated && serial_idle_sleep_ms_ > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(serial_idle_sleep_ms_));
      }
    }
    Send_Stop_Command();
}

origincar_base::origincar_base()
: rclcpp::Node ("origincar_base")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200;

  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<bool>("cmd_vel_watchdog_enabled", true);
  this->declare_parameter<double>("cmd_vel_watchdog_timeout_sec", 0.35);
  this->declare_parameter<int>("serial_idle_sleep_ms", 1);
  this->declare_parameter<double>("imu_gyro_scale_rad_s_per_lsb", 0.00026644);
  this->declare_parameter<double>("imu_accel_scale_lsb_per_mps2", 1671.84);
  this->declare_parameter<double>("imu_gyro_bias_z_rad_s", 0.0);
  this->declare_parameter<double>("odom_velocity_scale_mps_per_lsb", 0.001);
  this->declare_parameter<double>("odom_world_x_scale", 0.926);
  this->declare_parameter<double>("odom_world_y_scale", 1.0);
  this->declare_parameter<double>("odom_max_integration_dt_sec", 0.25);
  this->declare_parameter<double>("odom_max_valid_speed_mps", 1.20);
  this->declare_parameter<double>("voltage_scale_v_per_lsb", 0.001);
  this->declare_parameter<bool>("imu_gyro_auto_calibration_enabled", true);
  this->declare_parameter<int>("imu_gyro_calibration_samples", 200);
  this->declare_parameter<double>("imu_gyro_calibration_max_speed_mps", 0.02);
  this->declare_parameter<double>("imu_gyro_calibration_max_yaw_rate_rad_s", 0.04);

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("cmd_vel_watchdog_enabled", cmd_vel_watchdog_enabled_);
  this->get_parameter("cmd_vel_watchdog_timeout_sec", cmd_vel_watchdog_timeout_sec_);
  this->get_parameter("serial_idle_sleep_ms", serial_idle_sleep_ms_);
  this->get_parameter("imu_gyro_scale_rad_s_per_lsb", imu_gyro_scale_rad_s_per_lsb_);
  this->get_parameter("imu_accel_scale_lsb_per_mps2", imu_accel_scale_lsb_per_mps2_);
  this->get_parameter("imu_gyro_bias_z_rad_s", imu_gyro_bias_z_rad_s_);
  this->get_parameter("odom_velocity_scale_mps_per_lsb", odom_velocity_scale_mps_per_lsb_);
  this->get_parameter("odom_world_x_scale", odom_world_x_scale_);
  this->get_parameter("odom_world_y_scale", odom_world_y_scale_);
  this->get_parameter("odom_max_integration_dt_sec", odom_max_integration_dt_sec_);
  this->get_parameter("odom_max_valid_speed_mps", odom_max_valid_speed_mps_);
  this->get_parameter("voltage_scale_v_per_lsb", voltage_scale_v_per_lsb_);
  this->get_parameter("imu_gyro_auto_calibration_enabled", imu_gyro_auto_calibration_enabled_);
  this->get_parameter("imu_gyro_calibration_samples", imu_gyro_calibration_samples_);
  this->get_parameter("imu_gyro_calibration_max_speed_mps", imu_gyro_calibration_max_speed_mps_);
  this->get_parameter("imu_gyro_calibration_max_yaw_rate_rad_s", imu_gyro_calibration_max_yaw_rate_rad_s_);
  cmd_vel_watchdog_timeout_sec_ = std::max(0.05, cmd_vel_watchdog_timeout_sec_);
  serial_idle_sleep_ms_ = std::max(0, std::min(10, serial_idle_sleep_ms_));
  imu_gyro_scale_rad_s_per_lsb_ = std::max(1e-8, imu_gyro_scale_rad_s_per_lsb_);
  imu_accel_scale_lsb_per_mps2_ = std::max(1e-8, imu_accel_scale_lsb_per_mps2_);
  odom_velocity_scale_mps_per_lsb_ = std::max(1e-8, odom_velocity_scale_mps_per_lsb_);
  odom_world_x_scale_ = std::max(0.0, odom_world_x_scale_);
  odom_world_y_scale_ = std::max(0.0, odom_world_y_scale_);
  odom_max_integration_dt_sec_ = std::max(0.01, odom_max_integration_dt_sec_);
  odom_max_valid_speed_mps_ = std::max(0.1, odom_max_valid_speed_mps_);
  voltage_scale_v_per_lsb_ = std::max(1e-8, voltage_scale_v_per_lsb_);
  imu_gyro_calibration_samples_ = std::max(1, imu_gyro_calibration_samples_);
  imu_gyro_calibration_max_speed_mps_ = std::max(0.0, imu_gyro_calibration_max_speed_mps_);
  imu_gyro_calibration_max_yaw_rate_rad_s_ =
      std::max(0.0, imu_gyro_calibration_max_yaw_rate_rad_s_);
  imu_gyro_bias_sum_rad_s_ = 0.0;
  imu_gyro_calibration_count_ = 0;
  imu_gyro_calibrated_ = !imu_gyro_auto_calibration_enabled_;
  last_cmd_nonzero_ = false;
  last_cmd_received_at_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
              "IMU mapping: accel rx[8:13] signed big-endian / %.3f LSB per m/s2; "
              "gyro rx[14:19] signed big-endian scale=%.9f rad/s/LSB bias=%.6f rad/s "
              "auto_calibration=%s samples=%d",
              imu_accel_scale_lsb_per_mps2_,
              imu_gyro_scale_rad_s_per_lsb_, imu_gyro_bias_z_rad_s_,
              imu_gyro_auto_calibration_enabled_ ? "true" : "false",
              imu_gyro_calibration_samples_);
  RCLCPP_INFO(this->get_logger(),
              "ODOM mapping: vx=rx[2:3], vy=rx[4:5], wz=rx[6:7] signed big-endian "
              "scale=%.6f m/s per LSB; world scales x=%.3f y=%.3f max_dt=%.3fs "
              "max_speed=%.3fm/s; "
              "voltage=rx[20:21] scale=%.6f V/LSB",
              odom_velocity_scale_mps_per_lsb_, odom_world_x_scale_, odom_world_y_scale_,
              odom_max_integration_dt_sec_, odom_max_valid_speed_mps_, voltage_scale_v_per_lsb_);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

  robotpose_publisher = create_publisher<origincar_msg::msg::Data>("robotpose", 10);

  robotvel_publisher = create_publisher<origincar_msg::msg::Data>("robotvel", 10);

  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 1, std::bind(&origincar_base::Cmd_Vel_Callback, this, _1));
  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 1, std::bind(&origincar_base::Akm_Cmd_Vel_Callback, this, _1));

  // Sign_Switch_Sub = create_subscription<std_msgs::msg::Int32>(
  //     "/sign4return", 1, std::bind(&origincar_base::Sign_Switch_Callback, this, _1));
  try  {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(20);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(),"origincar_base can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(),"origincar_base serial port opened");
  }
}

void origincar_base::Update_Imu_Gyro_Calibration()
{
  if (!imu_gyro_auto_calibration_enabled_ || imu_gyro_calibrated_) {
    return;
  }

  bool command_is_zero = false;
  {
    std::lock_guard<std::mutex> lock(cmd_state_mutex_);
    command_is_zero = !last_cmd_nonzero_;
  }
  const bool chassis_is_stationary =
      std::hypot(static_cast<double>(Robot_Vel.X), static_cast<double>(Robot_Vel.Y)) <=
          imu_gyro_calibration_max_speed_mps_ &&
      std::abs(static_cast<double>(Robot_Vel.Z)) <= imu_gyro_calibration_max_yaw_rate_rad_s_;
  if (!command_is_zero || !chassis_is_stationary) {
    return;
  }

  const double gyro_z = static_cast<double>(Mpu6050_Data.gyros_z_data) *
                        imu_gyro_scale_rad_s_per_lsb_;
  imu_gyro_bias_sum_rad_s_ += gyro_z;
  ++imu_gyro_calibration_count_;
  if (imu_gyro_calibration_count_ < imu_gyro_calibration_samples_) {
    return;
  }

  imu_gyro_bias_z_rad_s_ = imu_gyro_bias_sum_rad_s_ /
                           static_cast<double>(imu_gyro_calibration_count_);
  imu_gyro_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(),
              "IMU gyro z auto-calibration complete: bias=%.6f rad/s (%.3f deg/s) samples=%d",
              imu_gyro_bias_z_rad_s_, imu_gyro_bias_z_rad_s_ * 180.0 / PI,
              imu_gyro_calibration_count_);
}


void origincar_base::Send_Stop_Command()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!Stm32_Serial.isOpen()) {
      return;
    }
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;

    Send_Data.tx[4] = 0;
    Send_Data.tx[3] = 0;

    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;

    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
    Send_Data.tx[10]=FRAME_TAIL;

    try {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException&) {
    }
}

void origincar_base::Check_Cmd_Vel_Watchdog()
{
  double age_sec = 0.0;
  {
    std::lock_guard<std::mutex> lock(cmd_state_mutex_);
    if (!cmd_vel_watchdog_enabled_ || !last_cmd_nonzero_) {
      return;
    }
    age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_cmd_received_at_).count();
    if (age_sec <= cmd_vel_watchdog_timeout_sec_) {
      return;
    }
    last_cmd_nonzero_ = false;
  }
  Send_Stop_Command();
  RCLCPP_WARN(this->get_logger(), "cmd_vel watchdog stopped stale command after %.3fs", age_sec);
}

origincar_base::~origincar_base()
{
  RCLCPP_INFO(this->get_logger(),"Shutting down");
}
