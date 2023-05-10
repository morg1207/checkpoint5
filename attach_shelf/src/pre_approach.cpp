#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <ctime>
#include <thread>

#define VEL_ANGLE_Z 0.6
#define RAD 57.2957
using namespace std::chrono_literals;
using std::placeholders::_1;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    state = '1';
    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0);

    getting_params();

    // init messages
    msg_cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();
    laser_data = std::make_shared<std::vector<float>>();
    // pubs and subs
    pub_cmd =
        this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 1);
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&PreApproach::laserCallback, this, _1));
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&PreApproach::odomCallback, this, _1));
    // timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this));
  }
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    *laser_data = msg->ranges;
    laser_rang_max = msg->angle_max;
    laser_rang_min = msg->angle_min;
    laser_increment_angle = msg->angle_increment;

    flag_laser = true;
    // RCLCPP_INFO(this->get_logger(), "Laser scan arrived");
  }
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w); // asume que ya tiene el cuaternión

    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    current_angle = yaw;
    // RCLCPP_INFO(this->get_logger(), "odom");
  }
  void getting_params() {

    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
    RCLCPP_INFO(this->get_logger(), "obstacles %.3f ", obstacle);
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "odom");
    if (flag_laser == true) {
      int laser_index_front =
          (int)(((laser_rang_max - laser_rang_min) / laser_increment_angle) /
                2);
      RCLCPP_INFO(this->get_logger(), "%d", laser_index_front);

      switch (state) {
      case '1':
        if (laser_data->at(laser_index_front) > obstacle) {
          RCLCPP_INFO(this->get_logger(), "laser data  %.3f  obstacle  %.3f",
                      laser_data->at(laser_index_front), obstacle);
          msg_cmd_vel->angular.z = 0;
          msg_cmd_vel->linear.x = 0.2;
          pub_cmd->publish(*msg_cmd_vel);
          RCLCPP_INFO(this->get_logger(),
                      "Avanzando ->Distancia %.3f  state %c",
                      laser_data->at(laser_index_front), state);
        } else {
          state = '2';
        }
        break;
      case '2':

        msg_cmd_vel->angular.z = 0.0;
        msg_cmd_vel->linear.x = 0.0;
        pub_cmd->publish(*msg_cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Parada ->   Distancia %.3f  state %c",
                    laser_data->at(360), state);
        state = '3';
        set_point_angle = current_angle + degrees / RAD;
        break;
      case '3':
        if (degrees > 0) {
          if (abs(current_angle - set_point_angle) > 0.1) {
            msg_cmd_vel->angular.z = VEL_ANGLE_Z;
            msg_cmd_vel->linear.x = 0.0;
            pub_cmd->publish(*msg_cmd_vel);
            RCLCPP_INFO(this->get_logger(),
                        "Girando left->Distancia %.3f  state %c",
                        laser_data->at(360), state);
          }

          else {
            msg_cmd_vel->angular.z = 0;
            msg_cmd_vel->linear.x = 0.0;
            pub_cmd->publish(*msg_cmd_vel);
            timer_->cancel();
          }
          break;
        }

        else {
          if (abs(current_angle - set_point_angle) > 0.1) {
            msg_cmd_vel->angular.z = -1 * VEL_ANGLE_Z;
            msg_cmd_vel->linear.x = 0.0;
            pub_cmd->publish(*msg_cmd_vel);
            RCLCPP_INFO(this->get_logger(),
                        "Girando rigth->Distancia %.3f  state %c",
                        laser_data->at(360), state);
          } else {
            msg_cmd_vel->angular.z = 0;
            msg_cmd_vel->linear.x = 0.0;
            pub_cmd->publish(*msg_cmd_vel);
            timer_->cancel();
          }
          break;
        }
      }
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  std::shared_ptr<geometry_msgs::msg::Twist> msg_cmd_vel;
  std::shared_ptr<std::vector<float>> laser_data;
  float obstacle;
  float degrees;
  float laser_rang_min;
  float laser_rang_max;
  float laser_increment_angle;
  bool flag_laser = false;
  char state;
  float current_angle;
  float set_point_angle;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}