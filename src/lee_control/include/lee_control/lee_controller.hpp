/**
 * @brief Lee controller
 * @file lee_controller.cpp
 * @author Salvatore Marcellini <salvatore.marcellini@unina.it>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <Eigen/Eigen>


#include <chrono>
#include <iostream>

using namespace::std;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class LeeController : public rclcpp::Node
{
public:
	LeeController();
	void arm();
	void disarm();

private:
	
	void publishOffboardControlMode();
	void publishWrenchSetpoint();
	void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void control();

    // Subscribers functions
    void odomFeedback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

    rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_publisher_;
    rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;


    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent


    Eigen::Vector3f e3_{0.0, 0.0, 1};
    Eigen::Vector3f pos_; // drone linear position
    Eigen::Vector3f vel_; // drone linear velocity
    Eigen::Matrix3f R_att_; // drone attitude rotation matrix
    Eigen::Vector3f w_att_; // drone angular rate

    // Controller variables
    Eigen::Vector3f acc_;
    float thrust_;
    Eigen::Vector3f b1_des_;
    Eigen::Vector3f b3_des_;
    Eigen::Vector3f b2_temp_, b2_des_;
    Eigen::Matrix3f R_ctrl_;
    Eigen::Matrix3f R_err_;
    Eigen::Vector3f dot_rate_ctrl_;
    px4_msgs::msg::VehicleTorqueSetpoint torque_pub_sp_;
    px4_msgs::msg::VehicleThrustSetpoint thrust_pub_sp_;

    // Control Gains
    Eigen::Matrix3f Kp_pos_;
    Eigen::Matrix3f Kd_pos_;
    Eigen::Matrix3f Kr_att_;
    Eigen::Matrix3f Kw_att_;

    // Drone parameters
    float mass_;
    float Ixx, Iyy, Izz;

};