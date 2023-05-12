#include "lee_control/lee_controller.hpp"

inline Eigen::Matrix3f QuatToMat(Eigen::Vector4f Quat){
    Eigen::Matrix3f Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
}

inline float sign(float val){
    if(val < 0)
        return -1.0;
    else if(val > 0)
        return 1.0;
    else
        return 0.0;
}

LeeController::LeeController() : Node("lee_controller")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    torque_publisher_ = this->create_publisher<VehicleTorqueSetpoint>("fmu/in/vehicle_torque_setpoint", 10);
    thrust_publisher_ = this->create_publisher<VehicleThrustSetpoint>("fmu/in/vehicle_thrust_setpoint", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
                    std::bind(&LeeController::odomFeedback, this, _1)); 

    offboard_setpoint_counter_ = 0;

    thrust_pub_sp_.xyz[0] = 0.0f;
    thrust_pub_sp_.xyz[1] = 0.0f;
    thrust_pub_sp_.xyz[2] = 0.0f;

    torque_pub_sp_.xyz[0] = 0.0f;
    torque_pub_sp_.xyz[1] = 0.0f;
    torque_pub_sp_.xyz[2] = 0.0f;

    auto timer_callback = [this]() -> void {

        if (offboard_setpoint_counter_ == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            // Arm the vehicle
            this->arm();
        }

        // // offboard_control_mode needs to be paired with trajectory_setpoint
        // publishOffboardControlMode();
        // // publish_trajectory_setpoint();

        // stop the counter after reaching 11
        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
            thrust_pub_sp_.xyz[0] = 0.0f;
            thrust_pub_sp_.xyz[1] = 0.0f;
            thrust_pub_sp_.xyz[2] = 0.0f;

            torque_pub_sp_.xyz[0] = 0.0f;
            torque_pub_sp_.xyz[1] = 0.0f;
            torque_pub_sp_.xyz[2] = 0.0f;
        }else{

            control();

        }

        publishWrenchSetpoint();

    };
    timer_ = this->create_wall_timer(2ms, timer_callback);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void LeeController::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void LeeController::arm()
{
	publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void LeeController::disarm()
{
	publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        To send the wrench vector we will need
 *        the actuator field set to true
 */
void LeeController::publishOffboardControlMode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
    msg.actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the evaluated wrench vector
 */
void LeeController::publishWrenchSetpoint()
{
	torque_pub_sp_.timestamp = thrust_pub_sp_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publishOffboardControlMode();
	torque_publisher_->publish(torque_pub_sp_);
    thrust_publisher_->publish(thrust_pub_sp_);
}

/**
 * @brief Read the odometry values
*/
void LeeController::odomFeedback(px4_msgs::msg::VehicleOdometry::UniquePtr msg){
    pos_ << msg->position[0], msg->position[1], msg->position[2];
    vel_ << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    
    R_att_ =  QuatToMat( Eigen::Vector4f( msg->q[0], msg->q[1], msg->q[2], msg->q[3] ));
    
    w_att_ << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
}

void LeeController::control(){
    
    Eigen::Vector3f pos_des{Eigen::Vector3f::Zero()};
    Eigen::Vector3f vel_des{Eigen::Vector3f::Zero()};
    Eigen::Vector3f acc_des{Eigen::Vector3f::Zero()};
    float yaw_des, yaw_rate_des;

    //To do: read desired trajectory from topic
    yaw_des = 1.57;
    yaw_rate_des = 0.0f;
    pos_des[2] = 2.0;

    //To do: add them as parameters
    mass_ = 1.5;
    Kp_pos_.diagonal() << 50.0, 50.0, 75.0;
    Kd_pos_.diagonal() << 10.0, 10.0, 20.0;
    Kr_att_.diagonal() << 0.5, 0.5, 1.0;
    Kw_att_.diagonal() << 0.1, 0.1, 0.6;
    float kf = 5.84e-06;
    float km = kf * 0.06;
    float l_arm = 0.735;
    float n_max = pow(1100.0,2);
    Eigen::Vector4f angle(0.5337, -2.565, 2.565, -0.5337);
    Eigen::Matrix3f I_mat;
    I_mat.diagonal() << 0.0347563, 0.0458929, 0.0977;

    float thrust_max = 4.0 * kf * n_max;
    // For roll and pitch we just need to sum on the same side
    float torque_max_x = fabs(  kf * l_arm * (sin(angle[0]) + sin(angle[2]) ) * n_max );
    float torque_max_y = fabs( -kf * l_arm * (cos(angle[1]) + cos(angle[2]) ) * n_max );
    float torque_max_z = 2.0 * km * n_max;
    
    Eigen::Vector3f err_pos = pos_des - pos_;
    Eigen::Vector3f err_vel = vel_des - vel_; 
    
    err_pos[0] = fabs(err_pos[0]) < 0.1 ? 0.0 : err_pos[0];
    err_pos[1] = fabs(err_pos[1]) < 0.1 ? 0.0 : err_pos[1];
    err_pos[2] = fabs(err_pos[2]) < 0.1 ? 0.0 : err_pos[2];

    err_vel[0] = fabs(err_vel[0]) < 0.1 ? 0.0 : err_vel[0];
    err_vel[1] = fabs(err_vel[1]) < 0.1 ? 0.0 : err_vel[1];
    err_vel[2] = fabs(err_vel[2]) < 0.1 ? 0.0 : err_vel[2];
    
    acc_ = Kp_pos_ * err_pos + Kd_pos_ * err_vel - mass_*9.81*Eigen::Vector3f::UnitZ() + mass_*acc_des;;
    thrust_ = -acc_.dot( R_att_.col(2) );

    b1_des_ << cos(yaw_des), sin(yaw_des), 0;
    b3_des_ = -acc_ / acc_.norm();
    b2_temp_ = b3_des_.cross(b1_des_);
    b2_des_ = b2_temp_ / b2_temp_.norm();

    R_ctrl_.col(0) = b2_des_.cross(b3_des_);
    R_ctrl_.col(1) = b2_des_;
    R_ctrl_.col(2) = b3_des_;

    R_err_ = 0.5 * (R_ctrl_.transpose() * R_att_ - R_att_.transpose() * R_ctrl_);
    Eigen::Vector3f angle_error;
    angle_error << R_err_(2, 1), R_err_(0,2), R_err_(1, 0);

    Eigen::Vector3f angular_rate_des(0.0, 0.0, yaw_rate_des);
        
    Eigen::Vector3f angular_rate_error = - R_att_.transpose() * R_ctrl_ * angular_rate_des + w_att_;

    dot_rate_ctrl_ = - Kr_att_*angle_error - Kw_att_*angular_rate_error; // + w_att_.cross(I_mat*w_att_) -I_mat*((w_att_skew*R_att_.transpose()*R_ctrl_*angular_rate_des));

    
    thrust_pub_sp_.xyz[0] = 0.0f;
    thrust_pub_sp_.xyz[1] = 0.0f;
    thrust_pub_sp_.xyz[2] = ( fabs(thrust_) > thrust_max ) ? -1.0 : -fabs(thrust_)/thrust_max;

    torque_pub_sp_.xyz[0] = ( fabs(dot_rate_ctrl_[0]) > torque_max_x ) ? sign(dot_rate_ctrl_[0]) * 1.0 : dot_rate_ctrl_[0] / torque_max_x ;
    torque_pub_sp_.xyz[1] = ( fabs(dot_rate_ctrl_[1]) > torque_max_y ) ? sign(dot_rate_ctrl_[1]) * 1.0 : dot_rate_ctrl_[1] / torque_max_y ;
    torque_pub_sp_.xyz[2] = ( fabs(dot_rate_ctrl_[2]) > torque_max_z ) ? sign(dot_rate_ctrl_[2]) * 1.0 : dot_rate_ctrl_[2] / torque_max_z ;

    // publishWrenchSetpoint();

    cout << thrust_pub_sp_.xyz[2] << endl;
    cout << torque_pub_sp_.xyz[0] << " " << torque_pub_sp_.xyz[1] << " " << torque_pub_sp_.xyz[2] << endl;
    cout << " ------------------- \n";
}

int main(int argc, char *argv[])
{
	std::cout << "Starting lee controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LeeController>());

	rclcpp::shutdown();
	return 0;
}
