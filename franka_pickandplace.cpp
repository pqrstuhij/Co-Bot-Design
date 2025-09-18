#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <franka/gripper.h>
#include <tf2/LinearMath/Quaternion.h>

#define FRANKA_IP "192.168.1.135"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_pickandplace");

moveit_msgs::msg::Constraints createPathConstraints(const std::string& joint_name, double position, double tolerance, double weight) {
 moveit_msgs::msg::JointConstraint joint_constraint;
 joint_constraint.joint_name = joint_name;
 joint_constraint.position = position;
 joint_constraint.tolerance_above = tolerance;
 joint_constraint.tolerance_below = tolerance;
 joint_constraint.weight = weight;

 moveit_msgs::msg::Constraints constraints;
 constraints.joint_constraints.push_back(joint_constraint);
 return constraints;
}

geometry_msgs::msg::Pose createTargetPose(double x, double y, double z, double roll, double pitch, double yaw) {
 geometry_msgs::msg::Pose pose;
 pose.position.x = x;
 pose.position.y = y;
 pose.position.z = z;

 tf2::Quaternion q;
 q.setRPY(roll, pitch, yaw);
 pose.orientation.x = q.x();
 pose.orientation.y = q.y();
 pose.orientation.z = q.z();
 pose.orientation.w = q.w();

 return pose;
}

void moveToPose(moveit::planning_interface::MoveGroupInterface &move_group, 
 const geometry_msgs::msg::Pose &target_pose, 
 const moveit_msgs::msg::Constraints &constraints) {
 move_group.setPoseTarget(target_pose);
 move_group.setPathConstraints(constraints);

 moveit::planning_interface::MoveGroupInterface::Plan plan;
 bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

 if (success) {
 RCLCPP_INFO(LOGGER, "Executing plan...");
 move_group.move();
 } else {
 RCLCPP_WARN(LOGGER, "Planning failed.");
 }

 move_group.clearPathConstraints();
}

class MinimalSubscriber : public rclcpp::Node {
public:
 MinimalSubscriber() : Node("subNode_yoloResult"), data_received_(false) {
 subscription_ = this->create_subscription<std_msgs::msg::String>(
 "Team02_Ros2Topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
 }

 std::string getData() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return recvStr_;
 }

 void clearData() {
 std::lock_guard<std::mutex> lock(mutex_);
 recvStr_ = "";
 }

 void resetDataReceivedFlag() {
 std::lock_guard<std::mutex> lock(mutex_);
 data_received_ = false;
 }

 bool isDataReceived() const {
 return data_received_;
 }

private:
 void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
 try {
 if (!data_received_) { // 데이터를 한 번만 받도록 확인
 recvStr_ = msg->data;
 RCLCPP_INFO(this->get_logger(), "Received data: %s", recvStr_.c_str());
 data_received_ = true;
 }
 } catch (const std::exception& e) {
 std::cerr << e.what() << '\n';
 }
 }

 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
 mutable std::mutex mutex_;
 std::string recvStr_ = ""; // Default option
 bool data_received_; // 데이터 수신 여부를 확인하는 플래그
};

void moveRobot(moveit::planning_interface::MoveGroupInterface &move_group, MinimalSubscriber &subscriber, 
 moveit_visual_tools::MoveItVisualTools &visual_tools) {
 move_group.setMaxVelocityScalingFactor(2.0);
 move_group.setMaxAccelerationScalingFactor(2.0);

 auto fGripper = std::make_shared<franka::Gripper>(FRANKA_IP);
 while (rclcpp::ok()) {
 std::string recvData = subscriber.getData();

 int pType = -1;
 double pAngle = 0.0, pDiff = 0.0;

 std::istringstream iss(recvData);
 std::string token;
 
 if(recvData.length() > 1)
 {
 // 문자열 파싱
 std::istringstream iss(recvData);
 std::string token;

 // 첫 번째 값 (int)
 if (std::getline(iss, token, ';')) {
 pType = std::stoi(token); // 문자열을 int로 변환
 } else {
 throw std::runtime_error("Failed to parse int value");
 }

 // 두 번째 값 (double)
 if (std::getline(iss, token, ';')) {
 pAngle = std::stod(token); // 문자열을 double로 변환
 } else {
 throw std::runtime_error("Failed to parse first double value");
 }

 // 세 번째 값 (double)
 if (std::getline(iss, token, ';')) {
 pDiff = std::stod(token); // 문자열을 double로 변환
 } else {
 throw std::runtime_error("Failed to parse second double value");
 }

 std::cout << "pType: " << pType << "\t pAngle: " << pAngle << "\t pDiff: " << pDiff << std::endl;

 subscriber.clearData();
 }

 if (pType == 0) {

 
 std::vector<double> joint_goal;
 joint_goal.push_back(0.0); // panda_joint1 값
 joint_goal.push_back(0.5); // panda_joint2 값
 joint_goal.push_back(-0.5); // panda_joint3 값
 joint_goal.push_back(0.0); // panda_joint4 값
 joint_goal.push_back(0.0); // panda_joint5 값
 joint_goal.push_back(0.0); // panda_joint6 값
 joint_goal.push_back(0.0); // panda_joint7 값

 // 원하는 조인트 목표 값으로 설정
 move_group.setJointValueTarget(joint_goal);

 // 조인트 목표로 이동
 moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
 moveit::core::MoveItErrorCode error_code = move_group.plan(joint_plan);
 if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(joint_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Joint path planning failed with error code: %d", error_code.val);
 }


 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 std::this_thread::sleep_for(std::chrono::seconds(18));

 // 카르테시안 경로 설정을 위한 목표 포즈들
 std::vector<geometry_msgs::msg::Pose> waypoints;

 // 첫 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cr = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.push_back(target_pose_cr);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints1 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan1;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints1);
 double fraction1 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction1 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 두 번째 목표 포즈 (컨베이어 아래)
 geometry_msgs::msg::Pose target_pose_cc = createTargetPose(0.35, -0.61, 0.27, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.clear();
 waypoints.push_back(target_pose_cc);



 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints2 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan2;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints2);
 double fraction2 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan2.trajectory_);
 if (fraction2 == 1.0) {
 move_group.execute(plan2);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 오므리기
 fGripper->grasp(0.02, 0.1, 0.5);

 // 세 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cu = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_cu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints3 = createPathConstraints("panda_joint1", -M_PI / 9, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan3;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints3);
 double fraction3 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan3.trajectory_);
 if (fraction3 == 1.0) {
 move_group.execute(plan3);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 std::vector<double> default_joint_goal;
 default_joint_goal.push_back(0.0); // panda_joint1 값
 default_joint_goal.push_back(-0.785); // panda_joint2 값
 default_joint_goal.push_back(0.0); // panda_joint3 값
 default_joint_goal.push_back(-2.356); // panda_joint4 값
 default_joint_goal.push_back(0.0); // panda_joint5 값
 default_joint_goal.push_back(1.571); // panda_joint6 값
 default_joint_goal.push_back(0.785); // panda_joint7 값

 // 기본 위치로 이동하기 위한 목표 값 설정
 move_group.setJointValueTarget(default_joint_goal);

 // 기본 위치로 이동 계획
 moveit::planning_interface::MoveGroupInterface::Plan default_plan;
 moveit::core::MoveItErrorCode default_error_code = move_group.plan(default_plan);
 if (default_error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(default_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Default position planning failed with error code: %d", default_error_code.val);
 }

 // 다섯 번째 목표 포즈 (회색위)
 geometry_msgs::msg::Pose target_pose_gu = createTargetPose(0.67, 0.12, 0.50, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_gu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints5 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan5;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints5);
 double fraction5 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan5.trajectory_);
 if (fraction5 == 1.0) {
 move_group.execute(plan5);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 여섯 번째 목표 포즈 (회색)
 geometry_msgs::msg::Pose target_pose_g = createTargetPose(0.67, 0.12, 0.35, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_g);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints6 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan6;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints6);
 double fraction6 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan6.trajectory_);
 if (fraction6 == 1.0) {
 move_group.execute(plan6);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 
 subscriber.resetDataReceivedFlag();
 pType = -1;

 geometry_msgs::msg::Pose target_pose_cre = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25)); //-
 waypoints.push_back(target_pose_cre);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints7 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan7;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints7);
 double fraction7 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction7 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 }
 else if (pType == 1) {

 
 std::vector<double> joint_goal;
 joint_goal.push_back(0.0); // panda_joint1 값
 joint_goal.push_back(0.5); // panda_joint2 값
 joint_goal.push_back(-0.5); // panda_joint3 값
 joint_goal.push_back(0.0); // panda_joint4 값
 joint_goal.push_back(0.0); // panda_joint5 값
 joint_goal.push_back(0.0); // panda_joint6 값
 joint_goal.push_back(0.0); // panda_joint7 값

 // 원하는 조인트 목표 값으로 설정
 move_group.setJointValueTarget(joint_goal);

 // 조인트 목표로 이동
 moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
 moveit::core::MoveItErrorCode error_code = move_group.plan(joint_plan);
 if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(joint_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Joint path planning failed with error code: %d", error_code.val);
 }


 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 std::this_thread::sleep_for(std::chrono::seconds(18));

 // 카르테시안 경로 설정을 위한 목표 포즈들
 std::vector<geometry_msgs::msg::Pose> waypoints;

 // 첫 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cr = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.push_back(target_pose_cr);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints1 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan1;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints1);
 double fraction1 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction1 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 두 번째 목표 포즈 (컨베이어 아래)
 geometry_msgs::msg::Pose target_pose_cc = createTargetPose(0.35, -0.61, 0.27, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.clear();
 waypoints.push_back(target_pose_cc);



 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints2 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan2;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints2);
 double fraction2 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan2.trajectory_);
 if (fraction2 == 1.0) {
 move_group.execute(plan2);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 오므리기
 fGripper->grasp(0.02, 0.1, 0.5);

 // 세 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cu = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_cu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints3 = createPathConstraints("panda_joint1", -M_PI / 9, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan3;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints3);
 double fraction3 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan3.trajectory_);
 if (fraction3 == 1.0) {
 move_group.execute(plan3);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 std::vector<double> default_joint_goal;
 default_joint_goal.push_back(0.0); // panda_joint1 값
 default_joint_goal.push_back(-0.785); // panda_joint2 값
 default_joint_goal.push_back(0.0); // panda_joint3 값
 default_joint_goal.push_back(-2.356); // panda_joint4 값
 default_joint_goal.push_back(0.0); // panda_joint5 값
 default_joint_goal.push_back(1.571); // panda_joint6 값
 default_joint_goal.push_back(0.785); // panda_joint7 값

 // 기본 위치로 이동하기 위한 목표 값 설정
 move_group.setJointValueTarget(default_joint_goal);

 // 기본 위치로 이동 계획
 moveit::planning_interface::MoveGroupInterface::Plan default_plan;
 moveit::core::MoveItErrorCode default_error_code = move_group.plan(default_plan);
 if (default_error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(default_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Default position planning failed with error code: %d", default_error_code.val);
 }

 // 다섯 번째 목표 포즈 (빨간색위)
 geometry_msgs::msg::Pose target_pose_ru = createTargetPose(0.67, -0.23, 0.50, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_ru);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints5 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan5;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints5);
 double fraction5 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan5.trajectory_);
 if (fraction5 == 1.0) {
 move_group.execute(plan5);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 여섯 번째 목표 포즈 (빨간색)
 geometry_msgs::msg::Pose target_pose_r = createTargetPose(0.67, -0.23, 0.35, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_r);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints6 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan6;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints6);
 double fraction6 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan6.trajectory_);
 if (fraction6 == 1.0) {
 move_group.execute(plan6);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 
 subscriber.resetDataReceivedFlag();
 pType = -1;

 geometry_msgs::msg::Pose target_pose_cre = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25)); //-
 waypoints.push_back(target_pose_cre);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints7 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan7;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints7);
 double fraction7 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction7 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 }
 else if (pType == 2) {

 
 std::vector<double> joint_goal;
 joint_goal.push_back(0.0); // panda_joint1 값
 joint_goal.push_back(0.5); // panda_joint2 값
 joint_goal.push_back(-0.5); // panda_joint3 값
 joint_goal.push_back(0.0); // panda_joint4 값
 joint_goal.push_back(0.0); // panda_joint5 값
 joint_goal.push_back(0.0); // panda_joint6 값
 joint_goal.push_back(0.0); // panda_joint7 값

 // 원하는 조인트 목표 값으로 설정
 move_group.setJointValueTarget(joint_goal);

 // 조인트 목표로 이동
 moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
 moveit::core::MoveItErrorCode error_code = move_group.plan(joint_plan);
 if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(joint_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Joint path planning failed with error code: %d", error_code.val);
 }


 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 std::this_thread::sleep_for(std::chrono::seconds(18));

 // 카르테시안 경로 설정을 위한 목표 포즈들
 std::vector<geometry_msgs::msg::Pose> waypoints;

 // 첫 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cr = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.push_back(target_pose_cr);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints1 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan1;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints1);
 double fraction1 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction1 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 두 번째 목표 포즈 (컨베이어 아래)
 geometry_msgs::msg::Pose target_pose_cc = createTargetPose(0.35, -0.61, 0.27, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.clear();
 waypoints.push_back(target_pose_cc);



 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints2 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan2;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints2);
 double fraction2 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan2.trajectory_);
 if (fraction2 == 1.0) {
 move_group.execute(plan2);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 오므리기
 fGripper->grasp(0.02, 0.1, 0.5);

 // 세 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cu = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_cu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints3 = createPathConstraints("panda_joint1", -M_PI / 9, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan3;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints3);
 double fraction3 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan3.trajectory_);
 if (fraction3 == 1.0) {
 move_group.execute(plan3);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 std::vector<double> default_joint_goal;
 default_joint_goal.push_back(0.0); // panda_joint1 값
 default_joint_goal.push_back(-0.785); // panda_joint2 값
 default_joint_goal.push_back(0.0); // panda_joint3 값
 default_joint_goal.push_back(-2.356); // panda_joint4 값
 default_joint_goal.push_back(0.0); // panda_joint5 값
 default_joint_goal.push_back(1.571); // panda_joint6 값
 default_joint_goal.push_back(0.785); // panda_joint7 값

 // 기본 위치로 이동하기 위한 목표 값 설정
 move_group.setJointValueTarget(default_joint_goal);

 // 기본 위치로 이동 계획
 moveit::planning_interface::MoveGroupInterface::Plan default_plan;
 moveit::core::MoveItErrorCode default_error_code = move_group.plan(default_plan);
 if (default_error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(default_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Default position planning failed with error code: %d", default_error_code.val);
 }

 // 다섯 번째 목표 포즈 (파란색위)
 geometry_msgs::msg::Pose target_pose_bu = createTargetPose(0.35, 0.12, 0.50, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_bu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints5 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan5;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints5);
 double fraction5 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan5.trajectory_);
 if (fraction5 == 1.0) {
 move_group.execute(plan5);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 여섯 번째 목표 포즈 (파란색)
 geometry_msgs::msg::Pose target_pose_b = createTargetPose(0.35, 0.12, 0.35, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_b);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints6 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan6;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints6);
 double fraction6 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan6.trajectory_);
 if (fraction6 == 1.0) {
 move_group.execute(plan6);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 
 subscriber.resetDataReceivedFlag();
 pType = -1;

 geometry_msgs::msg::Pose target_pose_cre = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25)); //-
 waypoints.push_back(target_pose_cre);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints7 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan7;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints7);
 double fraction7 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction7 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 }
 else if (pType == 3) {

 
 std::vector<double> joint_goal;
 joint_goal.push_back(0.0); // panda_joint1 값
 joint_goal.push_back(0.5); // panda_joint2 값
 joint_goal.push_back(-0.5); // panda_joint3 값
 joint_goal.push_back(0.0); // panda_joint4 값
 joint_goal.push_back(0.0); // panda_joint5 값
 joint_goal.push_back(0.0); // panda_joint6 값
 joint_goal.push_back(0.0); // panda_joint7 값

 // 원하는 조인트 목표 값으로 설정
 move_group.setJointValueTarget(joint_goal);

 // 조인트 목표로 이동
 moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
 moveit::core::MoveItErrorCode error_code = move_group.plan(joint_plan);
 if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(joint_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Joint path planning failed with error code: %d", error_code.val);
 }


 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 std::this_thread::sleep_for(std::chrono::seconds(18));

 // 카르테시안 경로 설정을 위한 목표 포즈들
 std::vector<geometry_msgs::msg::Pose> waypoints;

 // 첫 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cr = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.push_back(target_pose_cr);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints1 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan1;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints1);
 double fraction1 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction1 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 두 번째 목표 포즈 (컨베이어 아래)
 geometry_msgs::msg::Pose target_pose_cc = createTargetPose(0.35, -0.61, 0.27, M_PI, 0, +(M_PI * 0.25) - (M_PI * pAngle / 180)); //-
 waypoints.clear();
 waypoints.push_back(target_pose_cc);



 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints2 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan2;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints2);
 double fraction2 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan2.trajectory_);
 if (fraction2 == 1.0) {
 move_group.execute(plan2);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 오므리기
 fGripper->grasp(0.02, 0.1, 0.5);

 // 세 번째 목표 포즈 (컨베이어 위)
 geometry_msgs::msg::Pose target_pose_cu = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_cu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints3 = createPathConstraints("panda_joint1", -M_PI / 9, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan3;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints3);
 double fraction3 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan3.trajectory_);
 if (fraction3 == 1.0) {
 move_group.execute(plan3);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 std::vector<double> default_joint_goal;
 default_joint_goal.push_back(0.0); // panda_joint1 값
 default_joint_goal.push_back(-0.785); // panda_joint2 값
 default_joint_goal.push_back(0.0); // panda_joint3 값
 default_joint_goal.push_back(-2.356); // panda_joint4 값
 default_joint_goal.push_back(0.0); // panda_joint5 값
 default_joint_goal.push_back(1.571); // panda_joint6 값
 default_joint_goal.push_back(0.785); // panda_joint7 값

 // 기본 위치로 이동하기 위한 목표 값 설정
 move_group.setJointValueTarget(default_joint_goal);

 // 기본 위치로 이동 계획
 moveit::planning_interface::MoveGroupInterface::Plan default_plan;
 moveit::core::MoveItErrorCode default_error_code = move_group.plan(default_plan);
 if (default_error_code == moveit::core::MoveItErrorCode::SUCCESS) {
 move_group.execute(default_plan);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Default position planning failed with error code: %d", default_error_code.val);
 }

 // 다섯 번째 목표 포즈 (파란색위)
 geometry_msgs::msg::Pose target_pose_yu = createTargetPose(0.35, -0.23, 0.50, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_yu);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints5 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan5;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints5);
 double fraction5 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan5.trajectory_);
 if (fraction5 == 1.0) {
 move_group.execute(plan5);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 여섯 번째 목표 포즈 (파란색)
 geometry_msgs::msg::Pose target_pose_y = createTargetPose(0.35, -0.23, 0.35, M_PI, 0, (M_PI * 0.25));
 waypoints.clear();
 waypoints.push_back(target_pose_y);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints6 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan6;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints6);
 double fraction6 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan6.trajectory_);
 if (fraction6 == 1.0) {
 move_group.execute(plan6);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 // 그리퍼 벌리기
 fGripper->grasp(0.1, 0.1, 0.5);

 
 subscriber.resetDataReceivedFlag();
 pType = -1;

 geometry_msgs::msg::Pose target_pose_cre = createTargetPose(0.35, -0.61, 0.50, M_PI, 0, -(M_PI * 0.25)); //-
 waypoints.push_back(target_pose_cre);

 // 경로 계산 및 실행
 moveit_msgs::msg::Constraints constraints7 = createPathConstraints("panda_joint1", -M_PI, M_PI / 3, 1.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan7;
 move_group.setStartStateToCurrentState();
 move_group.setPathConstraints(constraints7);
 double fraction7 = move_group.computeCartesianPath(waypoints, 0.01, 0.0, plan1.trajectory_);
 if (fraction7 == 1.0) {
 move_group.execute(plan1);
 } else {
 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cartesian path planning failed!");
 }

 }
 
 std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Loop delay
 }
}

int main(int argc, char *argv[]) {
 rclcpp::init(argc, argv);

 // Multi-threaded executor
 rclcpp::executors::MultiThreadedExecutor executor;

 // Subscriber Node
 auto subscriber_node = std::make_shared<MinimalSubscriber>();
 executor.add_node(subscriber_node);

 // MoveIt Node
 rclcpp::NodeOptions node_options;
 node_options.automatically_declare_parameters_from_overrides(true);
 auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

 static const std::string PLANNING_GROUP = "panda_arm";
 moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

 // Visual Tools
 namespace rvt = rviz_visual_tools;
 moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial", move_group.getRobotModel());
 visual_tools.deleteAllMarkers();
 visual_tools.loadRemoteControl();

 // Franka Gripper
 auto fGripper = std::make_shared<franka::Gripper>(FRANKA_IP);

 // Thread for MoveIt
 std::thread moveit_thread([&move_group, &subscriber_node, &visual_tools]() {
 moveRobot(move_group, *subscriber_node, visual_tools);
 });

 // Spin executor
 executor.spin();

 // Clean up
 moveit_thread.join();
 rclcpp::shutdown();
 return 0;
}