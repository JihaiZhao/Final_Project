// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <franka/rate_limiting.h>

#include <memory>
#include <mutex>

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */

void csv_torque(const franka::RobotState& robot_state,
                const std::array<double, 7>& commanded_torque) {
  std::ofstream outputFile("robot_data.csv", std::ios_base::app);
  if (outputFile.is_open()) {
    // Write the Robot State to the file
    outputFile << robot_state;

    // Write the start of the array
    outputFile << "{\"Commanded_Torque\": [";
    // Write the values of tau_d_array to the file
    for (size_t i = 0; i < commanded_torque.size(); ++i) {
      outputFile << commanded_torque[i];
      if (i != commanded_torque.size() - 1) {
        outputFile << ",";  // Add comma delimiter for all but the last element
      } else {
        outputFile << "]}";  // Close Bracket on the last element
      }
    }
    outputFile << std::endl;
    outputFile.close();
  } else {
    std::cerr << "Error opening file for writing." << std::endl;
  }
}

void csv_torque(const franka::RobotState& robot_state,
                const std::array<double, 7>& commanded_torque,
                const std::array<double, 7>& coriolis) {
  std::ofstream outputFile("robot_data.csv", std::ios_base::app);
  if (outputFile.is_open()) {
    // Write the Robot State to the file
    outputFile << robot_state;

    // Write the start of the array
    outputFile << "{\"Commanded_Torque\": [";
    // Write the values of tau_d_array to the file
    for (size_t i = 0; i < commanded_torque.size(); ++i) {
      outputFile << commanded_torque[i];
      if (i != commanded_torque.size() - 1) {
        outputFile << ",";  // Add comma delimiter for all but the last element
      } else {
        outputFile << "], ";  // Close Bracket on the last element
      }
    }

    // Write the start of the array
    outputFile << "\"Coriolis\": [";
    for (size_t i = 0; i < coriolis.size(); ++i) {
      outputFile << coriolis[i];
      if (i != coriolis.size() - 1) {
        outputFile << ",";  // Add comma delimiter for all but the last element
      } else {
        outputFile << "]}";  // Close Bracket on the last element
      }
    }

    outputFile << std::endl;
    outputFile.close();
  } else {
    std::cerr << "Error opening file for writing." << std::endl;
  }
}

void csv_torque(const franka::RobotState& robot_state,
                const std::array<double, 7>& commanded_torque,
                const std::array<double, 7>& coriolis,
                const Eigen::MatrixXd& jacobian) {
  std::ofstream outputFile("robot_data.csv", std::ios_base::app);
  if (outputFile.is_open()) {
    // Write the Robot State to the file
    outputFile << robot_state;

    // Write the start of the array
    outputFile << "{\"Commanded_Torque\": [";
    // Write the values of tau_d_array to the file
    for (size_t i = 0; i < commanded_torque.size(); ++i) {
      outputFile << commanded_torque[i];
      if (i != commanded_torque.size() - 1) {
        outputFile << ",";  // Add comma delimiter for all but the last element
      } else {
        outputFile << "], ";  // Close Bracket on the last element
      }
    }

    // Write the start of the array
    outputFile << "\"Coriolis\": [";
    for (size_t i = 0; i < coriolis.size(); ++i) {
      outputFile << coriolis[i];
      if (i != coriolis.size() - 1) {
        outputFile << ",";  // Add comma delimiter for all but the last element
      } else {
        outputFile << "], ";  // Close Bracket on the last element
      }
    }

    // Write the start of the array
    outputFile << "\"Jacobian\": [";
    for (int i = 0; i < jacobian.rows(); ++i) {
      for (int j = 0; j < jacobian.cols(); ++j) {
        outputFile << jacobian(i, j);
        outputFile << ",";  // Add comma delimiter for all elements
      }
    }
    outputFile << "]}";  // Close Bracket on the last element

    outputFile << std::endl;
    outputFile.close();
  } else {
    std::cerr << "Error opening file for writing." << std::endl;
  }
}

class teleop_sub : public rclcpp::Node {
 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr slave_pub_;

  mutable std::mutex joint_mutex_;
  sensor_msgs::msg::JointState joint_;
  bool joint_updated_ = false;  // Flag to mark if new data is received

  std::vector<double> q_;
  std::vector<double> torques_;

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_.position = msg->position;
    joint_updated_ = true;  // Mark data as updated
  }

 public:
  teleop_sub() : Node("teleop_sub") {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_pub", 10, std::bind(&teleop_sub::joint_callback, this, std::placeholders::_1));
    slave_pub_ = create_publisher<sensor_msgs::msg::JointState>("/slave_pub", 10);
  }

  std::vector<double> get_joint_position() const {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    return joint_.position;
  }

  bool is_joint_updated() const {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    return joint_updated_;
  }

  void reset_joint_updated() {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_updated_ = false;
  }

  void publish_slave_state(const std::vector<double>& q, const std::vector<double>& torques) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    joint_state.position = q;
    joint_state.effort = torques;
    slave_pub_->publish(joint_state);
  }

  void set_joint_position(const std::vector<double>& joint_position) { q_ = joint_position; }

  void set_torques(const std::vector<double>& torques) { torques_ = torques; }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop_sub>();

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_initial = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_initial);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Stiffness
    const std::array<double, 7> k_gains = {{240, 240, 240, 240, 100, 60, 20}};
    // Damping
    const std::array<double, 7> d_gains = {{20, 20, 20, 20, 12, 10, 6}};

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState start_state = robot.readOnce();
    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(start_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    std::array<double, 7> q_joint_positions;
    std::copy(start_state.q.data(), start_state.q.data() + 7, q_joint_positions.begin());

    std::vector<double> q_joint_slave;
    std::vector<double> q_torque_slave;

    auto joint_position_callback = [&](const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::JointPositions {
      rclcpp::spin_some(node);

      if (node->is_joint_updated()) {
        auto joint_positions = node->get_joint_position();

        // print the dq to the terminal
        std::cout << "joint Array = ";
        for (const auto& value : joint_positions) {
          std::cout << value << " ";
        }
        std::cout << std::endl;

        if (joint_positions.size() == 7) {
          std::array<double, 7> joint_positions_array;
          std::copy(joint_positions.begin(), joint_positions.end(), joint_positions_array.begin());

        //   std::vector<double> j(joint_positions_array.begin(), joint_positions_array.end());
        //   q_joint_slave = j;
          node->reset_joint_updated();
          return joint_positions_array;
        }
        std::cerr << "Invalid joint positions size: " << joint_positions.size() << std::endl;
      }

      return q_joint_positions;
    };

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [&](const franka::RobotState& state, franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis = model.coriolis(state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);

      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
      
      std::vector<double> tau(tau_d_rate_limited.begin(), tau_d_rate_limited.end());
      q_torque_slave = tau;

      q_joint_slave = std::vector<double>(state.q.begin(), state.q.end());

      node->publish_slave_state(q_joint_slave, q_torque_slave);

      return tau_d_rate_limited;
    };


    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback, joint_position_callback);
    // robot.control(joint_position_callback);

    // node->publish_slave_state(q_joint_slave, q_torque_slave);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
