// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */

void csv_torque(const franka::RobotState& robot_state, const std::array<double, 7>& commanded_torque){

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
          outputFile << ","; // Add comma delimiter for all but the last element
        } else {
          outputFile << "]}"; // Close Bracket on the last element
        }
      }
      outputFile << std::endl;
      outputFile.close();
    } else {
      std::cerr << "Error opening file for writing." << std::endl;
    }

}

void csv_torque(const franka::RobotState& robot_state, const std::array<double, 7>& commanded_torque, const std::array<double, 7>& coriolis){

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
          outputFile << ","; // Add comma delimiter for all but the last element
        } else {
          outputFile << "], "; // Close Bracket on the last element
        }
      }
      
      // Write the start of the array
      outputFile << "\"Coriolis\": [";
      for (size_t i = 0; i < coriolis.size(); ++i) {
        outputFile << coriolis[i];
        if (i != coriolis.size() - 1) {
          outputFile << ","; // Add comma delimiter for all but the last element
        } else {
          outputFile << "]}"; // Close Bracket on the last element
        }
      }

      outputFile << std::endl;
      outputFile.close();
    } else {
      std::cerr << "Error opening file for writing." << std::endl;
    }

}

void csv_torque(const franka::RobotState& robot_state, const std::array<double, 7>& commanded_torque, const std::array<double, 7>& coriolis, const Eigen::MatrixXd& jacobian){

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
          outputFile << ","; // Add comma delimiter for all but the last element
        } else {
          outputFile << "], "; // Close Bracket on the last element
        }
      }
      
      // Write the start of the array
      outputFile << "\"Coriolis\": [";
      for (size_t i = 0; i < coriolis.size(); ++i) {
        outputFile << coriolis[i];
        if (i != coriolis.size() - 1) {
          outputFile << ","; // Add comma delimiter for all but the last element
        } else {
          outputFile << "], "; // Close Bracket on the last element
        }
      }

      // Write the start of the array
      outputFile << "\"Jacobian\": [";
      for (int i = 0; i < jacobian.rows(); ++i) {
        for (int j = 0; j < jacobian.cols(); ++j) {
          outputFile << jacobian(i,j);
          outputFile << ","; // Add comma delimiter for all elements
        }
      }
      outputFile << "]}"; // Close Bracket on the last element

      outputFile << std::endl;
      outputFile.close();
    } else {
      std::cerr << "Error opening file for writing." << std::endl;
    }

}


int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{0.0}; // 150.0
  const double rotational_stiffness{50.0}; // 10.0
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  // Original Code
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);

    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7), tau_comp(7), tau_tot(7), friction_comp(7);

      // Spring damper system with damping ratio=1
      tau_task.setZero();
      tau_task.tail(3) << jacobian.rightCols(3).transpose() * (-stiffness * error - damping * (jacobian.rightCols(3) * dq.tail(3)));
      tau_d << tau_task + coriolis;

      // // print the dq of each joint to the terminal
      // std::cout << "dq Array = ";
      // for (const auto& value : dq) {
      //     std::cout << value << " ";
      // }
      // std::cout << std::endl;

      tau_comp.setZero(); 
      // account for friction as a proportion of joint velocity
      if (dq[0] < -0.001) {
        tau_comp[0] = -0.1;
      } else if (dq[0] > 0.001) {
        tau_comp[0] = 0.1;
      }
      if (dq[1] < -0.1) {
        tau_comp[1] = -0.7;
      } else if (dq[1] > 0.1) {
        tau_comp[1] = 0.7;
      }
      if (dq[2] < -0.01) {
        tau_comp[2] = -0.18;
      } else if (dq[2] > 0.01) {
        tau_comp[2] = 0.18;
      }
      if (dq[3] < -0.01) {
        tau_comp[3] = -0.4;
      } else if (dq[3] > 0.01) {
        tau_comp[3] = 0.4;
      }
      
      // friction_comp << 0.06,0.6,0.12,0.2,0.0,0.0,0.0;
      // tau_comp << friction_comp.array() * dq.array();
      // tau_tot << tau_comp + tau_d;

      tau_tot << tau_comp + tau_d;

      std::array<double, 7> tau_d_array{};
      // Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_tot;


      // save data to csv
      csv_torque(robot_state, tau_d_array, coriolis_array, jacobian);

      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
