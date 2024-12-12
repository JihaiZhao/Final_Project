# Final_Project

---
## Final Video

<video src="https://github.com/user-attachments/assets/aea727c7-ccae-4171-87d1-23032c203890" controls></video>

## Overview

The purpose of this project is to leverage **ergodic imitation** to learn both desirable and undesirable behaviors. 

To facilitate effective data collection, I first implemented an **impedance control mode** for a 7-DoF collaborative robotic arm (Franka) in collaboration with Courtney. Subsequently, I developed a **haptic-guided teleoperation** system for the Franka robot. This system enables the user to control **Franka 1**, which operates in impedance control mode, and couples its movements to **Franka 2**, such that any motion of Franka 1 is mirrored by Franka 2.

Following this, I employed a **learning-from-demonstration (LfD)** approach to derive robust task definitions from a combination of positive and negative demonstrations. The algorithmic framework for task learning is based on the **ergodic metric**, a measure of the information content in motion. 

Finally, I demonstrated the efficacy of this learning approach on real tasks including obstacle-avoidance, cleaning table, and object balancing using the 7-DoF Franka arm.

## Packages

- **impedance_control**: It includes the code for diffenent mode of impedance control me and Courtney implemented. It also includes the implementation of direct joint-position coupling teleoperation and haptic-guided teleoperation.

- **data_analyze**: It includes the code for transfering F/T sensor data to csv file, and the pipeline for analyzing the performance of different mode of impedance control.

- **density map**: Using the data collected by user using haptic-guided teleoperation to generate a density map for the a specific task.

- **ergodic**: Implement iterative linear quadratic regular (iLQR) ergodic control to generate a optimal path.

- **control_franka**: Performing the path by using moveit.