# Robotics Project 34753:  Robotic Arm, Group 27

This repository contains the implementation of the project assignment for the **Robotics 34753** course at DTU. The project focuses on the theoretical and practical aspects of robotics, using a 4-joint robotic arm equipped with Dynamixel AX-12A servos and a camera for computer vision tasks.

## Key Features
- **Kinematics**: Forward and inverse kinematics computations.
- **Trajectory Planning**: Polynomial interpolation for precise motion control.
- **Dynamics**: Derivation of the dynamic system equations and torque computation.
- **Computer Vision**: Detection and interaction with objects using OpenCV.

## Project Highlights
- **Problem 11b**: Detection and probing of red "smarties" using the robotic arm. This involves:
  - Camera calibration for accurate object detection.
  - HSV-based color filtering and Hough Circle Transform for identifying "smarties."
  - Motion planning to probe identified objects.

## How to Use
1. Clone the repository.
2. Install the necessary Python libraries listed in `requirements.txt`.
3. Follow the setup guide to initialize the robotic arm and camera.
4. Run the scripts to solve the problems or perform the computer vision task.

## Project Components
- `kinematics/`: Solutions to Problems 1–5 (forward/inverse kinematics, Jacobian).
- `trajectory/`: Trajectory planning and interpolation (Problems 6–7).
- `dynamics/`: Dynamics derivation and torque analysis (Problem 10).
- `vision/`: Camera calibration and smarties detection (Problem 11b).

## Resources
- [Robotis AX-12A Documentation](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
- [OpenCV Tutorials](https://en.wikibooks.org/wiki/Applied_Robotics/Sensors_and_Perception/Open_CV/Basic_OpenCV_Tutorial)

For detailed instructions, refer to the [report](./report.pdf) or individual problem folders.

