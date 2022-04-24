# IBVS (Image-based Visual Servoing Control)

![ROS](https://img.shields.io/badge/ROS-melodic-yellowgreen)
![ROS](https://img.shields.io/badge/ROS-noetic-brightgreen)
![ROS](https://img.shields.io/badge/Windows-10-blue)
![Unity](https://img.shields.io/badge/Unity-2020.2+-lightgrey)

---

## Introduction

This project is about visual servoing based on the main reference paper [Occlusion-Free Visual Servoing for the Shared Autonomy Teleoperation of Dual-Arm Robots](https://ieeexplore.ieee.org/document/8253809).
Visual servoing could be used in many teleoperation scenarios for remote control where the subject needs to manipulate one robot arm
while another moves correspondently in an autonomous way to provide FOV (Field of View) for observing the obstacle/object.

## Kinova Models

We carefully design the scenario in Unity using two [Kinova-Gen3-6-Dof](https://www.kinovarobotics.com/product/gen3-robots) robot arms
with one equipped the [Robotiq 2F-85](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) gripper naming *Manipulator Arm* while another attached an eye-in-hand camera naming *Camera Arm*. The model and DH parameters are defined as follows <p align="center"><img src="Image/Kinova Model/Kinova Model.png" width="720"/></p>

## Architecture

Basically, our designed IBVS Control System contains three parts which are the **Unity side** (Remote scene telepresence), **ROS Backend** (Receive arm/object msg and send back the control command) and **Matlab** (Optimizer) 
The overall architecture/pipeline looks like.

## Supporting OS
In order to simulate remote control scene in reality, we have introduced the VR HMD (Head Mounted Device) [Oculus Quest 2](https://www.oculus.com/quest-2/) where the program runs on Windows. So there're actually two OS versions we support.
One is ubuntu with ROS installed and another is tested under Windows 10 with WSL (Windows Subsystem for Linux). Both version all need MATLAB installed.

---

### Linux

- Ubuntu 18.04/20.04 (ROS Melodic/Noetic)
- Matlab R2021b
- Python 3.8+
- Unity 20.3+

The Linux version is used for implementing the program without using any VR devices, because of which you have to make
sure the Oculus and OpenVR Loader two items not ticked in Unity Project Settings.

<img src="Image/OS Version/Linux/untick.png"/>

For generating the Matlab engine python module to be used in ROS backend, you may follow the instructions step by step.

---

### Windows

- Windows 10/11 
- WSL (with ROS Melodic/Noetic)
- Matlab R2021b (Installed in WSL)
- Python 3.8+ (Installed in WSL)
- SteamVR (Oculus Quest 2)
- OculusClient
- Unity 20.3+

Make sure you have got WSL (together with ROS, Matlab and Python) installed under your Windows system. In order to use Oculus as the VR devices, on the contrary of what you need to do in Linux, you have to ensure
sure the Oculus and OpenVR Loader two items are ticked in Unity Project Settings.

<img src="Image/OS Version/Windows/tick.png"/>

You can follow the [guidance online]("https://www.tomsguide.com/how-to/how-to-connect-oculus-quest-2-to-a-pc") to connect your Oculus Headset with PC via cable, and start the SteamVR afterwards. Once you get this status in SteamVR, you're ready for launching the program with Oculus.

<img src="Image/Oculus/SteamVR.png" width="360"/>

> Note: Please get noticed that you have to install the Matlab in WSL meaning the Matlab version is of Linux instead of Windows because 
> we have to use ROS in Python interpreter where ROS could only be compiled under Linux rather than Windows.

---

