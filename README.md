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
with one equipped the [Robotiq 2F-85](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) gripper naming *Manipulator Arm* while another attached an eye-in-hand camera naming *Camera Arm*.
<<<<<<< HEAD
The model and DH parameters are defined as follows <p align="center"><img src="Image/Kinova Model/Kinova Model.png" width="720"/></p>
=======
The model and DH parameters are defined as follows <p align="center"><img src="Image/Kinova Model.png" width="720"/></p>
>>>>>>> f7c954607e4542b640012545d6d237bb2aa7ca3e

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
sure the Oculus and OpenVR Loader two items not ticked in Unity Project Settings.<img src="Image/OS Version/Linux/untick.png" width="720"/>
For generating the Matlab engine python module to be used in ROS backend, you may follow the instructions step by step.

### Windows

- Windows 10/11 
- WSL (with ROS Melodic/Noetic)
- Matlab R2021b (Installed in WSL)
- Python 3.8+ (Installed in WSL)
- SteamVR (Oculus Quest 2)
- OculusClient
- Unity 20.3+

Make sure you have got WSL (together with ROS, Matlab and Python) installed under your Windows system. In order to use Oculus as the VR devices, on the contrary of what you need to do in Linux, you have to ensure
sure the Oculus and OpenVR Loader two items are ticked in Unity Project Settings.<img src="Image/OS Version/Windows/tick.png" width="720"/>
You can follow the [guidance online]("https://www.tomsguide.com/how-to/how-to-connect-oculus-quest-2-to-a-pc") to connect your Oculus Headset with PC via cable, and start the SteamVR afterwards. Once you get this status in SteamVR, you're ready for launching the program with Oculus.

<img src="Image/Oculus/SteamVR.png" width="720"/>

> Note: Please get noticed that you have to install the Matlab in WSL meaning the Matlab version is of Linux instead of Windows because 
> we have to use ROS in Python interpreter where ROS could only be compiled under Linux rather than Windows.

---

Unity's tools for robotic simulation enable users to integrate Unity with ROS-based workflows. [ROS](http://wiki.ros.org/ROS/Introduction) (Robot Operating System) provides services such as message-passing, package management, low-level device control, and hardware abstraction. Unity's robotics tools are able to support **importing URDF files** and **sending and receiving messages between ROS and Unity**. This tutorial will go through the steps necessary to integrate ROS with Unity, from installing the Unity Editor to creating a scene with an imported URDF to completing a pick-and-place task with known poses using [MoveIt](https://moveit.ros.org/) trajectory planning.

This tutorial is designed such that you do not need prior experience with Unity or C# in order to follow the scene setup steps, and you do not need prior robotics experience to get started with ROS integration. The tutorial is divided into high-level phases, from basic Unity and ROS initial setup through executing a pick-and-place task.

**Want to skip the tutorial and run the full demo? Check out our [Quick Demo](quick_demo.md)**

> Note: This project has been tested with Python 2 and ROS Melodic, as well as Python 3 and ROS Noetic.

---

We're currently working on lots of things! As a first step for this tutorial, please take a short moment fill out our [survey](https://unitysoftware.co1.qualtrics.com/jfe/form/SV_0ojVkDVW0nNrHkW) to help us identify what products and packages to build next.

---

### Pick-and-Place Tutorial
  - [Requirements](#requirements)
  - [Part 0: ROS Setup](#part-0-ros-setup)
  - [Part 1: Create Unity scene with imported URDF](#part-1-create-unity-scene-with-imported-urdf)
  - [Part 2: ROS–Unity Integration](#part-2-rosunity-integration)
  - [Part 3: Pick-and-Place In Unity](#part-3-pick-and-place-in-unity)
  - [Part 4: Pick-and-Place on the Real Robot](#part-4-pick-and-place-on-the-real-robot)

## Requirements

This repository provides project files for the pick-and-place tutorial, including Unity assets, URDF files, and ROS scripts. Clone this repository to a location on your local machine:
  ```bash
  git clone --recurse-submodules https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
  ```

## [Part 0: ROS Setup](0_ros_setup.md)

<img src="img/0_docker.png" width="400"/>

This part provides two options for setting up your ROS workspace: using Docker, or manually setting up a catkin workspace.
<<<<<<< HEAD
=======

## [Part 1: Create Unity scene with imported URDF](1_urdf.md)

<img src="img/1_end.gif" width="400"/>

This part includes downloading and installing the Unity Editor, setting up a basic Unity scene, and importing a robot--the [Niryo One](https://niryo.com/niryo-one/)--using the URDF Importer.

## [Part 2: ROS–Unity Integration](2_ros_tcp.md)

<img src="img/2_echo.png" width="400"/>

This part covers creating a TCP connection between Unity and ROS, generating C# scripts from a ROS msg and srv files, and publishing to a ROS topic.

## [Part 3: Pick-and-Place In Unity](3_pick_and_place.md)

<img src="img/0_pick_place.gif" width="400"/>

This part includes the preparation and setup necessary to run a pick-and-place task with known poses using MoveIt. Steps covered include creating and invoking a motion planning service in ROS, moving a Unity Articulation Body based on a calculated trajectory, and controlling a gripping tool to successfully grasp and drop an object.

## [Part 4: Pick-and-Place on the Real Robot](4_pick_and_place.md)

<img src="img/4_pick_and_place.gif" width="400"/>

This part is going to be a little different than the previous tutorials in that it will utilize a real Niryo One robot. We do not assume that everyone has access to a Niryo One outside of simulation. As such this tutorial should mostly be used as a reference for how to move from executing commands on a simulated robot to a real one.
>>>>>>> f7c954607e4542b640012545d6d237bb2aa7ca3e
