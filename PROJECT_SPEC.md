# Multimodal Hands-Free Robotic Assistant

## Overview

This project is a ROS 2 Jazzy based multimodal robotic assistant designed for tabletop object acquisition.

The goal is to allow a user to select an object through voice commands or gaze tracking and have a robotic arm autonomously move toward and grasp the selected object.

This is a term project and not an industrial system.

The focus is on:

- ROS 2 architecture
- Modularity
- Human-robot interaction
- Perception pipeline integration
- Motion planning integration
- Clean software engineering practices

---

# Core Objectives

The system should:

1. Scan a tabletop workspace.
2. Detect ArUco markers.
3. Create a workspace coordinate frame.
4. Detect objects using a YOLO-based model.
5. Store object locations.
6. Accept user input through voice and gaze.
7. Determine the intended object.
8. Plan a robotic motion using MoveIt 2.
9. Command an Arduino-controlled robotic arm.
10. Reach and grasp the selected object.

---

# Scope

The project is intentionally limited.

Included:

- Tabletop environment
- Static robotic arm
- Static workspace
- Object detection
- Gaze selection
- Voice selection
- MoveIt integration
- Arduino communication

Not Included:

- Mobile robots
- SLAM
- Navigation
- Autonomous exploration
- Object delivery
- Human handover
- Continuous object tracking
- Multi-room operation

---

# Environment

Operating System:

Ubuntu 24.04

ROS Version:

ROS 2 Jazzy

Language:

Python

ROS Client Library:

rclpy

Build System:

ament_python

Version Control:

Git + GitHub

---

# Hardware

Primary Controller:

Arduino Uno

Future Upgrade:

ESP32

Workspace Camera:

Phone camera via IP stream

User Camera:

Laptop webcam

Manipulator:

4 DOF Servo-Based Robotic Arm

Gripper:

Simple open/close gripper

---

# Software Stack

Perception:

- OpenCV
- OpenCV ArUco
- YOLOv8n

Interaction:

- MediaPipe FaceMesh
- Vosk

Motion:

- MoveIt 2

Communication:

- ROS 2 Topics
- ROS 2 Services
- ROS 2 Actions

---

# System Architecture

User
│
├── Voice Input
│
└── Gaze Input
│
▼
Selection Manager
│
▼
Selected Object
│
▼
Motion Planner
│
▼
MoveIt 2
│
▼
Arduino
│
▼
Robot Arm

---

# Selection Rules

Voice and gaze are both supported.

If both agree:

Execute selection.

If both disagree:

Voice input has priority.

---

# Workspace Scanning Pipeline

Phone Camera
│
▼
ArUco Detection
│
▼
Workspace Frame
│
▼
YOLO Detection
│
▼
Object Coordinate Mapping
│
▼
object_map.json

The generated object map should be reusable by other nodes.

---

# Object Database

Use a JSON file.

Example:

{
    "cup":
    {
        "x": 12,
        "y": 8
    },
    "eraser":
    {
        "x": 5,
        "y": 14
    }
}

Do not use SQL databases.

JSON is sufficient.

---

# ROS Package Layout

assistant_msgs

Contains:
- Custom messages
- Custom services
- Custom actions

assistant_perception

Contains:
- Camera interfaces
- ArUco processing
- YOLO processing
- Workspace mapping

assistant_interaction

Contains:
- Voice input
- Gaze input
- Selection management

assistant_motion

Contains:
- MoveIt interface
- Pose lookup
- Motion execution
- Arduino communication

assistant_bringup

Contains:
- Launch files
- Config files
- Runtime parameters

---

# Communication Design

Topics:

/voice/object_id

/gaze/object_id

/final_selection

/object_detections

/workspace_status

Services:

/scan_workspace

Actions:

/execute_grasp

---

# Development Philosophy

Priority Order:

1. Clean architecture
2. Modularity
3. Build stability
4. ROS correctness
5. Future extensibility

Avoid:

- Monolithic nodes
- Hardcoded paths
- Circular dependencies
- Unnecessary complexity

---

# Future Extensions

Potential future work:

- Better gaze tracking
- Continuous workspace updates
- ESP32 migration
- Improved grasp planning
- Pick-and-place workflows
- Dynamic object tracking

These features should not be implemented now.

The initial architecture should leave extension points for them.

---

# Success Criteria

The project is considered successful if:

1. Workspace scan completes.
2. Object map is generated.
3. Voice or gaze selects an object.
4. Motion planning executes.
5. Arm reaches object.
6. Gripper closes.

Object lifting is desirable but not mandatory.

Reliable target acquisition is the primary objective.
