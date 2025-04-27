Cognitive Architecture for Post-Earthquake Search and Rescue
============================================================

Introduction  
------------
This project implements a cognitive architecture for a TIAGo robot operating in post-earthquake scenarios with dummy code Implementation showcasing the proposed architecture, designed to autonomously locate injured individuals, assess structural risks, and prioritize rescue efforts. Developed as part of the *Cognitive Architecture for Robotics* course at the Università di Genova, the system integrates sensor fusion, real-time decision-making, and human-robot collaboration to address critical challenges in disaster response.

Project Context
---------------
In disaster scenarios, rapid assessment of structural stability and efficient victim triage are vital for saving lives. This architecture enables a robot to:  
- Create 3D environmental maps for navigation  
- Detect structural damage using multi-sensor data  
- Identify and classify victims’ medical conditions  
- Transmit real-time reports to human supervisors  

Mandatory Components 
-------------------- 
The implementation focuses on three core components as defined in **Topic 1**:  

1. **Structural Risk Assessment**  
   - Analyzes LiDAR, RGB-D, and force sensor data to detect cracks and unstable structures  
   - Implements risk classification (Low/Medium/High)  
   - Triggers autonomous repositioning for detailed inspections  

2. **Victim Detection and Reporting**  
   - Combines audio processing (microphones) and computer vision (RGB-D)  
   - Publishes victim locations with SLAM-integrated coordinates  
   - Implements multi-modal confirmation for reduced false positives  

3. **Triage System**  
   - Assesses consciousness via vocal responsiveness checks  
   - Classifies victims using Red/Yellow/Green priority levels  
   - Integrates with a speaker service for human interaction  

Technical Implementation 
------------------------ 
- **ROS Framework**: Nodes communicate via ROS topics/services  
- **Sensor Integration**:  
  - RGB-D Camera: Depth imaging for victim detection  
  - LiDAR/SONAR: Environmental mapping and obstacle avoidance  
  - Force Sensors: Structural stress analysis  
- **Modular Design**: Components follow stateless service patterns for scalability  

Compliance with Requirements  
----------------------------
- **Component Diagram**: Structural diagram showcasing the system composition of Subsystems and their internal components and the interfaces between them  
- **Behavioral Diagrams**: Acitivity diagram for Structural Risk Assessment showcasing the actions taken and how they flow from one activity to another, Sequence diagram for Triage System showcasing modeling the flow of messages exchanged between objects over time.
- **Integration Testing**: Validated through ROS-based test scenarios  
- **GitHub Repository**: [CogAR_assignment1](https://github.com/MazenAtta/CogAR_assignment1.git) contains launch files, dummy implementations, and an integration test  