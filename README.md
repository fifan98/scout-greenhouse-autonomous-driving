# scout-greenhouse-autonomous-driving
This repository represents code implementation for Agilex Scout mini in autonomous greenhouse driving and all param files.

Arduino code is for air lift used for lifting plants.
Inside param folder is YAML files for setting up global and local planner in ROS navigation stack. Also, there is YAML files for global, local and common costmaps.
In main branch there are three files: 
1. master.py - which is master node for autonomous driving
2. PID.py - only doing PID control. Receives references via ROS topics and send cmd_vel to topic /cmd_vel
3. mjeh.py - communicates with arduino for air lift via serial port

This whole repository is pogram support for graduation thesis "An autonomous robotic system for extra-terrestrial agriculture" by Filip Bozic, FER, Croatia. 
