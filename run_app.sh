#!/bin/bash
# Run Robot Camera Application
# This script sets up the environment to avoid Qt plugin conflicts

# Unset OpenCV's Qt plugin path
unset QT_QPA_PLATFORM_PLUGIN_PATH

# Run the application
python3 robot_camera_app.py
