#!/bin/bash
# Run Robot Camera Application
# This script sets up the environment to avoid Qt plugin conflicts

# Force OpenCV to not use Qt backend
export QT_QPA_PLATFORM_PLUGIN_PATH=""
export OPENCV_VIDEOIO_PRIORITY_MSMF=0

# Disable OpenCV's highgui Qt backend
export OPENCV_VIDEOIO_DEBUG=1

# Run the application
python3 robot_camera_app.py
