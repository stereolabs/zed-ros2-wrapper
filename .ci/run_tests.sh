#!/bin/bash
set -e

ttk='--->'

echo "${ttk} Workdir content"
ls

echo "${ttk} "ros2_ws" content"
ls ros2_ws

echo "${ttk} "ros2_ws/install" content"
ls ros2_ws/install
