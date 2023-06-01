#!/bin/bash
set -e
printf "Building zed-ros2-wrapper, tmp_sources should not exist already"

mkdir tmp_sources
cp -r ../zed* ./tmp_sources
docker build -t "zed-ros2-wrapper" -f Dockerfile.l4t35_1-humble-devel .
rm -r ./tmp_sources
