![](../../images/Picto+STEREOLABS_Black.jpg)
# ZED Multi Camera

This is an example that illustrates how to configure and run a multi camera rig composed by a ZED-M camera lying on a ZED.

## Configuration
The folder `config` contains the configuration YAML files modified in order to run two wrapper nodes without generating conflicts.

This is a list of parameters that must be modified such that `zed.yaml` and `zedm.yaml` can be used for different cameras:
* `zed_id`: each camera must have an univocal ID
* `camera_frame`: the two cameras must have a different TF tree
* `left_camera_frame`: the two cameras must have a different TF tree
* `left_camera_optical_frame`: the two cameras must have a different TF tree
* `right_camera_frame`: the two cameras must have a different TF tree
* `right_camera_optical_frame`: the two cameras must have a different TF tree

*Note*: the file `common.yaml` has been removed and integrated in both `zed.yaml` and `zedm.yaml` because the namespace for the parameter settings must be univocal for each node:

ZED:
```
zed:
    zed_node:
        ros__parameters:
[...]
```

ZED-M:
```
zed:
    zedm_node:
        ros__parameters:
[...]
```


*Note*: changing the names of the TF frames for a camera requires that also the relative URDF if modified to match the new configuration, otherwise the Robot State Publisher node used to publish the static TF frames will generate conflicts.

## Launch
The folder `launch` contains the Python script that configure two ROS2 wrapper nodes and launch them.
The launch file is based on the Lifecycle launch file described in the [online documentation](https://www.stereolabs.com/docs/ros2/lifecycle/#automatic-control)

Command:
`$ ros2 launch stereolabs_example_multi_camera zed_multi_camera.launch.py`

`
