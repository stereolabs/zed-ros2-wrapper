
## Submitting your code changes

Code contributions should be made via pull requests to the appropriate repositories:
* [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/pulls)
* [zed-ros2-interfaces](https://github.com/stereolabs/zed-ros2-interfaces/pulls)
* [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples/pulls)

We ask all contributors to follow the practices explained in [ROS 2 documentation](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

Before submitting a pull request please perform this list of tasks from the root of your ROS 2 workspace:
1. Automatic code formatting: 
   
   `$ ament_uncrustify --reformat src`

2. Build the packages to check for compile errors: 
   
   `$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release`

3. Perform the automatic build tests: 
   
   `$ colcon test`

4. Analyze and solve eventually reported errors: 
   
   `$ colcon test-result --verbose`

5. Repeat steps (1) -> (4) until all reported formatting errors have been resolved.


## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).