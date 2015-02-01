
# Stereo vision assignment

Note that the data in in the `data/` directory.

## To build

- Make sure you have all dependencies (ROS Indigo, OpenCV, PCL, Boost). Note: we recommend Ubuntu 14.04 with ROS Indigo.
- Create a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Clone the package into the `src` directory, optionally using `wstool` (http://wiki.ros.org/wstool)
- Run `catkin_make` and wait

In practice the last three steps (without `wstool`) look something like:

```bash
# create workspace and src dir
source /opt/ros/indigo/setup.bash
mkdir -p stereo_ws/src
cd stereo_ws/src
git clone git@bitbucket.org:castacks/stereo_assignment.git
cd ..
catkin_make
# wait while packages are compiled...
```

If you think there's something wrong with the package (e.g. missing dependency)
let me know at `dimatura@cmu.edu`.

## To run

Make sure your environment is up to date
```sh
source devel/setup.bash
```

Go to the `data` directory in the package and run
```sh
rosrun stereo_assignment stereo_assignment_main data.json left_calib.json right_calib.json
```

If everything is OK, the program writes `out.pcd` with a simple point cloud. To view run
```sh
pcl_viewer out.pcd
```

Then press `r` to center and `5` to view the RGB channel.

## If you get some sort of "json parsing" error

This is a bug in older versions of boost ([bug
report](https://svn.boost.org/trac/boost/ticket/4387)). You should upgrade.
Our lab uses Ubuntu 14.04 with ROS Indigo.

