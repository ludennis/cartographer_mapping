To install any src package please run:
	catkin_make --only-pkg-with-deps [package_name] (e.g. catkin_make --only-pkg-with-deps point_cloud_merger)
	source devel/setup.bash
	rosrun [package_name] [package_name] [arguments ...]

To install "cartographer" run:
	catkin_make_isolated --install --use-ninja
	source devel_isolated/setup.bash
	roslaunch cartographer_ros [launch_filename] [arguments ...]

