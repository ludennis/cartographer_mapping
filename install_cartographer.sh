rm -rf build_isolated/ devel_isolated/ install_isolated/ src/ protobuf/

sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build

wstool init src

wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

src/cartographer/scripts/install_proto3.sh

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

catkin_make_isolated --install --use-ninja

rm -rf src/cartographer/.git* src/cartographer_ros/.git* src/ceres-solver/.git* protobuf/.git*
