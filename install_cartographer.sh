rm -rf build_isolated/ devel_isolated/ install_isolated/ src/ protobuf/

sudo apt-get update

if [ "$(rosversion -d)" == "noetic" ]; then
  sudo apt-get install -y python3-wstool python3-rosdep ninja-build
else
  sudo apt-get install -y python-wstool python-rosdep ninja-build
fi

wstool init src

wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

if [ "$(rosversion -d)" == "noetic" ]; then
  sudo apt-get install stow
  src/cartographer/scripts/install_abseil.sh
fi

src/cartographer/scripts/install_proto3.sh

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

catkin_make_isolated --install --use-ninja

rm -rf src/cartographer/.git* src/cartographer_ros/.git* src/ceres-solver/.git* protobuf/.git*
