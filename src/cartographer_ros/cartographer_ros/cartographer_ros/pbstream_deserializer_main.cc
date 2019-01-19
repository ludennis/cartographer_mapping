#include <fstream>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to extract pose_graph from.");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename) {

  ::cartographer::mapping::proto::PoseGraph pose_graph =
    cartographer::io::DeserializePoseGraphFromFile(pbstream_filename);

  std::ofstream out_file(pbstream_filename + ".csv");
  out_file << "node_id,node_timestamp,node_x,node_y,node_z" << std::endl;

  for (::cartographer::mapping::proto::Trajectory trajectory : pose_graph.trajectory()) {
    for (const ::cartographer::mapping::proto::Trajectory::Node& node : trajectory.node()) {
      ::cartographer::transform::Rigid3d node_pose =
          ::cartographer::transform::ToRigid3(node.pose());
      int node_index = node.node_index();
      long int node_timestamp = node.timestamp();

      std::cout << "Node[" << node_index << ", " << node_timestamp << "]: ("
                << node_pose.translation()[0] << ", " << node_pose.translation()[1] << ", " << node_pose.translation()[2]
                << ")" << std::endl;
      out_file << node_index << "," << node_timestamp << "," << node_pose.translation()[0]
               << "," << node_pose.translation()[1] << "," << node_pose.translation()[2] << std::endl;
    }
  }
}

} // namespace
} // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename);
}
