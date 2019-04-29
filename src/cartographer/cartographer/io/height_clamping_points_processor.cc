#include "cartographer/io/height_clamping_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io{

std::unique_ptr<HeightClampingPointsProcessor>
HeightClampingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<HeightClampingPointsProcessor>(
      dictionary->GetDouble("min_height"), dictionary->GetDouble("max_height"),
      next);
}

HeightClampingPointsProcessor::HeightClampingPointsProcessor(
    const double min_height, const double max_height, PointsProcessor* next)
    : min_height_(min_height), max_height_(max_height), next_(next) {}

void HeightClampingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  std::unordered_set<int> to_remove;
  size_t cnt = 0;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const float point_z = (batch->points[i] - batch->origin)(2);
    // printf ("min_height_ = %f, max_height_ = %f, point_z = %f\n",
    //     min_height_, max_height_, point_z);
    if (!(min_height_ <= point_z && point_z <= max_height_)) {
      to_remove.insert(i);
      ++cnt;
    }
  }
  printf ("Removed %ld points\n", cnt);

  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult HeightClampingPointsProcessor::Flush() {
  return next_->Flush();
}

} // namespace io
} // namespace cartographer
