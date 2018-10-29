#include "cartographer/io/height_filtering_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io{

static std::unique_ptr<HeightFilteringPointsProcessor>
HeightFilteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<HeightFilteringPointsProcessor>(
      dictionary->GetDouble("height"), next);
}

HeightFilteringPointsProcessor::HeightFilteringPointsProcessor(
    const double height, PointsProcessor* next) : height_(height), next_(next) {}

void HeightFilteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  std::unordered_set<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    if (batch->points[i].z < height_) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult HeightFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

} // namespace io
} // namespace cartographer
