#include "cartographer/io/motion_filter_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/io/points_batch.h"

namespace cartographer{
namespace io{

std::unique_ptr<MotionFilterPointsProcessor>
MotionFilterPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* dictionary, PointsProcessor* next) {
  return common::make_unique<MotionFilterPointsProcessor> (
      dictionary->GetDouble("filter_speed_kmph"),
      dictionary->GetDouble("filter_distance"),
      next);
}

MotionFilterPointsProcessor::MotionFilterPointsProcessor(
    const double filter_speed_kmph, const double filter_distance,
    PointsProcessor* next)
    : filter_speed_kmph_(filter_speed_kmph),
      filter_distance_(filter_distance),
      next_(next) {}


void MotionFilterPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if (batch->index >= 1) {
    LOG(INFO) << "Previous PointsBatch Timestamp: " << batch->prev_start_time;
    LOG(INFO) << "PointsBatch Timestamp: " << batch->start_time;
    LOG(INFO) << "Previous PointsBatch origin: (" << batch->prev_origin[0]
              << ", " << batch->prev_origin[1] << ", " << batch->prev_origin[2]
              << ")";
    LOG(INFO) << "Current PointsBatch origin: (" << batch->origin[0] << ", "
              << batch->origin[1] << ", " << batch->origin[2] << ")";

    const double delta_distance = (batch->prev_origin - batch->origin).norm();
    const double delta_seconds = common::ToSeconds(batch->start_time - batch->prev_start_time);

    const double speed_mps = delta_distance / delta_seconds;
    const double speed_kmph = speed_mps / 1000. * 60. * 60.;

    LOG(INFO) << "delta_distance: " << delta_distance << "m, delta_seconds: "
              << delta_seconds << "seconds, speed_mps: " << speed_mps
              << "m/s, speed_kmph: " << speed_kmph << "km/h";

    if (delta_distance >= filter_distance_ && speed_kmph >= filter_speed_kmph_) {
      next_->Process(std::move(batch));
    }
  }
}

PointsProcessor::FlushResult MotionFilterPointsProcessor::Flush() {
  return next_->Flush();
}

} // namespace io
} // namespace cartographer
