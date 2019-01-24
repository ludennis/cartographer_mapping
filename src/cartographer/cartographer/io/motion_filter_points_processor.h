#ifndef ITRI_IO_MOTION_FILTER_POINTS_PROCESSOR_H_
#define ITRI_IO_MOTION_FILTER_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/points_processor.h"
#include "cartographer/common/lua_parameter_dictionary.h"

namespace cartographer{
namespace io{

// filter out points batch with speed and delta distance less than filtering value
class MotionFilterPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "motion_filter";
  MotionFilterPointsProcessor(
      double filter_speed_kmph, double filter_distance,
      PointsProcessor* next);

  static std::unique_ptr<MotionFilterPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~MotionFilterPointsProcessor() override {}

  MotionFilterPointsProcessor(const MotionFilterPointsProcessor&) = delete;
  MotionFilterPointsProcessor& operator=(
      const MotionFilterPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double filter_speed_kmph_;
  const double filter_distance_;
  PointsProcessor* const next_;
};

} // namespace io
} // namespace cartographer

#endif // ITRI_IO_MOTION_FILTER_POINTS_PROCESSOR_H_
