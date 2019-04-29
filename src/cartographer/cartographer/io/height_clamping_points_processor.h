#ifndef ITRI_IO_HEIGHT_CLAMPING_POINTS_PROCESSOR_H_
#define ITRI_IO_HEIGHT_CLAMPING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer{
namespace io{

class HeightClampingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "height_clamping";
  HeightClampingPointsProcessor(
      double min_height, double max_height, PointsProcessor* next);

  static std::unique_ptr<HeightClampingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~HeightClampingPointsProcessor() override {}

  HeightClampingPointsProcessor(
      const HeightClampingPointsProcessor&) = delete;
  HeightClampingPointsProcessor& operator=(
      const HeightClampingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
   const double min_height_;
   const double max_height_;
   PointsProcessor* const next_;
};

} // namespace io
} // namespace cartographer

#endif // ITRI_IO_HEIGHT_CLAMPING_POINTS_PROCESSOR_H_
