#ifndef ITRI_IO_HEIGHT_FILTERING_POINTS_PROCESSOR_H_
#define ITRI_IO_HEIGHT_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer{
namespace io{

class HeightFilteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "height_filter";
  HeightFilteringPointsProcessor(double height, PointsProcessor* next);

  static std::unique_ptr<HeightFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~HeightFilteringPointsProcessor() override {}

  HeightFilteringPointsProcessor(
      const HeightFilteringPointsProcessor&) = delete;
  HeightFilteringPointsProcessor& operator=(
      const HeightFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
   const double height_;
   PointsProcessor* const next_;
};


} // namespace io
} // namespace cartographer

#endif // ITRI_IO_HEIGHT_FILTERING_POINTS_PROCESSOR_H_
