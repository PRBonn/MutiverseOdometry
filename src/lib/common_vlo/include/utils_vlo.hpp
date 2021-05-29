#pragma once

#include <utility>
#include "common_vlo.hpp"
#include "IOdometer.hpp"
namespace vlo
{
/**
   * @brief Function that sorts pairs by first element
   * @param a first pair
   * @param b second pair
   * @return success
   */
bool sortbyfirst(const std::pair<double, double> &a,
                 const std::pair<double, double> &b);


struct OdometerStruct{
    IOdometer::Ptr odometer;
    Transform3D reference_orientation = Transform3D::Identity();
};

}
