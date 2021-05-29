#include "IOdometer.hpp"

namespace vlo {

Transform3D IOdometer::getLastPose() const { return trajectory_.back(); }

}  // namespace vlo
