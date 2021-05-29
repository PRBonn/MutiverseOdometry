#include "utils_vlo.hpp"

namespace vlo {

bool sortbyfirst(const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
    return (a.first < b.first);
}

}
