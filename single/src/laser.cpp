/*
 * laser.cc
 *
 *  Created on: 23-mar-2009
 *      Author: pablo
 */

#include "laser.h"
#include <cmath>



void laser::fill(PlayerCc::LaserProxy & proxy) {
    max_range = proxy.GetMaxRange();
    min_angle = proxy.GetMinAngle();
    angle_resolution = proxy.GetScanRes();
    ranges.resize(proxy.GetCount());

    for (size_t i = 0; i < ranges.size(); i++)
        ranges[i] = proxy.GetRange(i);
}

void laser::filter() {
    if (count() < 3)
        return;

    for (size_t i = 0; i < ranges.size(); i++) {
        if (i == 0) {
            if (fabs(ranges[0] - ranges[1]) > 0.05)
                ranges[0] = ranges[1];
        } else if (i == ranges.size() - 1) {
            if (fabs(ranges[i - 1] - ranges[i]) > 0.05)
                ranges[i] = ranges[i - 1];
        } else {
            if (fabs(ranges[i - 1] - ranges[i + 1]) < 0.1 and (fabs(ranges[i
                    - 1] - ranges[i]) > 0.05 or fabs(ranges[i + 1] - ranges[i])
                    > 0.05)) {
                ranges[i] = 0.5 * (ranges[i - 1] + ranges[i + 1]);
            }
        }
    }

}

