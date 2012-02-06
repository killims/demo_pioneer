/*
 * utils.h
 *
 *  Created on: 14-may-2009
 *      Author: pablo
 */

#ifndef UTILS_H_
#define UTILS_H_

//////////////////////////////////////////////////////////////////////////
#include <string>
#include "gns.h"
#include <list>
#include <algorithm>
//////////////////////////////////////////////////////////////////////////
namespace utils {
    std::string name(const std::string& base, int index,
            const std::string& ext = ".png", int width = 5, char fill = '0');

}

////////////////////////////////////////////////////////////////////////////////
struct change_frame {
        change_frame(const gns::frame& f) :
            m_wxr(f) {
        }

        gns::point operator()(const gns::point p) {
            return m_wxr * p;
        }

    protected:
        gns::frame m_wxr;

};
////////////////////////////////////////////////////////////////////////////////
struct robot_shape {

        std::list<gns::point> operator()(const gns::frame& f, double width,
                double length, double scale = 0.05) const {
            std::list<gns::point> local_points;
            for (double x = 0; x < length / 2; x += scale) {
                local_points.push_back(gns::point(x, width / 2));
                local_points.push_back(gns::point(x, -width / 2));
                local_points.push_back(gns::point(-x, -width / 2));
                local_points.push_back(gns::point(-x, width / 2));
            }

            for (double y = 0; y < width / 2; y += scale) {
                local_points.push_back(gns::point(length / 2, y));
                local_points.push_back(gns::point(length / 2, -y));
                local_points.push_back(gns::point(-length / 2, -y));
                local_points.push_back(gns::point(-length / 2, y));
            }

            std::transform(local_points.begin(), local_points.end(),
                    local_points.begin(), change_frame(f));

            return local_points;
        }
};
////////////////////////////////////////////////////////////////////////////////

#endif /* UTILS_H_ */
