/*
 * laser.h
 *
 *  Created on: 23-mar-2009
 *      Author: pablo
 */

#ifndef LASER_H_
#define LASER_H_
////////////////////////////////////////////////////////////////////////////////
#include <libplayerc++/playerc++.h>
#include <vector>
#include "gns.h"
////////////////////////////////////////////////////////////////////////////////
class laser {
    public:
        // TYPES
        typedef std::vector<double>::iterator iterator;
        typedef std::vector<double>::const_iterator const_iterator;
        typedef std::vector<double>::reverse_iterator reverse_iterator;
        typedef std::vector<double>::const_reverse_iterator
                const_reverse_iterator;

        // CONSTRUCTORS
        laser(double x, double y, double theta);
        //laser(PlayerCc::LaserProxy& proxy);

        // GETTERS
        double angle(size_t i) const;
        double range(size_t i) const;
        unsigned int count() const;

        // FILLER
        void fill(PlayerCc::LaserProxy& proxy);

        // FILTER
        void filter();

        // ITERATORS
        iterator begin();
        iterator end();
        const_iterator begin() const;
        const_iterator end() const;
        reverse_iterator rbegin();
        reverse_iterator rend();
        const_reverse_iterator rbegin() const;
        const_reverse_iterator rend() const;

        // DATA
        double max_range, min_angle, angle_resolution;
        std::vector<double> ranges;
        gns::frame rXl;
};
////////////////////////////////////////////////////////////////////////////////
inline laser::laser(double x, double y, double theta) :
    rXl(x, y, theta) {
}
////////////////////////////////////////////////////////////////////////////////
inline double laser::angle(size_t i) const {
    return min_angle + angle_resolution * i;
}
////////////////////////////////////////////////////////////////////////////////
inline double laser::range(size_t i) const {
    return ranges[i];
}
////////////////////////////////////////////////////////////////////////////////
inline unsigned int laser::count() const {
    return ranges.size();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::iterator laser::begin() {
    return ranges.begin();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::iterator laser::end() {
    return ranges.end();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::const_iterator laser::begin() const {
    return ranges.begin();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::const_iterator laser::end() const {
    return ranges.end();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::reverse_iterator laser::rbegin() {
    return ranges.rbegin();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::reverse_iterator laser::rend() {
    return ranges.rend();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::const_reverse_iterator laser::rbegin() const {
    return ranges.rbegin();
}
////////////////////////////////////////////////////////////////////////////////
inline laser::const_reverse_iterator laser::rend() const {
    return ranges.rend();
}
////////////////////////////////////////////////////////////////////////////////

#endif /* LASER_H_ */
