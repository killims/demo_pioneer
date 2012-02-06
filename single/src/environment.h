/*
 * environment.h
 *
 *  Created on: 13-ene-2009
 *      Author: pablo
 */
////////////////////////////////////////////////////////////////////////////////
#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_
////////////////////////////////////////////////////////////////////////////////
#include "gns.h"
#include <vector>
#include <map>
////////////////////////////////////////////////////////////////////////////////
enum situation {
    FREE, AVOIDANCE, TRAP
};
////////////////////////////////////////////////////////////////////////////////
class environment {
    public:

        void setup(double scale, double width, double length,
                double security_distance);
        situation analyze(const gns::point& robot, const gns::point& goal,
                const std::vector<gns::point>& points) const;

    protected:
        double m_scale;
        double m_width, m_length, m_security_distance;

        void fill_free_spaces(std::map<int, double>& m) const;
};
////////////////////////////////////////////////////////////////////////////////
#endif /* ENVIRONMENT_H_ */
