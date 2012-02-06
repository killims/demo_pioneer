/*
 * nd_proxy.h
 *
 *  Created on: 10-dic-2008
 *      Author: pablo
 */

#ifndef NDPROXY_H_
#define NDPROXY_H_
////////////////////////////////////////////////////////////////////////////////
#include "nd.h"
#include "nd2.h"
#include "gns.h"
#include <string>
#include <vector>
////////////////////////////////////////////////////////////////////////////////
class nd_proxy {
    public:

        nd_proxy(double front, double back, double left);
        nd_proxy(const std::string& s);

        template<class InputIterator>
        void update(const gns::frame& robot, InputIterator begin,
                InputIterator end);

        template<class InputIterator>
        void update(double x, double y, double theta, InputIterator begin,
                InputIterator end);

        void update(double x, double y, double theta,
                std::vector<gns::point> scan);
        void get_speeds(double goalx, double goaly, double goal_tol, double& v,
                double& w);

    protected:
        void load_configuration(const std::string& s);
        TParametersND m_parameters;
        TInfoEntorno m_environment;
        TInfoMovimiento m_movement;
        TInfoND m_infond;
};
////////////////////////////////////////////////////////////////////////////////
template<class InputIterator>
void nd_proxy::update(const gns::frame& robot, InputIterator begin,
        InputIterator end) {
    update(robot.x, robot.y, robot.theta, begin, end);
}
////////////////////////////////////////////////////////////////////////////////
template<class InputIterator>
void nd_proxy::update(double x, double y, double theta, InputIterator begin,
        InputIterator end) {
    m_movement.SR1.orientacion = theta;
    m_movement.SR1.posicion.x = x;
    m_movement.SR1.posicion.y = y;

    m_environment.longitud = 0;
    //gns::point r(x, y);
    for (InputIterator p = begin; p != end; p++) {
        m_environment.punto[m_environment.longitud].x = p->x;
        m_environment.punto[m_environment.longitud].y = p->y;
        m_environment.longitud++;
    }
}
////////////////////////////////////////////////////////////////////////////////
#endif /* NDPROXY_H_ */
