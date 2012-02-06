/*
 * dynamicmodel.cpp
 *
 *  Created on: 18-dic-2008
 *      Author: pablo
 */

#include "dynamicmodel.h"

dynamic_model::dynamic_model(double F, double b, double ki, double h, double T) :
    m_F(F), m_b(b), m_ki(ki), m_h(h), m_T(T) {
}

std::pair<double, double> dynamic_model::direct(const gns::vector& input,
        const double v, const double w) const {
    gns::vector aux = input;

    if (aux.length() > m_F)
        aux = aux.direction() * m_F;

    return std::pair<double, double>((aux.x - 2 * m_b * v) * m_T + v, (m_h
            * aux.y - 2 * m_b * m_ki * w) * m_T + w);
}

gns::vector dynamic_model::inverse(const double obj_v, const double obj_w,
        const double v, const double w) const {
    double fx = (obj_v - v) / m_T + 2 * m_b * v;
    double fy = (obj_w - w) / (m_T * m_ki * m_h) + 2 * m_b * w / m_h;

    return gns::vector(fx, fy);
}
