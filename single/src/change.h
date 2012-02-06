/*
 * change.h
 *
 *  Created on: 01-abr-2009
 *      Author: pablo
 */

#ifndef CHANGE_H_
#define CHANGE_H_
////////////////////////////////////////////////////////////////////////////////
#include "gns.h"
#include <utility>
#include <cmath>
#include "pointi.h"
////////////////////////////////////////////////////////////////////////////////
class change {
    public:
        change(const int size, const double scale, const gns::point& center);
        pointi toCell(const gns::point& p) const;
        gns::point toPoint(const pointi& p) const;

    protected:
        int m_size;
        double m_scale;
        gns::frame m_center;

};
////////////////////////////////////////////////////////////////////////////////
inline change::change(const int size, const double scale,
        const gns::point& center) :
    m_size(size), m_scale(scale), m_center(center.x, center.y, 0) {
}
////////////////////////////////////////////////////////////////////////////////
inline pointi change::toCell(const gns::point& p) const {
    gns::point aux = (~m_center) * p;
    return pointi(int(round(aux.x / m_scale)) + m_size / 2, int(round(aux.y
            / m_scale)) + m_size / 2);
}
////////////////////////////////////////////////////////////////////////////////
inline gns::point change::toPoint(const pointi& p) const {
    gns::point out((p.x - m_size / 2) * m_scale + m_center.x,
            (p.y - m_size / 2) * m_scale + m_center.y);

    return out;
}
////////////////////////////////////////////////////////////////////////////////
#endif /* CHANGE_H_ */
