/*
 * dynamicmodel.h
 *
 *  Created on: 18-dic-2008
 *      Author: pablo
 */

#ifndef DYNAMICMODEL_H_
#define DYNAMICMODEL_H_

#include "gns.h"
#include <utility>

////////////////////////////////////////////////////////////////////////////////
class dynamic_model {
    public:
        dynamic_model(double F, double b, double ki, double h, double T);

        std::pair<double, double> direct(const gns::vector& input,
                const double v, const double w) const;

        gns::vector inverse(const double obj_v, const double obj_w,
                const double v, const double w) const;

    protected:
        double m_F, m_b, m_ki, m_h, m_T;
};

#endif /* DYNAMICMODEL_H_ */
