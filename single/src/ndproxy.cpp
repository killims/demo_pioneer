/*
 * nd_proxy.cc
 *
 *  Created on: 10-dic-2008
 *      Author: pablo
 */

#include "ndproxy.h"
#include <string>
#include <sstream>
#include <fstream>
#include <limits>
#include <boost/program_options.hpp>
#include <iostream>
#include <cmath>

namespace bpo = boost::program_options;

nd_proxy::nd_proxy(const std::string& s) {
    load_configuration(s);
    InicializarND(&m_parameters);
}

nd_proxy::nd_proxy(double front, double back, double left) {

    m_parameters.holonomic = 0;
    m_parameters.aamax = 0.75;
    m_parameters.almax = 0.75;
    m_parameters.vamax = M_PI_4;
    m_parameters.vlmax = 0.5;

    m_parameters.geometryRect = 1;

    m_parameters.front = front;
    m_parameters.back = back;
    m_parameters.left = left;
    m_parameters.discontinuity = 2.0 * m_parameters.left;

    m_parameters.dsmax = 0.5;
    //m_parameters.dsmax = 1.0; //mejor adaptada a velocidad y aceleracion maximas
    m_parameters.dsmin = 0.25 * m_parameters.dsmax;
    /*m_parameters.dsmin = 0.5
     * sqrt(pow(0.5 * (front + back), 2) + pow(left, 2));*/
    //m_parameters.enlarge = 0.2;
    m_parameters.enlarge = 0.2 * m_parameters.dsmin;

    m_parameters.T = 0.2;

    InicializarND(&m_parameters);
}

void nd_proxy::update(double x, double y, double theta,
        std::vector<gns::point> scan) {
    update(x, y, theta, scan.begin(), scan.end());
}

void nd_proxy::get_speeds(double goalx, double goaly, double goal_tol,
        double& v, double& w) {

    //check if goal is behind the robot
    gns::point pgoal(goalx, goaly);
    gns::frame wXr(m_movement.SR1.posicion.x, m_movement.SR1.posicion.y,
            m_movement.SR1.orientacion);

    gns::point sgr = ~wXr * pgoal;
    double angle = atan2(sgr.y, sgr.x);
    if (angle < -M_PI / 2.0) {
        sgr.x = 0;
        sgr.y = -1.1 * goal_tol;
        pgoal = wXr * sgr;
    } else if (angle > M_PI / 2.0) {//left
        sgr.x = 0;
        sgr.y = 1.1 * goal_tol;
        pgoal = wXr * sgr;
    }

    TCoordenadas goal;
    goal.x = pgoal.x;
    goal.y = pgoal.y;

    TVelocities* out = IterarND(goal, goal_tol, &m_movement, &m_environment,
            &m_infond);

    if (out) {
        v = out->v;
        w = out->w;
    } else {
        v = 0;
        w = 0;
    }
}

void nd_proxy::load_configuration(const std::string& s) {
    bpo::variables_map vm;

    bpo::options_description movement_options("movement");
    movement_options.add_options()("movement", bpo::value<short int>(
            &m_parameters.holonomic)->default_value(0));
    movement_options.add_options()("almax", bpo::value<float>(
            &m_parameters.almax)->default_value(0.75));
    movement_options.add_options()("aamax", bpo::value<float>(
            &m_parameters.aamax)->default_value(0.75));
    movement_options.add_options()("vlmax", bpo::value<float>(
            &m_parameters.vlmax)->default_value(0.5));
    movement_options.add_options()("vamax", bpo::value<float>(
            &m_parameters.vamax)->default_value(0.79));

    bpo::options_description shape_options("shape");
    shape_options.add_options()("shape", bpo::value<short int>(
            &m_parameters.geometryRect)->default_value(1));
    shape_options.add_options()("radius",
            bpo::value<float>(&m_parameters.R)->default_value(0.402));
    shape_options.add_options()("front",
            bpo::value<float>(&m_parameters.front)->default_value(0.313));
    shape_options.add_options()("back",
            bpo::value<float>(&m_parameters.back)->default_value(0.313));
    shape_options.add_options()("left",
            bpo::value<float>(&m_parameters.left)->default_value(0.253));
    shape_options.add_options()("discontinuity", bpo::value<float>(
            &m_parameters.discontinuity)->default_value(0.5));

    bpo::options_description behaviour_options("behaviour");
    behaviour_options.add_options()("dsmax", bpo::value<float>(
            &m_parameters.dsmax)->default_value(1.0));

    behaviour_options.add_options()("dsmin", bpo::value<float>(
            &m_parameters.dsmin)->default_value(0.25));

    behaviour_options.add_options()("enlarge", bpo::value<float>(
            &m_parameters.enlarge)->default_value(0.05));

    behaviour_options.add_options()("period",
            bpo::value<float>(&m_parameters.T)->default_value(0.2));

    bpo::options_description options;
    options.add(movement_options).add(shape_options).add(behaviour_options);

    std::ifstream ifs(s.c_str());

    if (ifs.fail()) {
        std::cout << "Configuration file not found" << std::endl;
        std::cout << "Using default configuration" << std::endl;
    }

    bpo::store(bpo::parse_config_file(ifs, options), vm);
    bpo::notify(vm);
}

