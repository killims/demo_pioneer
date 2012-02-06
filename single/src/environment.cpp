/*
 * environment.cc
 *
 *  Created on: 13-ene-2009
 *      Author: pablo
 */

#include "environment.h"
#include <limits>
#include <cmath>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////
void interpolate(std::map<int, double>& m, int begin, int end);
void find_sector(const std::map<int, double>& m, int s, int& begin, int & end);
////////////////////////////////////////////////////////////////////////////////
void environment::setup(double scale, double width, double length,
        double security_distance) {
    m_scale = scale;
    m_width = width;
    m_length = length;
    m_security_distance = security_distance;
}
////////////////////////////////////////////////////////////////////////////////
struct reframe: public std::binary_function<gns::frame, gns::point, gns::point> {
        gns::point operator()(const gns::frame& f, const gns::point& p) const {
            return f * p;
        }
};
////////////////////////////////////////////////////////////////////////////////
struct close_to {
        close_to(const gns::point& p, double distance) :
            m_ref(p), m_distance(distance) {

        }

        bool operator()(const gns::point& p) const {
            return gns::distance(m_ref, p) < m_distance;
        }
    protected:
        gns::point m_ref;
        double m_distance;
};
////////////////////////////////////////////////////////////////////////////////
situation environment::analyze(const gns::point& robot_pose,
        const gns::point& goal_pose, const std::vector<gns::point>& points) const {

    if (points.empty())
        return FREE;

    close_to ct(robot_pose, m_security_distance);
    bool too_close = std::count_if(points.begin(), points.end(), ct) > 0;

    std::vector<gns::point> hpoints = points;

    gns::vector vgoal = goal_pose - robot_pose;

    gns::frame wXh(robot_pose.x, robot_pose.y, vgoal.angle());
    gns::frame hXw = ~wXh;

    //histogram points
    for (size_t i = 0; i < hpoints.size(); i++)
        hpoints[i] = hXw * points[i];

    //compute histogram size
    double max = 0.0;
    double min = 0.0;
    for (std::vector<gns::point>::iterator p = hpoints.begin(); p
            != hpoints.end(); p++) {
        if (p->y < min)
            min = p->y;
        if (p->y > max)
            max = p->y;
    }

    const int SIZE = int(ceil((max - min) / m_scale));

    gns::point hgoal = hXw * goal_pose;

    const int goalsector(int(round((hgoal.y - min) / m_scale)));

    //creating histogram

    std::vector<double> histogram(SIZE);
    std::fill(histogram.begin(), histogram.end(), -1);

    for (std::vector<gns::point>::iterator p = hpoints.begin(); p
            != hpoints.end(); p++) {
        int sector(int(floor((p->y - min) / m_scale)));
        if (p->x > 0 and p->x < hgoal.x and (histogram[sector] < 0 or p->x
                < histogram[sector]))
            //update the closest obstacle in the sector if it is closer than the goal
            histogram[sector] = p->x;
    }

    //fill small free spaces

    {
        int start = int(histogram.size());
        int end = int(histogram.size());
        for (int i = 0; i < int(histogram.size()); i++) {
            if (histogram[i] < 0 and start > i) //new free space
                start = i; //set start
            else if (histogram[i] > 0 and start < i) { //free space ended

                end = i; //set end
                if (start > 0) {
                    gns::point pstart((start - 1) * m_scale, histogram[start
                            - 1]), pend(end * m_scale, histogram[end]);

                    if ((end - (start - 1)) * m_scale < m_width) { //is small

                        //interpolate
                        double a = (histogram[start - 1] - histogram[end])
                                / (start - 1 - end);
                        double b = histogram[start - 1] - a * (start - 1);
                        for (int j = start; j < end; j++)
                            histogram[j] = a * j + b;
                    }
                }
                start = int(histogram.size()); //unset start
            }
        }
    }

    //all freespaces in histogram are big enough for the robot to navigate

    if (histogram[goalsector] < 0) {
        //goalsector is in a free space
        int inicio = goalsector;

        while (inicio >= 0 and histogram[inicio] < 0)
            --inicio;
        if (inicio < 0)
            inicio = 0;
        else
            ++inicio;

        int fin(goalsector + 1);
        while (fin < SIZE and histogram[fin] < 0)
            ++fin;

        if (fin >= SIZE)
            fin = SIZE;

        if (fabs((goalsector - inicio) * m_scale) > m_security_distance
                and fabs((fin - goalsector) * m_scale) > m_security_distance
                and not too_close) {
            //there is enough free space to go through with no care
            return FREE;
        } else
            return AVOIDANCE;
    }

    //the goal is behind an obstacle

    {
        //compute starting point of the obstacle
        int inicio = goalsector;
        while (true) {
            if (inicio < 0) {
                inicio = 0;
                break;
            }

            //inicio >= 0
            if (histogram[inicio] < 0) {
                inicio++;
                break;
            }

            //inicio >= 0 and histogram[inicio] >= 0
            if (inicio == 0) {
                break;
            }

            //inicio > 0 and histogram[inicio] >= 0
            if (histogram[inicio - 1] < 0) {
                break;
            }

            inicio--;
        }

        //compute end point of the obstacle
        int fin(goalsector + 1);

        while (true) {
            if (fin >= SIZE) {
                fin = SIZE;
                break;
            }

            //fin < SIZE
            if (histogram[fin] < 0) {
                break;
            }

            fin++;
        }

        //compute maxima and minima in the obstacle
        std::vector<int> maximun, minimun;
        {
            //all local maxima and minima candidates
            enum state {
                inc, dec, eq
            };
            state s = eq;
            for (int i = inicio; i < fin - 1; i++) {
                double input = histogram[i + 1] - histogram[i];
                if (input > 0) {
                    if (s == dec)
                        minimun.push_back(i);
                    s = inc;
                } else if (input < 0) {
                    if (s == inc)
                        maximun.push_back(i);
                    s = dec;
                }
            }

            //filter small neightbourhood and depth
            std::vector<int> tmp = maximun;
            maximun.clear();
            for (std::vector<int>::iterator i = tmp.begin(); i != tmp.end(); i++) {
                //compute neighbourhood
                int n = 1;
                while (true) {
                    if (*i - n < inicio) {
                        n = *i - inicio;
                        break;
                    }

                    if (*i + n >= fin) {
                        n = fin - *i - 1;
                        break;
                    }

                    if (histogram[*i - n] > histogram[*i] or histogram[*i + n]
                            > histogram[*i]) {
                        n--;
                        break;
                    }
                    n++;
                }

                double depth = histogram[*i] - 0.5 * (histogram[*i - n]
                        + histogram[*i + n - 1]);

                if (n * m_scale > m_width and depth > m_length)
                    maximun.push_back(*i);
            }

            tmp = minimun;
            minimun.clear();
            for (std::vector<int>::iterator i = tmp.begin(); i != tmp.end(); i++) {
                //compute neighbourhood
                int n = 1;
                while (true) {
                    if (*i - n < inicio) {
                        n = *i - inicio;
                        break;
                    }

                    if (*i + n >= fin) {
                        n = fin - *i - 1;
                        break;
                    }

                    if (histogram[*i - n] < histogram[*i] or histogram[*i + n]
                            < histogram[*i]) {
                        n--;
                        break;
                    }
                    n++;
                }

                if (n * m_scale > m_width)
                    minimun.push_back(*i);
            }
        }

        if (not maximun.empty())
            return TRAP;
        else
            return AVOIDANCE;
    }

}
////////////////////////////////////////////////////////////////////////////////
void interpolate(std::map<int, double>& m, int begin, int end) {
    double a = (m[end] - m[begin - 1]) / (end - (begin - 1));
    double b = -a * (begin - 1) + m[begin - 1];

    for (int i = begin; i < end; i++)
        m[i] = a * i + b;
}
////////////////////////////////////////////////////////////////////////////////
void find_sector(const std::map<int, double>& m, int s, int& begin, int & end) {
    begin = s;
    while (begin >= m.begin()->first and m.find(begin) == m.end())
        --begin;

    if (begin < m.begin()->first)
        begin = m.begin()->first;
    else
        ++begin;

    end = s + 1;
    while (end <= m.rbegin()->first and m.find(end) == m.end())
        ++end;

    if (end >= m.rbegin()->first)
        end = m.rbegin()->first + 1;
}
