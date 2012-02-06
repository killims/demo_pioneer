/*
 * leader.cpp
 *
 *  Created on: 08-may-2009
 *      Author: pablo
 */
////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <cmath>
#include <iostream>

#include <boost/array.hpp>

#include "astar.h"
#include "cell.h"
#include "change.h"
#include "grid.h"
#include "leader.h"
#include "random.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////////
leader::leader() :
	base_robot() {
}
////////////////////////////////////////////////////////////////////////////////
leader::leader(config& configuration) :
	base_robot(configuration) {
}
////////////////////////////////////////////////////////////////////////////////
void leader::setup(config& configuration) {
	base_robot::setup(configuration);
	std::ostringstream goals_oss;
	goals_oss << m_start_time << "-" << m_port << "-goals.log";
	m_goal_log.open(goals_oss.str().c_str());

	//cvNamedWindow("window", CV_WINDOW_AUTOSIZE);
}
////////////////////////////////////////////////////////////////////////////////
void leader::new_position() {

	static bool first_time = true;

	base_robot::new_position();

	if (gns::distance(m_goal, m_wxr.zero()) < 0.5) {
		m_goal_reached = true;
		m_active_goal = false;
		m_goal_log << " 1" << std::endl;
		//stop();
	}

	pointi r = m_map->get_change().toCell(m_wxr.zero());
	pointi g = m_map->get_change().toCell(m_goal);

	if (first_time) {
		//m_planner.init(r.x, r.y, g.x, g.y);
		first_time = false;
	} else if (m_map->moved()) {
		m_path.clear();
		//m_planner.init(r.x, r.y, g.x, g.y);
		//fill the grid again
		//init_planner();
	} /*else
	 m_planner.updateStart(r.x, r.y);*/
}
namespace {
bool obstacle(const grid<400>& map, const state& s) {
	if (s.x < 0 or s.y < 0 or s.x >= 400 or s.y >= 400)
		return false;
	return map(s.x, s.y) == cell::NONFREE;
}

double cost(const grid<400>& map, const state& s) {
	if (s.x < 0 or s.y < 0 or s.x >= 400 or s.y >= 400)
		return 2;
	switch (map(s.x, s.y)) {
	case cell::NONFREE:
		return std::numeric_limits<double>::infinity();
	case cell::FREE:
		return 1.0;
	default:
		return 2.0;
	}
}
}
////////////////////////////////////////////////////////////////////////////////
void leader::new_laser() {
	base_robot::new_laser();
	//update_planner();

	change mapchange = m_map->get_change();

	pointi goal = mapchange.toCell(m_goal);
	//compute subgoal
	gns::point subg = m_goal;

	astar planner;
	pointi r = m_map->get_change().toCell(m_wxr.zero());
	pointi g = m_map->get_change().toCell(m_goal);

	timer t;
	t.start();
	if (planner.compute_path(state(r.x, r.y), state(g.x, g.y),
			m_map->get_masked_grid()))
		m_path = planner.get_path();

	subg = compute_subgoal(m_goal);

	//update nd
	std::vector<gns::point> points = m_map->get_points();
	m_nd_proxy->update(m_wxr, points.begin(), points.end());

	double v, w;
	m_nd_proxy->get_speeds(subg.x, subg.y, 0.25, v, w);

	//compute speeds
	m_command_v = v;
	m_command_w = w;

	//drawing
	m_map->get_grid().save(m_image);

	if (m_active_goal) {
    pixel color;    
    color.r = m_path.empty()? 255: 0;
    color.g = m_path.empty()? 0: 255;
    color.b = 0;
    draw_path(color);
		pointi p = mapchange.toCell(m_goal);
    cvCircle(m_image, cvPoint(p.y, p.x), 3, m_path.empty()?CV_RGB(255, 0,0):CV_RGB(0, 255,0), CV_FILLED);
	}

	draw_robot();

	//cvFlip(m_image, m_image, 0);

}
////////////////////////////////////////////////////////////////////////////////
void leader::set_goal(const gns::point& goal) {
	if (not m_goal_reached)
		m_goal_log << " 0" << std::endl;
	m_goal_reached = false;
	m_goal = goal;
	m_goal_log << m_goal.x << " " << m_goal.y;
	pointi g = m_map->get_change().toCell(m_goal);
	//m_planner.updateGoal(g.x, g.y);

}
////////////////////////////////////////////////////////////////////////////////
void leader::set_goal(int x, int y) {
	m_goal = m_map->get_change().toPoint(pointi(x, y));
	boost::mutex::scoped_lock lock(m_active_goal_mutex);
	m_active_goal = true;
}
////////////////////////////////////////////////////////////////////////////////
gns::point leader::compute_subgoal(const gns::point& goal) {
	//std::list<state> path = m_planner.getPath();

	gns::point out = goal;
	if (not m_path.empty()) {

		std::list<state>::iterator s = subgoal(m_map->get_masked_grid(),
				m_path.begin(), m_path.end());
		out = m_map->get_change().toPoint(pointi(s->x, s->y));

		if (gns::distance(out, m_wxr.zero()) < 0.25) {
			gns::vector v = out - m_wxr.zero();
			out = m_wxr.zero() + (v.direction() * 0.5);
		}
	} else {		
    return goal;
	}

	return out;
}
////////////////////////////////////////////////////////////////////////////////
void leader::draw_path(const pixel& color) {
  //pixel color;
  //color.r = 255;
  //color.g = color.b = 0;
	image pic(m_image);
	for (std::list<state>::iterator i = m_path.begin(); i != m_path.end(); i++) {
		if (0 <= i->x and i->x < m_image->width and 0 <= i->y and i->y
				< m_image->height)
			pic(i->x, i->y) = color;
	}
}
////////////////////////////////////////////////////////////////////////////////
void leader::draw_robot() {
	change mapchange = m_map->get_change();

	boost::array<CvPoint, 3> robot_points;

	gns::point bl = m_wxr * gns::point(-m_length / 2, m_width / 2);
	gns::point br = m_wxr * gns::point(-m_length / 2, -m_width / 2);
	gns::point fr = m_wxr * gns::point(m_length / 2, 0);

	robot_points[0] = mapchange.toCell(bl);
	robot_points[1] = mapchange.toCell(br);
	robot_points[2] = mapchange.toCell(fr);

	cvFillConvexPoly(m_image, robot_points.elems, 3, CV_RGB(0, 0, 255));
}

