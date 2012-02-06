/*
 * map.cpp
 *
 *  Created on: 13-ene-2009
 *      Author: pablo
 */

////////////////////////////////////////////////////////////////////////////////
#include "map.h"
#include <limits>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include "line.h"
#include <set>
#include <queue>

////////////////////////////////////////////////////////////////////////////////
local_map::local_map(double robot_size, int size, double scale) :
	m_robot_size(robot_size), m_scale(scale), m_moved(false) {
	m_grid.clear();
	m_masked.clear();
	m_mask = create_mask(0.8 * robot_size);
	m_laser.clear();
}
////////////////////////////////////////////////////////////////////////////////
void local_map::set_robot(const gns::frame& r) {
	static bool first_time = true;
	m_moved = false;

	m_wXr = r;

	if (first_time) {
		m_center = r.zero();
		first_time = false;
	} else if (gns::distance(r.zero(), m_center) > 1.0) {
		pointi delta = get_change().toCell(r.zero());
		move(delta);
		m_center = r.zero();
		m_moved = true;
	}
}
////////////////////////////////////////////////////////////////////////////////
void local_map::add_update(const gns::frame& f, const laser& l) {
	grid<400> projection;
	laser_projection(f, l, projection);
	m_updates.fuse(projection);
}
////////////////////////////////////////////////////////////////////////////////
void local_map::apply_updates() {
	//remove all the points inside the robot
	//clean_robot(m_updates);
	grid<400> updates_masked;
	m_updates.mask(m_mask, updates_masked);
	apply(m_masked, updates_masked, m_changes, cell::diff);
	m_grid.patch(m_updates);
	m_grid.mask(m_mask, m_masked);
}
////////////////////////////////////////////////////////////////////////////////
void local_map::clear_updates() {
	m_updates.clear();
}
////////////////////////////////////////////////////////////////////////////////
const grid<400>& local_map::get_updates() const {
	return m_changes;
}
////////////////////////////////////////////////////////////////////////////////
void local_map::laser_projection(const gns::frame& f, const laser& laser, grid<
		400>& out) {
	out.clear();
	gns::frame wXl = f * laser.rXl;
	change change = get_change();
	pointi lasercoord = change.toCell(wXl.zero());
	for (unsigned int i = 0; i < laser.count(); i++) {
		gns::point p = wXl * gns::point::polar(laser.range(i), laser.angle(i));
		if (gns::distance(p, m_wXr.zero()) <= m_robot_size)
			continue;

		pointi gcp = change.toCell(p);
		my::line theline(lasercoord, gcp);
		for (my::line::iterator j = theline.begin(); j != theline.end(); ++j) {
			if (out.valid(j->x, j->y) and out(j->x, j->y) == cell::UNKNOWN)
				out(j->x, j->y) = cell::FREE;
		}
		if (laser.range(i) < laser.max_range and out.valid(gcp.x, gcp.y))
			out(gcp.x, gcp.y) = cell::NONFREE;
	}

	for (int i = int(laser.count()) - 1; i >= 0; i--) {
		gns::point p = wXl * gns::point::polar(laser.range(i), laser.angle(i));
		if (gns::distance(p, m_wXr.zero()) <= m_robot_size)
			continue;

		pointi gcp = change.toCell(p);
		my::line theline(lasercoord, gcp);
		for (my::line::iterator j = theline.begin(); j != theline.end(); ++j) {
			if (out.valid(j->x, j->y) and out(j->x, j->y) == cell::UNKNOWN)
				out(j->x, j->y) = cell::FREE;
		}
		if (laser.range(i) < laser.max_range and out.valid(gcp.x, gcp.y))
			out(gcp.x, gcp.y) = cell::NONFREE;
	}
}
////////////////////////////////////////////////////////////////////////////////
grid<10> local_map::create_mask(double radius) {
	unsigned int rad = (unsigned int) (ceil(radius / m_scale));
	grid<10> out;
	for (unsigned int i = 0; i < out.size(); i++)
		for (unsigned int j = 0; j < out.size(); j++)
			if ((i - rad) * (i - rad) + (j - rad) * (j - rad) <= rad * rad)
				out(i, j) = cell::NONFREE;

	return out;
}
////////////////////////////////////////////////////////////////////////////////
void local_map::move(const pointi& newc) {
	int deltax = newc.x - m_grid.size() / 2;
	int deltay = newc.y - m_grid.size() / 2;
	m_grid.move(-deltax, -deltay);
}
////////////////////////////////////////////////////////////////////////////////
bool local_map::is_free(const gns::point& start, const gns::point& end) const {
	return m_grid.is_free(get_change().toCell(start), get_change().toCell(end));
}
////////////////////////////////////////////////////////////////////////////////
std::vector<gns::point> local_map::get_points() const {
	std::vector<gns::point> out;
	for (unsigned int i = 0; i < m_grid.size(); i++)
		for (unsigned int j = 0; j < m_grid.size(); j++)
			if (m_grid(i, j) == cell::NONFREE)
				out.push_back(get_change().toPoint(pointi(i, j)));

	return out;
}
////////////////////////////////////////////////////////////////////////////////
void fill(grid<400>& g, pointi c, int r);
////////////////////////////////////////////////////////////////////////////////
void local_map::clean_robot(const gns::frame& rf) {
	pointi c = get_change().toCell(rf.zero());
	int r = int(round(m_robot_size / m_scale));
	fill(m_updates, c, r);
}
////////////////////////////////////////////////////////////////////////////////
void fill(grid<400>& g, pointi c, int r) {
	std::queue<pointi> open;
	std::set<pointi> close;

	open.push(pointi(0, 0));

	while (not open.empty()) {
		pointi p = open.front();
		open.pop();

		if (close.find(p) != close.end())
			continue;
		close.insert(p);

		if (g.valid(c.x + p.x, c.y + p.y) and (p.x * p.x + p.y * p.y <= r * r)) {
			g(c.x + p.x, c.y + p.y) = cell::FREE;
			open.push(pointi(p.x + 1, p.y));
			open.push(pointi(p.x - 1, p.y));
			open.push(pointi(p.x, p.y + 1));
			open.push(pointi(p.x, p.y - 1));
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////
void local_map::line_projection(double x, double y, double theta,
		double length, double sy, double stheta, double sytheta, grid<400>& out) {
	out.clear();
	gns::frame line_frame(x, y, theta);
	gns::point line_begin = line_frame * gns::point(-length / 2, 0);
	gns::point line_end = line_frame * gns::point(length / 2, 0);
	change change = get_change();
	pointi begin = change.toCell(line_begin);
	pointi end = change.toCell(line_end);

	my::line cells(begin, end);
	for (my::line::iterator i = cells.begin(); i != cells.end(); ++i) {
		gns::point p = (~line_frame) * change.toPoint(*i);
		double error = sy + 2 * sytheta * p.x + stheta * p.x * p.x;
		gns::frame pframe(p.x, p.y, theta + M_PI / 2);
		gns::point error_begin = line_frame * pframe * gns::point(-error, 0);
		gns::point error_end = line_frame * pframe * gns::point(error, 0);
		pointi ebegin = change.toCell(error_begin);
		pointi eend = change.toCell(error_end);

		out.drawline(ebegin, eend, cell::NONFREE);
	}
}
