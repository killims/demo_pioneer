/*
 * map.h
 *
 *  Created on: 13-ene-2009
 *      Author: pablo
 */

#ifndef MAP_H_
#define MAP_H_
////////////////////////////////////////////////////////////////////////////////
#include <utility>
#include <vector>
#include <set>
#include "gns.h"
#include "grid.h"
#include "laser.h"
#include "change.h"
////////////////////////////////////////////////////////////////////////////////
class local_map {
public:
	local_map(double robot_size, int size = 400, double scale = 0.05);
	void set_robot(const gns::frame& robot);
	bool moved() const;

	std::vector<gns::point> get_points() const;

	bool is_free(const gns::point& start, const gns::point& end) const;
	change get_change() const;

	//updates management
	void clear_updates();
	void add_update(const gns::frame& f, const laser& l);
	const grid<400>& get_updates() const;
	void apply_updates();

	unsigned int size() const;
	const grid<400>& get_grid() const;
	const grid<400>& get_masked_grid() const;
	const grid<10>& get_mask() const;

	void clean_robot(const gns::frame& rf);


	void line_projection(double x, double y, double theta, double length,
					double sy, double stheta, double sytheta, grid<400>& out);
private:
	double m_robot_size;
	double m_scale;
	grid<400> m_grid, m_laser, m_updates, m_changes, m_masked;
	grid<10> m_mask;
	std::set<pointi> m_points;
	gns::point m_center;
	gns::frame m_wXr;
	bool m_moved;



	void laser_projection(const gns::frame& f, const laser& laser,
			grid<400>& out);
	grid<10> create_mask(double radius);
	void move(const pointi& newc);

	template<int Size>
	int points(grid<Size>& g, cell::type c) const {
		int out = 0;
		for (unsigned int i = 0; i < g.size(); i++)
			for (unsigned int j = 0; j < g.size(); j++)
				if (g(i, j) == c)
					out++;

		return out;
	}

};
////////////////////////////////////////////////////////////////////////////////
inline change local_map::get_change() const {
	return change(m_grid.size(), m_scale, m_center);
}
////////////////////////////////////////////////////////////////////////////////
inline bool local_map::moved() const {
	return m_moved;
}
////////////////////////////////////////////////////////////////////////////////
inline unsigned int local_map::size() const {
	return m_grid.size();
}
////////////////////////////////////////////////////////////////////////////////
inline const grid<400>& local_map::get_grid() const {
	return m_grid;
}
////////////////////////////////////////////////////////////////////////////////
inline const grid<400>& local_map::get_masked_grid() const {
	return m_masked;
}
////////////////////////////////////////////////////////////////////////////////
inline const grid<10>& local_map::get_mask() const {
	return m_mask;
}
////////////////////////////////////////////////////////////////////////////////
#endif /* MAP_H_ */
