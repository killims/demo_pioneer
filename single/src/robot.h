/*
 * robot.h
 *
 *  Created on: 06-may-2009
 *      Author: pablo
 */

#ifndef ROBOT_H_
#define ROBOT_H_
//////////////////////////////////////////////////////////////////////////
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <libplayerc++/playerc++.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "config.h"
//#include "dstar.h"
#include "dynamicmodel.h"
#include "gns.h"
#include "laser.h"
#include "map.h"
#include "ndproxy.h"

//////////////////////////////////////////////////////////////////////////
class base_robot {
public:
	//constructors
	base_robot(); //needed for singleton (requires default constructor)
	virtual void setup(config& configuration);

	//constructor + setup
	base_robot(config& configuration);

	//public methods
	void start();
	void stop();
	void get_speeds(double& v, double &w);

	IplImage* get_image() {
		return m_image;
	}

	void kill() {
		stop();
		m_end = true;
		boost::mutex::scoped_lock lock(m_active_goal_mutex);
		m_active_goal = true;
		m_active_goal_cond.notify_all();

	}

	void pause() {
		m_pause = !m_pause;
	}

protected:
	void run();

	//player server
	std::string m_host;
	unsigned int m_port;

	//player client
	std::auto_ptr<PlayerCc::PlayerClient> m_client;
	std::auto_ptr<PlayerCc::Position2dProxy> m_position_proxy;
	std::auto_ptr<PlayerCc::LaserProxy> m_laser_proxy;

	//odometry and laser callbacks
	virtual void new_position();
	virtual void new_laser();

	//localization
	gns::frame m_wxr;

	//velocities
	double m_v, m_w;

	//laser
	laser m_laser;

	//map scale
	double m_scale;

	//map
	std::auto_ptr<local_map> m_map;
	double m_width, m_length;
	double m_robot_size;

	//last speeds commanded
	double m_command_v, m_command_w;

	//nd
	std::auto_ptr<nd_proxy> m_nd_proxy;

	template<class InputIterator>
	InputIterator
	subgoal(const grid<400>& g, InputIterator start, InputIterator end);


	bool m_stop, m_end, m_pause;


	double m_desired_distance, m_desired_angle;

	bool m_active_goal;
	boost::condition_variable m_active_goal_cond;
	boost::mutex m_active_goal_mutex;

	//Dstar m_planner;
	void init_planner();
	void update_planner();

	std::string m_start_time;
	std::ofstream m_log;

	IplImage* m_image;
};
////////////////////////////////////////////////////////////////////////////////
template<class InputIterator>
InputIterator base_robot::subgoal(const grid<400>& g, InputIterator begin,
		InputIterator end) {

	typename std::iterator_traits<InputIterator>::difference_type d =
			std::distance(begin, end);

	if (d < 2)
		return begin;

	InputIterator middle = begin;
	std::advance(middle, d / 2);

	pointi b(begin->x, begin->y);
	pointi m(middle->x, middle->y);
	if (g.is_free(b, m))
		return subgoal(g, middle, end);
	else
		return subgoal(g, begin, middle);
}
////////////////////////////////////////////////////////////////////////////////
#endif /* ROBOT_H_ */
