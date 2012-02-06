/*
 * robot.cpp
 *
 *  Created on: 06-may-2009
 *      Author: pablo
 */

#include <iostream>
#include <boost/signals.hpp>
#include <sstream>
#include <iomanip>
#include "timer.h"
#include "robot.h"
#include "utils.h"
#include <opencv/highgui.h>
//////////////////////////////////////////////////////////////////////////
base_robot::base_robot() :
	m_laser(0.16, 0, 0) {
	m_command_v = 0.0;
	m_command_w = 0.0;
}
//////////////////////////////////////////////////////////////////////////
base_robot::base_robot(config& configuration) :
	m_laser(0.16, 0, 0) {
	m_command_v = 0.0;
	m_command_w = 0.0;
	setup(configuration);
}
//////////////////////////////////////////////////////////////////////////
void base_robot::setup(config& configuration) {
	//fill the time string
	time_t seconds_from_epoch;
	time(&seconds_from_epoch);
	tm* actual_time = localtime(&seconds_from_epoch);

	std::ostringstream date_oss;
	date_oss << "logs/";
	date_oss.fill('0');
	date_oss << actual_time->tm_year + 1900 << "-";
	date_oss.width(2);
	date_oss << actual_time->tm_mon + 1;
	date_oss << "-";
	date_oss.width(2);
	date_oss << actual_time->tm_mday;
	date_oss << "-";
	date_oss.width(2);
	date_oss << actual_time->tm_hour;
	date_oss << ":";
	date_oss.width(2);
	date_oss << actual_time->tm_min;
	m_start_time = date_oss.str();

	//player server info
	m_host = configuration.get<std::string> ("host");

	m_port = configuration.get<unsigned int> ("port");

	//player client setup
	try {
		m_client.reset(new PlayerCc::PlayerClient(m_host, m_port));

		// communication setup
		m_client->SetDataMode(PLAYER_DATAMODE_PULL);
		m_client->SetReplaceRule(true, PLAYER_MSGTYPE_DATA, -1, -1);

		m_position_proxy.reset(new PlayerCc::Position2dProxy(m_client.get(), 1));
		m_laser_proxy.reset(new PlayerCc::LaserProxy(m_client.get()));

		//enabling movement
		m_position_proxy->SetMotorEnable(true);
	} catch (PlayerCc::PlayerError& e) {
		std::cout << e.GetErrorFun() << " " << e.GetErrorStr() << std::endl;
	}

	//map setup
	unsigned int map_size = configuration.get<int> ("map_size");
	m_scale = configuration.get<double> ("scale");
	m_width = configuration.get<double> ("width");
	m_length = configuration.get<double> ("length");
	m_robot_size = 0.5 * sqrt(pow(m_width, 2) + pow(m_length, 2));
	m_map.reset(new local_map(m_robot_size, map_size, m_scale));

	//nd setup
	std::string ndconf = configuration.get<std::string> ("ndconf");
	m_nd_proxy.reset(new nd_proxy(ndconf));

	//logs
	std::ostringstream log_oss;
	log_oss << m_start_time << "-" << m_port << ".log";
	m_log.open(log_oss.str().c_str());

	m_image = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 3);
	cvFillImage(m_image, 128);
}
//////////////////////////////////////////////////////////////////////////
void base_robot::start() {
	m_stop = false;
	run();
}
//////////////////////////////////////////////////////////////////////////
void base_robot::stop() {
	m_command_v = 0.0;
	m_command_w = 0.0;
	m_stop = true;
}
////////////////////////////////////////////////////////////////////////////////
void base_robot::run() {
	const double cycletime = 100; //ms
	timer t;
	m_end = false;
	m_pause = false;
	m_active_goal = false;
	while (not m_end) {

		while (not m_stop) {
			t.start();
			m_client->Read();

			if (m_position_proxy->IsFresh()) {
				new_position();
				m_position_proxy->NotFresh();
			}

			if (m_laser_proxy->IsFresh()) {
				new_laser();
				m_laser_proxy->NotFresh();
			}

			if (m_active_goal and not m_pause)
				m_position_proxy->SetSpeed(m_command_v, m_command_w);
			else
				m_position_proxy->SetSpeed(0, 0);

			t.stop();
			unsigned int sleeptime;
			double tosleep = cycletime - t.ms_elapsed();
			if (tosleep < 0)
				sleeptime = 10;
			else
				sleeptime = static_cast<unsigned int> (round(tosleep * 1e3));
			usleep(sleeptime);
		}
	}
}
//////////////////////////////////////////////////////////////////////////
void base_robot::new_position() {
	m_wxr.x = m_position_proxy->GetXPos();
	m_wxr.y = m_position_proxy->GetYPos();
	m_wxr.theta = m_position_proxy->GetYaw();

	m_v = m_position_proxy->GetXSpeed();
	m_w = m_position_proxy->GetYawSpeed();
	m_log << m_wxr.x << " " << m_wxr.y << " " << m_wxr.theta << " " << m_v
			<< " " << m_w << std::endl;

	m_map->set_robot(m_wxr);

}
//////////////////////////////////////////////////////////////////////////
void base_robot::new_laser() {
	m_map->clear_updates();

	m_laser.fill(*m_laser_proxy);
	m_laser.max_range = 7.9;

	m_map->add_update(m_wxr, m_laser);

	//m_map->clean_robot(m_wxr);
	m_map->apply_updates();
}
//////////////////////////////////////////////////////////////////////////
void base_robot::get_speeds(double& v, double &w) {
	v = m_command_v;
	w = m_command_w;
}
////////////////////////////////////////////////////////////////////////////////
/*
 void base_robot::init_planner() {
 const grid<400>& g = m_map->get_masked_grid();
 for (unsigned int i = 0; i < g.size(); i++)
 for (unsigned int j = 0; j < g.size(); j++)
 if (g(i, j) == cell::NONFREE)
 m_planner.updateCell(i, j, -1);
 }
 //*/
////////////////////////////////////////////////////////////////////////////////
/*
 void base_robot::update_planner() {
 const grid<400>& g = m_map->get_updates();
 for (unsigned int i = 0; i < g.size(); i++) {
 for (unsigned int j = 0; j < g.size(); j++) {
 switch (g(i, j)) {
 case cell::TOFREE:
 m_planner.updateCell(i, j, 1);
 break;
 case cell::TONONFREE:
 m_planner.updateCell(i, j, -1);
 break;
 default:
 break;
 }

 }
 }
 }
 //*/
////////////////////////////////////////////////////////////////////////////////
