/*
 * main_leader.cpp
 *
 *  Created on: 21-may-2009
 *      Author: pablo
 */

#include "leader.h"
#include "singleton.h"
#include "config.h"
#include "random.h"
#include "timer.h"
#include <iostream>
#include <fstream>

#include <boost/bind.hpp>

namespace {
void start_robot(base_robot* br) {
	br->start();
}

void on_mouse(int event, int x, int y, int flags, void* param) {
	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		leader* therobot = (leader*)param;

		therobot->set_goal(y, x);
	}
}

}

int main(int argc, char **argv) {
	if (argc < 2) {
		std::cout << "ERROR: Wrong parameters" << std::endl;
		std::cout << "USAGE: " << argv[0] << " <configuration file>"
				<< std::endl;
		return 1;
	}

	config c(argv[1]);

	leader therobot;// = singleton<leader>::instance();

	therobot.setup(c);
	boost::thread th(boost::bind(start_robot, &therobot));
	//therobot->start();
	cvNamedWindow("Navegación Autónoma", 0);
	cvSetMouseCallback("Navegación Autónoma", on_mouse, &therobot);
	char key;
	do {
		cvShowImage("Navegación Autónoma", therobot.get_image());
		key = cvWaitKey(25);
		if (key == ' ')
			therobot.pause();
	} while (key != 27);

	therobot.kill();
	th.join();
	std::cout << "bye" << std::endl;
	return 0;
}
