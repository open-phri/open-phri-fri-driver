/*      File: main.cpp
*       This file is part of the program open-phri-fri-driver
*       Program description : An OpenPHRI driver for the Kuka LWR4 robot, based on the Fast Research Interface.
*       Copyright (C) 2018 -  Benjamin Navarro (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the LGPL license as published by
*       the Free Software Foundation, either version 3
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       LGPL License for more details.
*
*       You should have received a copy of the GNU Lesser General Public License version 3 and the
*       General Public License version 3 along with this program.
*       If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/fri_driver.h>
#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace phri;

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {
	PID_EXE(argv[0]);

	AppMaker app("examples_config/fri-driver-example.yaml");

	/***			Controller configuration			***/
	auto robot = app.getRobot();
	*robot->controlPointDampingMatrix() = phri::Vector6d(app.getParameter<std::vector<double>>("damping").data());
	auto deadband_values = std::make_shared<phri::Vector6d>(app.getParameter<std::vector<double>>("deadband").data());

	auto maximum_velocity = make_shared<double>(app.getParameter<double>("velocity_limit"));

	auto safety_controller = app.getController();
	safety_controller->add(
		"velocity constraint",
		VelocityConstraint(maximum_velocity));

	safety_controller->add(
		"ext force proxy",
		ExternalForce(robot));


	auto deadband = phri::Deadband<phri::Vector6d>(
		robot->controlPointExternalForce(),
		deadband_values);

	bool init_ok = app.init();

	if(init_ok)
		std::cout << "Starting main loop\n";
	else
		std::cout << "Initialization failed\n";

	signal(SIGINT, sigint_handler);

	while(init_ok and not _stop) {
		_stop |= not app.run([&](){deadband(); return true;});
	}

	app.stop();

	return 0;
}
