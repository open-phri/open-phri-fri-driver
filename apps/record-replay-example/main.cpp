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

int main() {
	AppMaker app("examples_config/record-replay-example.yaml");

	/***			Controller configuration			***/
	auto robot = app.getRobot();
	auto controller = app.getController();

	std::function<bool(void)> pre_controller_task;
	bool init_ok;

	auto init =
		[&](){
			init_ok = app.init();
			if(not init_ok) {
				throw std::runtime_error("Initialization failed");
			}
		};
	auto main_loop =
		[&](){
			if(init_ok)
				std::cout << "Starting main loop\n";
			else
				std::cout << "Initialization failed\n";

			signal(SIGINT, sigint_handler);

			_stop = false;
			while(init_ok and not _stop) {
				_stop |= not app.run(pre_controller_task);
			}
		};

	if(app.getParameter<bool>("record")) {
		std::cout << "Recording" << std::endl;
		*robot->controlPointDampingMatrix() = phri::Vector6d(app.getParameter<std::vector<double>>("damping").data());
		auto deadband_values = std::make_shared<phri::Vector6d>(app.getParameter<std::vector<double>>("deadband").data());

		auto maximum_velocity = make_shared<double>(app.getParameter<double>("velocity_limit"));

		controller->add(
			"velocity constraint",
			VelocityConstraint(maximum_velocity));

		controller->add(
			"ext force proxy",
			ExternalForce(robot));

		auto deadband = phri::Deadband<phri::Vector6d>(
			robot->controlPointExternalForce(),
			deadband_values);

		pre_controller_task = [&](){deadband(); return true;};

		init();
		main_loop();
	}
	else {
		std::cout << "Replaying" << std::endl;

		auto joint_init_position = std::make_shared<phri::VectorXd>(robot->jointCount());
		phri::DataReplayer<phri::VectorXdPtr>("/tmp/log_jointCurrentPosition.txt", joint_init_position, joint_init_position->size(), phri::SkipRows(0), phri::SkipCols(1)).process();
		std::cout << "joint_init_position: " << joint_init_position->transpose() << std::endl;

		auto joint_replay_velocity = std::make_shared<phri::VectorXd>(robot->jointCount());
		phri::DataReplayer<phri::VectorXdPtr> joint_velocity_replayer("/tmp/log_jointVelocity.txt", joint_replay_velocity, joint_replay_velocity->size(), phri::SkipRows(0), phri::SkipCols(1));

		phri::TrajectoryPoint<phri::VectorXd> joint_start_point;
		phri::TrajectoryPoint<phri::VectorXd> joint_end_point;

		joint_start_point.resize(robot->jointCount());
		joint_end_point.resize(robot->jointCount());

		*joint_end_point.y = *joint_init_position;

		auto joint_trajectory_generator = phri::TrajectoryGenerator<phri::VectorXd>(
			joint_start_point,
			app.getDriver()->getSampleTime(),
			phri::TrajectorySynchronization::SynchronizeTrajectory);

		phri::VectorXd dqmax(robot->jointCount()), d2qmax(robot->jointCount());
		dqmax.setConstant(1.5);
		d2qmax.setConstant(0.5);
		joint_trajectory_generator.addPathTo(joint_end_point, dqmax, d2qmax);

		controller->add(
			"joint traj vel",
			phri::JointVelocityProxy(joint_trajectory_generator.getVelocityOutput()));

		init();

		*joint_start_point.y = *robot->jointCurrentPosition();

		joint_trajectory_generator.computeTimings();

		pre_controller_task = [&](){return not joint_trajectory_generator.compute();};
		main_loop();

		controller->removeJointVelocityGenerator("joint traj vel");
		controller->add(
			"joint replay velocity",
			JointVelocityProxy(joint_replay_velocity));

		std::cout << "Starting replay" << std::endl;
		pre_controller_task = [&](){return joint_velocity_replayer();};
		main_loop();
		std::cout << "End of replay" << std::endl;
	}

	app.stop();
}
