/*      File: fri_driver.cpp
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
#include <OpenPHRI/drivers/fri_driver.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <LWRBaseControllerInterface.h>
#include <OSAbstraction.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using namespace phri;

bool FRIDriver::registered_in_factory = phri::DriverFactory::add<FRIDriver>("fri");

struct FRIDriver::pImpl {
	pImpl(double sample_time, int port) :
		sample_time(sample_time)
	{
		fri = std::make_unique<LWRBaseControllerInterface>(sample_time, port);
	}
	std::unique_ptr<LWRBaseControllerInterface> fri;
	double sample_time;
};

FRIDriver::FRIDriver(
	phri::RobotPtr robot,
	double sample_time,
	int port) :
	Driver(robot, sample_time)
{
	assert(robot->jointCount() == 7);

	sample_time_ = sample_time;
	impl_ = std::make_unique<FRIDriver::pImpl>(sample_time_, port);
}

FRIDriver::FRIDriver(
	const phri::RobotPtr& robot,
	const YAML::Node& configuration) :
	Driver(robot, 0.)
{
	assert(robot->jointCount() == 7);

	const auto& fri = configuration["driver"];

	int port;
	if(fri) {
		try {
			sample_time_ = fri["sample_time"].as<double>();
		}
		catch(...) {
			throw std::runtime_error(OPEN_PHRI_ERROR("You must provide a 'sample_time' field in the FRI configuration."));
		}
		try {
			port = fri["port"].as<double>();
		}
		catch(...) {
			throw std::runtime_error(OPEN_PHRI_ERROR("You must provide a 'port' field in the FRI configuration."));
		}
	}
	else {
		throw std::runtime_error(OPEN_PHRI_ERROR("The configuration file doesn't include a 'driver' field."));
	}

	impl_ = std::make_unique<FRIDriver::pImpl>(sample_time_, port);
}

FRIDriver::~FRIDriver() = default;

bool FRIDriver::init(double timeout) {
	std::cout << "\n";
	int result = impl_->fri->StartRobotInJointPositionControl(timeout);
	bool ok = (result == EOK);

	if(ok) {
		read();
		ok &= Driver::init(timeout);
	}
	else {
		std::cerr << "[phri::FRIDriver::init] ERROR, could not start Kuka LWR: " << strerror(result) << std::endl;
	}

	return ok;
}

bool FRIDriver::checkConnection() const {
	return impl_->fri->IsMachineOK();
}

bool FRIDriver::start(double timeout) {
	sync();

	std::cout << "\n[phri::FRIDriver::start] Current system state:" << std::endl << impl_->fri->GetCompleteRobotStateAndInformation() << std::endl;

	return checkConnection();
}

bool FRIDriver::stop() {
	sync();

	bool ok = impl_->fri->StopRobot() == EOK;
	if(not ok) {
		std::cerr << "[phri::FRIDriver::stop] An error occurred while stopping the Kuka LWR" << std::endl;
	}

	return ok;
}

bool FRIDriver::read() {
	float buffer[12];
	bool all_ok = true;

	if(checkConnection()) {

		sync();

		if(impl_->fri->GetMeasuredJointPositions(buffer) == EOK) {
			for (size_t i = 0; i < 7; i++) {
				(*robot_->jointCurrentPosition())[i] = buffer[i];
			}
		}
		else {
			std::cerr << "[phri::FRIDriver::read] Can't get joint positions from FRI" << std::endl;
			all_ok = false;
		}

		if(impl_->fri->GetEstimatedExternalJointTorques(buffer) == EOK)
			for (size_t i = 0; i < 7; ++i) {
				(*robot_->jointExternalTorque())[i] = buffer[i];
			}
		else {
			std::cerr << "[phri::FRIDriver::read] Can't get external joint torques from FRI" << std::endl;
			all_ok = false;
		}

		if(impl_->fri->GetEstimatedExternalCartForcesAndTorques(buffer) == EOK) {
			Vector6d& ext_force = *robot_->controlPointExternalForce();

			// The array comming from FRI is [Fx, Fy, Fz, Tz, Ty, Tx]
			for (size_t i = 0; i < 6; ++i) {
				ext_force[i] = -buffer[i];  // The provided wrench is the one applied to the environment, not what it is applied to the robot
			}

			// Swapping x & z
			std::swap(ext_force[3], ext_force[5]);
		}
		else {
			std::cerr << "[phri::FRIDriver::read] Can't get force sensor values from FRI" << std::endl;
			all_ok = false;
		}
	}
	else {
		std::cerr << "[phri::FRIDriver::read] The connection to the FRI has been lost" << std::endl;
		all_ok = false;
	}

	return all_ok;
}

bool FRIDriver::send() {
	float buffer[7];

	if(checkConnection()) {
		const VectorXd& velocity_vec = *robot_->jointVelocity();
		VectorXd& position_vec = *robot_->jointTargetPosition();

		position_vec += velocity_vec*impl_->sample_time;

		// Set joint target positions
		for (size_t i = 0; i < 7; ++i) {
			buffer[i] = position_vec[i];
		}

		impl_->fri->SetCommandedJointPositions(buffer);

		return true;
	}
	else {
		std::cerr << "[phri::FRIDriver::send] The connection to the FRI has been lost" << std::endl;
		return false;
	}
}

void FRIDriver::sync() const {
	impl_->fri->WaitForKRCTick();
}
