/*      File: fri_driver.h
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


/**
 * @file fri_driver.h
 * @author Benjamin Navarro
 * @brief Definition of the FRIDriver class
 * @date May 2018
 * @ingroup FRI
 */


/** @defgroup FRI
 * Provides an easy interface to the V-REP simulator
 *
 * Usage: #include <fri_driver/fri_driver.h>
 *
 */

#pragma once

#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <utility>

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>
#include <OpenPHRI/drivers/driver.h>

namespace phri {

/** @brief Wrapper for the FRI library.
 *  @details In order to simplify the usage with OpenPHRI, only joint position control is implemented and all the necessary data is read and updated at each cycle.
 */
class FRIDriver : virtual public phri::Driver {
public:

	/**
	 * @brief Construct a driver using an IP & port. Prefix and suffix can be used to target a specific robot.
	 * @param robot The robot to read/write data from/to.
	 * @param sample_time The sample time to set for the KRC.
	 * @param port The UDP port to use to connect with the KRC. Check your KRC configuration to find it.
	 */
	FRIDriver(
		phri::RobotPtr robot,
		double sample_time,
		int port);

	FRIDriver(
		const phri::RobotPtr& robot,
		const YAML::Node& configuration);

	virtual ~FRIDriver();

	/**
	 * Initialize the communication with V-REP
	 * @param timeout The maximum time to wait to establish the connection.
	 * @return true on success, false otherwise
	 */
	virtual bool init(double timeout = 30.) override;
	/**
	 * @brief Check the state of the connection.
	 * @return True if the connection is still open, false otherwise.
	 */
	bool checkConnection() const;


	/**
	 * @brief Start the simulation.
	 */
	virtual bool start() override;

	/**
	 * @brief Stop the simulation.
	 */
	virtual bool stop() override;

	virtual bool read() override;
	virtual bool send() override;

	void sync() const;

private:
	static bool registered_in_factory;

	struct pImpl;
	std::unique_ptr<pImpl> impl_;
};

using FRIDriverPtr = std::shared_ptr<FRIDriver>;
using FRIDriverConstPtr = std::shared_ptr<const FRIDriver>;

}
