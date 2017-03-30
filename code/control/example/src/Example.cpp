/**
 * Example - Example code.
 * Copyright (C) 2016 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdint.h>

#include <iostream>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/ExampleMessage.h"

#include "Example.h"

namespace scaledcars {
namespace control {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;

Example::Example(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "scaledcars-control-example") {}


Example::~Example() {}

void Example::setUp() {}

void Example::tearDown() {}

void Example::nextContainer(odcore::data::Container &c) {
    // Example on how to receive Containers.
    cout << " Received Container of type = " << c.getDataType() << endl;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Example::body() {
    // Example on how to use self-defined data structures.
    chalmersrevere::scaledcars::ExampleMessage em;
    cout << em.toString() << endl;
    em.setField1(1234);
    cout << em.toString() << endl;

    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        cout << "Inside the main processing loop." << endl;

        // Example how to send commands to the vehicle.
        automotive::VehicleControl vc;
        vc.setSpeed(2);
        vc.setSteeringWheelAngle(5 * cartesian::Constants::DEG2RAD);
        Container c(vc);
        getConference().send(c);
    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}
}
} // scaledcars::control
