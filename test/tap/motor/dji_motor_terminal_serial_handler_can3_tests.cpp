/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_terminal_serial_handler.hpp"
#include "tap/stub/terminal_device_stub.hpp"

using namespace tap::motor;
using namespace testing;
using namespace tap;

// CAN3 only exists on boards with three CAN buses (lbuild copies this file only there).
// getCan3Motor is not mocked, so these tests add motors to the real DjiMotorTxHandler store.

TEST(DjiMotorTerminalSerialHandlerCan3, terminalSerialCallback__can_3_prints_can3_motors)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m1(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS3, false, "m1");
    DjiMotor m6(&drivers, MotorId::MOTOR6, tap::can::CanBus::CAN_BUS3, false, "m6");
    drivers.djiMotorTxHandler.DjiMotorTxHandler::addMotorToManager(&m1);
    drivers.djiMotorTxHandler.DjiMotorTxHandler::addMotorToManager(&m6);

    char input[] = "can 3";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("1. m1"));
    EXPECT_THAT(output, HasSubstr("6. m6"));
}

TEST(DjiMotorTerminalSerialHandlerCan3, terminalSerialCallback__can_3_motor_6_prints_one_motor)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m1(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS3, false, "m1");
    DjiMotor m6(&drivers, MotorId::MOTOR6, tap::can::CanBus::CAN_BUS3, false, "m6");
    drivers.djiMotorTxHandler.DjiMotorTxHandler::addMotorToManager(&m1);
    drivers.djiMotorTxHandler.DjiMotorTxHandler::addMotorToManager(&m6);

    char input[] = "can 3 motor 6";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("1. m1")));
    EXPECT_THAT(output, HasSubstr("6. m6"));
}

TEST(DjiMotorTerminalSerialHandlerCan3, terminalSerialCallback__all_prints_can3)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m2(&drivers, MotorId::MOTOR2, tap::can::CanBus::CAN_BUS3, false, "can3 motor");
    drivers.djiMotorTxHandler.DjiMotorTxHandler::addMotorToManager(&m2);

    char input[] = "all";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("CAN 3"));
    EXPECT_THAT(output, HasSubstr("2. can3 motor"));
}

TEST(DjiMotorTerminalSerialHandlerCan3, terminalSerialCallback__can_4_invalid)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "can 4";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("Invalid can bus ID"));
}
