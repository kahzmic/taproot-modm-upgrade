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

#include "tap/architecture/clock.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"

#include "modm/architecture/interface/can_message.hpp"

using namespace testing;
using namespace tap::motor::motorsim;
using namespace tap::motor;
using namespace tap::can;

// CAN3 only exists on boards with three CAN buses (lbuild copies this file only there).

class DjiMotorSimHandlerCan3Test : public Test
{
protected:
    DjiMotorSimHandler handler;
    tap::arch::clock::ClockStub clock;
};

TEST_F(DjiMotorSimHandlerCan3Test, parseMotorMessage_can3_sim_only_takes_can3_messages)
{
    modm::can::Message msgLow(
        DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER,
        8,
        0xffff'ffff'ffff'ffff,
        false);
    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS3, MOTOR1));

    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS1, msgLow));
    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS2, msgLow));
    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_TRUE(handler.parseMotorMessage(CanBus::CAN_BUS3, msgLow));
    EXPECT_NE(0, sim->getCurrent());
}

TEST_F(DjiMotorSimHandlerCan3Test, encodeMessage_can3_motor_only_on_can3)
{
    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS3, MOTOR2));
    handler.resetMotorSims();

    modm::can::Message msg(0, 8, uint64_t{0}, false);
    EXPECT_FALSE(handler.encodeMessage(CanBus::CAN_BUS1, &msg));
    EXPECT_TRUE(handler.encodeMessage(CanBus::CAN_BUS3, &msg));
    EXPECT_EQ(static_cast<uint32_t>(MOTOR2), msg.getIdentifier());
}
