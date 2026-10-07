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
#include "tap/mock/dji_motor_mock.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"
#include "tap/test_macros.hpp"

using namespace testing;
using namespace tap;
using namespace tap::motor;
using namespace tap::mock;

// CAN3 only exists on boards with three CAN buses (lbuild copies this file only there).

class DjiMotorTxHandlerCan3Test : public Test
{
protected:
    DjiMotorTxHandlerCan3Test() : drivers(), djiMotorTxHandler(&drivers), motors()
    {
        for (size_t i = 0; i < DjiMotorTxHandler::DJI_MOTORS_PER_CAN; i++)
        {
            motors.emplace_back(new NiceMock<DjiMotorMock>(
                &drivers,
                NORMALIZED_ID_TO_DJI_MOTOR(i),
                can::CanBus::CAN_BUS3,
                false,
                "",
                0,
                0,
                i >= DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 2));

            if (i >= DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 2)
            {
                ON_CALL(*motors[i], isInCurrentControl).WillByDefault(Return(true));
            }
        }
    }

    ~DjiMotorTxHandlerCan3Test()
    {
        for (auto element : motors)
        {
            delete element;
        }
    }

    void SetUp() override
    {
        ON_CALL(drivers.can, isReadyToSend).WillByDefault(Return(true));
        ON_CALL(drivers.can, sendMessage).WillByDefault(Return(true));

        for (auto *motor : motors)
        {
            ON_CALL(*motor, getMotorIdentifier).WillByDefault([motor]() {
                return motor->DjiMotor::getMotorIdentifier();
            });

            ON_CALL(*motor, getCanBus).WillByDefault([motor]() {
                return motor->DjiMotor::getCanBus();
            });
        }
    }

    void addAllMotors()
    {
        for (auto motor : motors)
        {
            djiMotorTxHandler.addMotorToManager(motor);
        }
    }

    Drivers drivers;
    DjiMotorTxHandler djiMotorTxHandler;

    std::vector<NiceMock<DjiMotorMock> *> motors;
};

TEST_F(DjiMotorTxHandlerCan3Test, encodeAndSendCanData_all_can3_motors_3_messages_sent_on_can3)
{
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS1, _)).Times(0);
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS2, _)).Times(0);
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS3, _)).Times(3);

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerCan3Test, encodeAndSendCanData_sends_low_high_and_6020_current_ids)
{
    EXPECT_CALL(
        drivers.can,
        sendMessage(
            can::CanBus::CAN_BUS3,
            Field(&modm::can::Message::identifier, DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER)));
    EXPECT_CALL(
        drivers.can,
        sendMessage(
            can::CanBus::CAN_BUS3,
            Field(&modm::can::Message::identifier, DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER)));
    EXPECT_CALL(
        drivers.can,
        sendMessage(
            can::CanBus::CAN_BUS3,
            Field(
                &modm::can::Message::identifier,
                DjiMotorTxHandler::CAN_DJI_6020_CURRENT_IDENTIFIER)));

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerCan3Test, encodeAndSendCanData_does_not_send_if_can3_busy)
{
    ON_CALL(drivers.can, isReadyToSend(can::CanBus::CAN_BUS3)).WillByDefault(Return(false));
    EXPECT_CALL(drivers.can, sendMessage).Times(0);

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerCan3Test, getCan3Motor_returns_nullptr_when_invalid_motorid)
{
    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan3Motor(static_cast<MotorId>(motor::MOTOR8 + 1)));
    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan3Motor(static_cast<MotorId>(motor::MOTOR1 - 1)));
}

TEST_F(DjiMotorTxHandlerCan3Test, getCan3Motor_returns_nullptr_when_motor_id_not_in_motor_manager)
{
    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan3Motor(static_cast<MotorId>(motor::MOTOR1)));
}

TEST_F(DjiMotorTxHandlerCan3Test, getCan3Motor_returns_proper_motor_and_not_on_other_buses)
{
    djiMotorTxHandler.addMotorToManager(motors[0]);

    EXPECT_EQ(motors[0], djiMotorTxHandler.getCan3Motor(motor::MOTOR1));
    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan1Motor(motor::MOTOR1));
    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan2Motor(motor::MOTOR1));
}

TEST_F(DjiMotorTxHandlerCan3Test, removeFromMotorManager_removes_can3_motor)
{
    djiMotorTxHandler.addMotorToManager(motors[0]);

    djiMotorTxHandler.removeFromMotorManager(*motors[0]);

    EXPECT_EQ(nullptr, djiMotorTxHandler.getCan3Motor(motor::MOTOR1));
}

TEST_F(DjiMotorTxHandlerCan3Test, removeFromMotorManager_motor_not_added_errors)
{
    EXPECT_ERROR();

    djiMotorTxHandler.removeFromMotorManager(*motors[0]);
}
