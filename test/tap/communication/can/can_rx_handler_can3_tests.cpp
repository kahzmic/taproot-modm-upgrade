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

#include <memory>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/can_rx_handler_mock.hpp"
#include "tap/mock/can_rx_listener_mock.hpp"
#include "tap/test_macros.hpp"

using namespace testing;
using namespace tap::mock;
using tap::can::CanBus;
using tap::can::CanRxHandler;

// CAN3 only exists on boards with three CAN buses (lbuild copies this file only there).

class CanRxHandlerCan3Test : public Test
{
protected:
    CanRxHandlerCan3Test() : handler(&drivers) {}

    tap::Drivers drivers;
    CanRxHandler handler;
};

TEST_F(CanRxHandlerCan3Test, attachReceiveHandler_attaches_listener_to_can3_store_only)
{
    CanRxListenerMock listener(&drivers, 1, CanBus::CAN_BUS3);
    int bin = CanRxHandler::binIndexForCanId(listener.canIdentifier);

    handler.attachReceiveHandler(&listener);

    EXPECT_EQ(&listener, handler.getHandlerStore(CanBus::CAN_BUS3)[bin]);
    EXPECT_EQ(nullptr, handler.getHandlerStore(CanBus::CAN_BUS1)[bin]);
    EXPECT_EQ(nullptr, handler.getHandlerStore(CanBus::CAN_BUS2)[bin]);

    handler.removeReceiveHandler(listener);

    EXPECT_EQ(nullptr, handler.getHandlerStore(CanBus::CAN_BUS3)[bin]);
}

TEST_F(CanRxHandlerCan3Test, same_id_on_each_bus_goes_to_separate_stores)
{
    CanRxListenerMock listener1(&drivers, 1, CanBus::CAN_BUS1);
    CanRxListenerMock listener2(&drivers, 1, CanBus::CAN_BUS2);
    CanRxListenerMock listener3(&drivers, 1, CanBus::CAN_BUS3);
    int bin = CanRxHandler::binIndexForCanId(1);

    EXPECT_ERROR_TIMES(0);

    handler.attachReceiveHandler(&listener1);
    handler.attachReceiveHandler(&listener2);
    handler.attachReceiveHandler(&listener3);

    EXPECT_EQ(&listener1, handler.getHandlerStore(CanBus::CAN_BUS1)[bin]);
    EXPECT_EQ(&listener2, handler.getHandlerStore(CanBus::CAN_BUS2)[bin]);
    EXPECT_EQ(&listener3, handler.getHandlerStore(CanBus::CAN_BUS3)[bin]);

    handler.removeReceiveHandler(listener1);
    handler.removeReceiveHandler(listener2);
    handler.removeReceiveHandler(listener3);
}

TEST_F(CanRxHandlerCan3Test, pollCanData_routes_can3_message_to_can3_listener)
{
    CanRxListenerMock listener1(&drivers, 1, CanBus::CAN_BUS1);
    CanRxListenerMock listener3(&drivers, 1, CanBus::CAN_BUS3);
    handler.attachReceiveHandler(&listener1);
    handler.attachReceiveHandler(&listener3);

    const modm::can::Message rxMessage(1);
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS1, _)).WillOnce(Return(false));
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS2, _)).WillOnce(Return(false));
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS3, _))
        .WillOnce(DoAll(SetArgPointee<1>(rxMessage), Return(true)));

    EXPECT_CALL(listener1, processMessage).Times(0);
    EXPECT_CALL(listener3, processMessage);

    handler.pollCanData();

    handler.removeReceiveHandler(listener1);
    handler.removeReceiveHandler(listener3);
}

TEST_F(CanRxHandlerCan3Test, pollCanData_does_not_route_can1_message_to_can3_listener)
{
    CanRxListenerMock listener3(&drivers, 1, CanBus::CAN_BUS3);
    handler.attachReceiveHandler(&listener3);

    const modm::can::Message rxMessage(1);
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS1, _))
        .WillOnce(DoAll(SetArgPointee<1>(rxMessage), Return(true)));
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS2, _)).WillOnce(Return(false));
    EXPECT_CALL(drivers.can, getMessage(CanBus::CAN_BUS3, _)).WillOnce(Return(false));

    EXPECT_CALL(listener3, processMessage).Times(0);

    handler.pollCanData();

    handler.removeReceiveHandler(listener3);
}

TEST_F(CanRxHandlerCan3Test, removeReceiveHandler_error_logged_when_listener_not_on_can3)
{
    CanRxListenerMock listener1(&drivers, 1, CanBus::CAN_BUS1);
    CanRxListenerMock listener3(&drivers, 1, CanBus::CAN_BUS3);
    handler.attachReceiveHandler(&listener1);

    EXPECT_ERROR();

    handler.removeReceiveHandler(listener3);

    handler.removeReceiveHandler(listener1);
}
