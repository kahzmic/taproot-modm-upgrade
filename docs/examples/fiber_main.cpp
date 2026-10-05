/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * Example layout of a Taproot program on modm fibers. Copy it into your project as `main.cpp`.
 * It is not built here: Taproot CI builds test-project/ with only some modules enabled.
 *
 * Fibers are cooperative: a fiber runs until it calls `modm::this_fiber::yield()` (directly, or
 * inside a blocking Taproot/modm call such as an SPI transfer or
 * `RefSerialTransmitter::sendGraphic`) and only then does the next fiber run. A fiber that loops
 * without yielding stops every other fiber, including the one that sends motor outputs.
 *
 * - Taproot's driver fibers (`Drivers::startFibers()`): CAN receive, remote, referee serial and
 *   IMU each poll their hardware in their own fiber.
 * - `control`: runs the command scheduler and sends motor outputs at a fixed rate.
 * - `aux`: slow, blocking work (referee UI, OLED) that must never delay `control`.
 */

#include "tap/architecture/periodic_timer.hpp"
#include "tap/drivers.hpp"

#include "modm/processing/fiber.hpp"

namespace
{
class Drivers : public tap::Drivers
{
public:
    Drivers() : tap::Drivers() {}
};

Drivers drivers;

constexpr float CONTROL_FREQUENCY = 500.0f;

#ifdef PLATFORM_HOSTED
// Hosted fibers need room for the C++ standard library, e.g. iostream.
constexpr size_t STACK_SIZE = modm::fiber::StackSizeDefault;
#else
// Not measured yet; see the D1 notes on stack sizing.
constexpr size_t STACK_SIZE = 2048;
#endif

auto &imu()
{
#ifdef TAPROOT_MPU6500_HPP_
    return drivers.mpu6500;
#else
    return drivers.bmi088;
#endif
}

modm::Fiber<STACK_SIZE> control(
    [] {
        tap::arch::PeriodicMilliTimer controlTimer(1000.0f / CONTROL_FREQUENCY);
        while (true)
        {
            // Waits on Taproot's timer by yielding, so the driver fibers run in between.
            modm::this_fiber::poll([&] { return controlTimer.execute(); });

            imu().periodicIMUUpdate();
            drivers.commandScheduler.run();
            drivers.djiMotorTxHandler.encodeAndSendCanData();
        }
    },
    modm::fiber::Start::Later);

modm::Fiber<STACK_SIZE> aux(
    [] {
        tap::arch::PeriodicMilliTimer auxTimer(250);
        while (true)
        {
            // Stand-in for slow work such as `RefSerialTransmitter::sendGraphic`, which blocks
            // (yielding) for tens of milliseconds per message.
            modm::this_fiber::poll([&] { return auxTimer.execute(); });
        }
    },
    modm::fiber::Start::Later);

void initializeIo()
{
    drivers.analog.init();
    drivers.pwm.init();
    drivers.digital.init();
    drivers.leds.init();
    drivers.can.initialize();
    drivers.errorController.init();
    drivers.remote.initialize();
    imu().initialize(CONTROL_FREQUENCY, 0.1f, 0.0f);
    drivers.refSerial.initialize();
}
}  // namespace

int main()
{
    Board::initialize();
    initializeIo();

    // Start the fibers only after initialization, then hand the CPU to the scheduler.
    drivers.startFibers();
    control.start();
    aux.start();

    modm::fiber::Scheduler::run();
    return 0;
}
