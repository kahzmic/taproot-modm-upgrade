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
 * `RefSerialTransmitter::sendGraphic`) and only then does the next fiber run. A fiber that runs
 * for a long time without yielding delays every other fiber, including the control loop.
 *
 * - `mainLoop`: the old superloop, moved into a fiber. Every pass polls the non-blocking IO
 *   (`updateIo()`); every `CONTROL_FREQUENCY` tick it reads the IMU, runs the command scheduler
 *   and sends motor outputs. Code here may block briefly, as long as the loop still keeps its
 *   rate. The `yield()` at the end of each pass is what lets the other fibers run.
 * - `blockingIo`: work that blocks for a long time (referee UI, OLED). Give such work its own
 *   fiber so it never holds up `mainLoop`.
 *
 * Commands and subsystems run inside `commandScheduler.run()`, so they must not block for long
 * either. A command that used to run a protothread from `execute()` (e.g. a referee UI command)
 * should start a fiber instead of calling `run()`.
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

/// Non-blocking polls: each handles whatever data is ready and returns.
void updateIo()
{
    drivers.canRxHandler.pollCanData();
    drivers.remote.read();
    drivers.refSerial.updateSerial();
}

modm::Fiber<STACK_SIZE> mainLoop(
    [] {
        tap::arch::PeriodicMilliTimer controlTimer(1000.0f / CONTROL_FREQUENCY);
        while (true)
        {
            updateIo();

            if (controlTimer.execute())
            {
                imu().read();
                imu().periodicIMUUpdate();
                drivers.commandScheduler.run();
                drivers.djiMotorTxHandler.encodeAndSendCanData();
            }

            modm::this_fiber::yield();
        }
    },
    modm::fiber::Start::Later);

modm::Fiber<STACK_SIZE> blockingIo(
    [] {
        tap::arch::PeriodicMilliTimer uiTimer(250);
        while (true)
        {
            // Stand-in for slow work such as `RefSerialTransmitter::sendGraphic`, which blocks
            // (yielding) for tens of milliseconds per message.
            modm::this_fiber::poll([&] { return uiTimer.execute(); });
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
    mainLoop.start();
    blockingIo.start();

    modm::fiber::Scheduler::run();
    return 0;
}
