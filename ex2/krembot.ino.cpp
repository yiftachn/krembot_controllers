// Shlomi Ben-Shushan 311408264
// Khen Aharon 307947515

#include "krembot.ino.h"
#include <string>
#include <random>

using namespace std;

SandTimer timer;
default_random_engine generator;
int8_t rot_speed = 0;

void ex2_controller::setup() {
    krembot.setup();
    krembot.Led.write(0,255,0);
}

void ex2_controller::loop() {
    krembot.loop();
    if (timer.finished()) {
        BumpersRes bumpers = krembot.Bumpers.read();
        bool bumped = bumpers.front == BumperState::PRESSED ||
                      bumpers.front_left == BumperState::PRESSED ||
                      bumpers.front_right == BumperState::PRESSED;
        if (!bumped) {
            krembot.Base.drive(100, 0);
            krembot.Led.write(0,255,0);
        } else {
            krembot.Base.stop();
            uniform_int_distribution<int> speed_distribution(-100,100);
            rot_speed = speed_distribution(generator);
            uniform_int_distribution<int> time_distribution(1,3000);
            millis_time_t rot_time = time_distribution(generator);
            timer.setPeriod(rot_time);
            timer.start();
            krembot.Led.write(255,0,0);
        }
    } else {
        krembot.Base.drive(0, rot_speed);
        krembot.Led.write(255,0,0);
    }
}