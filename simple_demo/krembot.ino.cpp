/*******************************************************************************
* Copyright (c) 2019, Elhay Rauper.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of Elhay Rauper nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <Krembot/controller/krembot_controller.h>

//DO NOT EDIT THIS MACRO
KREMBOT_CONTROLLER_HEADER(CONTROLLER_NAME)

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

SandTimer timer;

void setup() {
    krembot.setup();

    // turn on LEDs in green
    krembot.Led.write(0, 255, 0);
}

void loop() {
    krembot.loop();
    krembot.Base.drive(50, 0);
    timer.start(1000);
    if (timer.finished()) {
        Particle.publish("one second event", "this will be printed in terminal", PRIVATE);
        Serial.Println("this will be printed in simulation GUI");
    }

}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

//DO NOT EDIT THESE MACROS
KREMBOT_CONTROLLER_FOOTER(CONTROLLER_NAME, CONTROLLER_LABEL)

