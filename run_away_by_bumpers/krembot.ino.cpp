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


/*
 * This file simulates the .ino file of the krembot.
 * The original structure of this file must be maintained
 * for the simulator to work correctly.
 * Don't create Krembot instance, it is created automatically.
 * Copy and past real robot setup() and loop() code into the functions below
 */

#include <Krembot/controller/krembot_controller.h>

//DO NOT EDIT THIS MACRO
KREMBOT_CONTROLLER_HEADER(CONTROLLER_NAME)

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

    SandTimer backTimer;
    SandTimer turnTimer;

    BumpersRes lastResults;

    const int FORWARD_SPEED = 60;
    const int BACKWARD_SPEED = -60;


    const int LEFT_SPEED = 75;
    const int RIGHT_SPEED = -75;

    const int BACK_INTERVAL = 450;
    const int TURN_INTERVAL = 800;

    bool drivingForward = false;
    bool drivingBack = false;
    bool turning = false;


    void setup() {
        krembot.setup();

        // start driving forward
        krembot.Base.drive(FORWARD_SPEED, 0);
        backTimer.start(BACK_INTERVAL);
        turnTimer.start(TURN_INTERVAL);
    }

    void loop() {
        krembot.loop();

        // read bumpre states
        BumpersRes results = krembot.Bumpers.read();
        if (results.isAnyPressed())
        {
            // if one of the bumpers was pressed check which one was pressed. based on the
            // bumper that was pressed. if it was one of the front bumpers - switch to
            // drivingBack mode for 450 miliseconds
            if(results.front == BumperState::PRESSED || results.front_left == BumperState::PRESSED || results.front_right == BumperState::PRESSED)
            {
                krembot.Base.drive(BACKWARD_SPEED, 0);
                backTimer.startOver();
                drivingBack = true;
                turning = false;
                krembot.Led.write(255,0,0);

                Serial.Print("---"); Serial.Print(krembot.getName()); Serial.Print("---");
                krembot.Bumpers.print();
                Serial.Println("------");

            }
                //if the pressed bumper is not one of the front ones
                // switch to turn mode for 450 miliseconds
            else
            {
                drivingBack = false;
                turning = true;
                turnTimer.startOver();
            }

            lastResults = results;

        }


        //if the finished turning disable turning mode
        if(turnTimer.finished() && turning)
        {
            turning = false;
            drivingBack = false;
            krembot.Led.write(0,255,0);

        }

        // if finished driving back, switch to turning mode
        if(backTimer.finished() && drivingBack)
        {
            drivingBack = false;
            turning = true;
            turnTimer.startOver();
            krembot.Led.write(0,0,255);

        }


        //decide the turning direction based on the pressed BUMPER_REAR_LEFT
        // if it was on the right side (or the front bumper) turn left.
        // if it was one of the left side - turn right
        if(turning)
        {
            krembot.Led.write(255,255,255);

            if(lastResults.front == BumperState::PRESSED)
            {
                // we chose to turn left
                krembot.Base.drive(0, LEFT_SPEED);
            }
            else if(lastResults.front_right == BumperState::PRESSED)
            {
                // we chose to turn left
                krembot.Base.drive(0, LEFT_SPEED);
            }
            else if(lastResults.right == BumperState::PRESSED)
            {
                krembot.Base.drive(0, LEFT_SPEED);
            }
            else if(lastResults.rear_right == BumperState::PRESSED)
            {
                krembot.Base.drive(0, LEFT_SPEED);
            }
            else if(lastResults.rear == BumperState::PRESSED)
            {
                krembot.Base.drive(FORWARD_SPEED, 0);
            }
            else if(lastResults.rear_left == BumperState::PRESSED)
            {
                krembot.Base.drive(0, RIGHT_SPEED);
            }
            else if(lastResults.left == BumperState::PRESSED)
            {
                krembot.Base.drive(0, RIGHT_SPEED);
            }
            else if(lastResults.front_left == BumperState::PRESSED)
            {
                krembot.Base.drive(0, RIGHT_SPEED);
            }

        }

        //if not turning and not driving - drive forward
        if(!turning && !drivingBack)
        {
            krembot.Base.drive(FORWARD_SPEED, 0);
            krembot.Led.write(0,0,0);
        }
    }


////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

//DO NOT EDIT THESE MACROS
KREMBOT_CONTROLLER_FOOTER(CONTROLLER_NAME, CONTROLLER_LABEL)



