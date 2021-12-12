#include "krembot.ino.h"


int col, row;

int ** occupancyGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;

enum State
{
    move,
    turn
} state = turn;

void walkingOnGrid_controller::setup() {
    krembot.setup();
    krembot.Led.write(0,255,0);

    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

}

void walkingOnGrid_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;

}
