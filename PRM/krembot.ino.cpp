#include "krembot.ino.h"
#include <string>
using namespace std;
void saveGridToFile(string name,int **grid,int _height, int _width);
int ** lowerGridResolution(int** grid,int reduction_factor,int height, int width);
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

void PRM_controller::setup() {
    krembot.setup();
    krembot.Led.write(0,255,0);

    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;
    saveGridToFile("grid",occupancyGrid,height,width);
    int** new_grid = lowerGridResolution(occupancyGrid,2,height,width);
    saveGridToFile("new_grid",new_grid,height/2,width/2);

}

void PRM_controller::loop() {
    krembot.loop();
    float distance = krembot.RgbaFront.readRGBA().Distance;
    if (distance < 15)
    {

        krembot.Base.stop();
        krembot.Led.write(255,0,0);
    } else
    {
//        Serial.Println(to_string(height));
//        Serial.Println(to_string(width));
        krembot.Base.drive(100,0);
    }

}

int ** lowerGridResolution(int** grid,int reduction_factor,int height, int width){
    int new_height = height/reduction_factor;
    int new_width = width/reduction_factor;
    int** lower_res_grid = 0;
    lower_res_grid = new int*[height];

    for(int row=height-1; row>=0; row--){
        lower_res_grid[row/reduction_factor] = new int[width/reduction_factor];
        for(int col=0; col<width; col++){
            lower_res_grid[row/reduction_factor][col/reduction_factor] = grid[row][col];
        }
    }
    return lower_res_grid;

}

void saveGridToFile(string name,int **grid,int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name,ios_base::trunc | ios_base::out);
    for(int row=_height-1; row>=0; row--){
        for(int col=0; col<_width; col++){
            m_cOutput<<grid[row][col];
        }
        m_cOutput<<endl;
    }
    m_cOutput.close();
}