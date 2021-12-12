#include "krembot.ino.h"
#include <string>
#include <random>
#include "graph.h"
#include "Point.h"
#include "KDTree.h"
using namespace std;

void saveGridToFile(string name, int **grid, int _height, int _width);

int **lowerGridResolution(int **grid, int reduction_factor, int grid_height, int grid_width);
vector<Point<2>> *sample_n_free_points(int n_points, int max_height, int max_width, int** grid,
                                       void (*sampling_func)(int &, int &, int, int));
void uniform_sampling(int &row, int &col, int max_heigth, int max_width);
bool is_cell_empty(int row, int col, int **grid);
int col, row;

int **occupancyGrid;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;

enum State {
    move,
    turn
} state = turn;

void PRM_controller::setup() {
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;
    saveGridToFile("grid", occupancyGrid, height, width);
    int reduction_factor = 5;
    int **new_grid = lowerGridResolution(occupancyGrid, reduction_factor, height, width);
    vector<Point<2>> waypoints = *sample_n_free_points(1000,30,40,new_grid,*uniform_sampling);
    KDTree<2,int> kd = KDTree<2,int>();
//    for (auto it = waypoints.begin();it != waypoints.end();++it){
//        kd.insert(*it);
//        cout << '(' << to_string(it[0][0]) << ',' << to_string(it[0][1]) <<  ')'  <<'\n';
//    }
    int a;
    Point<2> p= Point<2>(waypoints[1][0],waypoints[1][1]);
    a = kd.kNNValue(p,5000);
    cout << "knn value os " << a;
    auto b = kd.kNNNeighbours(Point<2>(waypoints[3][2],waypoints[0][1]),5);
    cout << "bpq size is " << b->size()<<'\n';
    cout << "waypoint1 is " << to_string(waypoints[0][0]) << to_string(waypoints[0][1]) << '\n';
    while (!(b->empty())){
        cout << to_string(b->dequeueMin());
    }
    cout<<'\n';

//    Graph *graph= new Graph();
//    for (pointVec::iterator it = waypoints.begin() ; it != waypoints.end(); ++it){
//        string vertex_name= Graph::vertex_name_from_ints(it[0][0], it[0][1]);
//        graph->addvertex(vertex_name);
//    }
//    auto res2 = waypoints_kd_tree.neighborhood_points(vector<double>{1,2},5.5);
//    for (point_t a : res2) {
//        for (double b : a) {
//            std::cout << b << " ";
//        }
//        std::cout << '\n';
//    }

//    vector<int>* vector_array = sample_n_free_points(1000,30,40,*uniform_sampling);
//    for (int i=0; i< 1000;i++){
//        std::cout << '('<< vector_array[i][0] << ',' << vector_array[i][1]<<")\n";
//
//    }
    saveGridToFile("new_grid", new_grid, height / reduction_factor, width / reduction_factor);

}

void PRM_controller::loop() {
    krembot.loop();
    float distance = krembot.RgbaFront.readRGBA().Distance;
    if (distance < 15) {

        krembot.Base.stop();
        krembot.Led.write(255, 0, 0);
    } else {
//        Serial.Println(to_string(height));
        LOG << PRM_controller::posMsg.pos;
        Serial.Println("");
        krembot.Base.drive(100, 0);
    }

}

int **lowerGridResolution(int **grid, int reduction_factor, int grid_height, int grid_width) {
    assert(grid_height % reduction_factor == 0);

    int new_height = grid_height / reduction_factor;
    int new_width = grid_width / reduction_factor;
    int **new_grid = new int *[new_height];


    for (int col = 0; col < new_height; col++) {
        new_grid[col] = new int[new_width];
    }

    for (int row = 0; row < width; row++) {
        for (int col = 0; col < height; col++) {
            new_grid[row / reduction_factor][col / reduction_factor] = max(0, grid[row][col]);
        }
    }
    return new_grid;

//    lower_res_grid = new int*[grid_height];
//
//    for(int row=grid_height-1; row>=0; row--){
//        lower_res_grid[row/reduction_factor] = new int[grid_width / reduction_factor];
//        for(int col=0; col < grid_width; col++){
//            lower_res_grid[row/reduction_factor][col/reduction_factor] = grid[row][col];
//        }
//    }
//    return lower_res_grid;

}

void saveGridToFile(string name, int **grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            m_cOutput << grid[row][col];
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

CVector2 PRM_controller::cell_to_cord(int cell_i, int cell_j, int reduction_factor) {
//    now it points to the origin of the cell, better imporve it to the center of the cell
    Real x_cord = origin.GetX() + reduction_factor * resolution * cell_j;
    Real y_cord = origin.GetY() + reduction_factor * resolution * cell_i;
    CVector2 cord(x_cord, y_cord);
    return cord;
}

void uniform_sampling(int &row, int &col, int max_heigth, int max_width) {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> height_dis(0, max_heigth);
    std::uniform_real_distribution<> weight_dis(0, max_width);

    row = (int) height_dis(gen);
    col = (int) weight_dis(gen);

}

vector<Point<2>> *sample_n_free_points(int n_points, int max_height, int max_width, int** grid,
                                                       void (*sampling_func)(int &, int &, int, int)) {
    auto points = new vector<Point<2>>;
    for (int i = 0; i < n_points; i++) {
        int col;
        int row;
        sampling_func(row, col, max_height, max_width);
        if (is_cell_empty(row,col,grid)) {
            while (!is_cell_empty(row, col, grid)) {
                sampling_func(row, col, max_height, max_width);
            }
        }
        auto new_point = new Point<2>(row,col);
        points->push_back(new_point[0]);
    }
    return points;
}

bool is_cell_empty(int row, int col, int **grid) {
    return grid[row][col] == 0;
}


//void PRM_controller::create_prm() {
//    sample_points()
//    insert
//    point
//    to
//    KD - tree
//    Add edges
//
//    waypoints_kd_tree = KDTree()
//    while number_of_free_Sampled_point < N:
//    point = sample
//    point;
//    if point
//    is
//    free:
//    add
//    to
//    waypoints_kd - tree;
//    for
//    each
//    waypoint
//    in
//    point_neiberhood:
//    add edge(waypoint, point)
//    AFTERWARDS
//    CHECK
//    IF
//    CONNECTED COMPONENTS;
//}

void PRM_controller::pos_to_cord(CVector2 position, int *i, int *j) {
    *i = (position.GetX() - origin.GetX()) / resolution;
    *j = (position.GetY() - origin.GetY()) / resolution;
}


void PRM_controller::pos_to_cell(CVector2 position, int *i, int *j, int size) {
    int pixel_i, pixel_j;
    pos_to_cord(position, &pixel_i, &pixel_j);
    *i = (int) ((pixel_i + 0.5 * size) / size) - 1;
    *j = (int) ((pixel_j + 0.5 * size) / size) - 1;
}


