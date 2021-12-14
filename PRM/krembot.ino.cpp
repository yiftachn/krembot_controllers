#include "krembot.ino.h"
#include <string>
#include <random>
#include <time.h>
#include <math.h>
#include <iostream>
#include "kdtree.hpp"
#include "Graph.h"
#include "SimpleEdge.h"
#include "Graph.h"

#define NUMBER_OF_VERTICES 1000

using namespace std;
using namespace Kdtree;

void saveGridToFile(string name, int **grid, int _height, int _width);
void clean_logger(string name);
void log_to_file(string name,string msg);
void print_nodes(const Kdtree::KdNodeVector &nodes);
void log_adj_mat_to_file(double** mat);
int **lowerGridResolution(int **grid, int reduction_factor, int grid_height, int grid_width);

vector<double*> sample_n_free_points(int n_points, int max_height, int max_width, int **grid,
                          void (*sampling_func)(int &, int &, int, int));
KdNodeVector insert_points_to_nodes(vector<double*> points);
double ** calculate_adj_matrix(KdTree& kd_tree,int**&grid,double (*distance_metric)(vector<double> ,vector<double> ,int**));
double l1_distance(vector<double> source, vector<double> dst, int**grid);
double l2_distance(vector<double> source, vector<double> dst, int**grid);
double l0_distance(vector<double> source, vector<double> dst, int**grid);

void uniform_sampling(int &row, int &col, int max_heigth, int max_width);
bool obstacle_in_the_middle(double source, double dst, int** grid);
bool is_cell_empty(int row, int col, int **grid);
string node_to_string(KdNode node);
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
    clean_logger("logger.txt");
    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;
    saveGridToFile("grid", occupancyGrid, height, width);
    int reduction_factor = 5;
    int **new_grid = lowerGridResolution(occupancyGrid, reduction_factor, height, width);
    vector<double*> vec = sample_n_free_points( NUMBER_OF_VERTICES, 30, 40, new_grid, *uniform_sampling);
    KdNodeVector nodes = insert_points_to_nodes(vec);
    Kdtree::KdTree kd_tree(&nodes);
//    print_nodes(kd_tree.allnodes);
    double ** adj_matrix = calculate_adj_matrix(kd_tree,new_grid,l2_distance);
    log_to_file("logger.txt","just after assingment");
    log_to_file("logger.txt","src: " + node_to_string(kd_tree.allnodes[500])+ " dst: " + node_to_string(kd_tree.allnodes[78]) + " adj_mat_dist1 "+
            to_string(adj_matrix[kd_tree.allnodes[500]._id][kd_tree.allnodes[78]._id]) + " adj_mat_dist2 "+
                                                                                       to_string(adj_matrix[kd_tree.allnodes[78]._id][kd_tree.allnodes[500]._id]));
    log_adj_mat_to_file(adj_matrix);


    saveGridToFile("new_grid", new_grid, height / reduction_factor, width / reduction_factor);
    log_to_file("logger.txt","just after saving grid to file");

}

void PRM_controller::loop() {
    krembot.loop();
    float distance = krembot.RgbaFront.readRGBA().Distance;
    if (distance < 15) {

        krembot.Base.stop();
        krembot.Led.write(255, 0, 0);
    } else {
//        Serial.Println(to_string(height));
//        LOG << PRM_controller::posMsg.pos;
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
string node_to_string(KdNode node){
    return "id: " + to_string(node._id) + "point: (" + to_string(node.point[0]) +','+ to_string(node.point[1])+")\n";
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

void clean_logger(string name){
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    m_cOutput<<endl;
    m_cOutput.close();
}
void log_to_file(string name,string msg){
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::app | ios_base::out);
    m_cOutput << msg << endl;
    m_cOutput.close();
}

void log_adj_mat_to_file(double** mat){
    ofstream m_cOutput;
    m_cOutput.open("adj_mat.txt", ios_base::trunc | ios_base::out);
    for (int i=0;i<NUMBER_OF_VERTICES;i++){
        for (int j=0;j<NUMBER_OF_VERTICES;j++){
            m_cOutput << mat[i][j] <<' ';
        }
        m_cOutput <<'\n';
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

vector<double*> sample_n_free_points(int n_points, int max_height, int max_width, int **grid,
                          void (*sampling_func)(int &, int &, int, int)) {
    vector<double*> vec;
    for (int i = 0; i < n_points; i++) {
        int col;
        int row;
        sampling_func(row, col, max_height, max_width);
        if (is_cell_empty(row, col, grid)) {
            while (!is_cell_empty(row, col, grid)) {
                sampling_func(row, col, max_height, max_width);
            }
        }

        auto point = new double[2];
        point[0] = row;
        point[1] = col;
        vec.push_back(point);
    }
    return vec;
}

KdNodeVector insert_points_to_nodes(vector<double*> points){
    KdNodeVector vec;
    for (auto it = points.begin();it!= points.end();it++){
        vector<double> point(2);
        point[0] = it[0][0];
        point[1] = it[0][1];
        vec.push_back(KdNode(point));
    }
    return vec;
}

double** calculate_adj_matrix(KdTree& kd_tree,int**&grid,double (*distance_metric)(vector<double> ,vector<double> ,int**)){
    KdNodeVector points_list = kd_tree.allnodes;
    double** adj_matrix = new double*[points_list.size()];
    for (int i=0;i<points_list.size();i++){
        adj_matrix[i] = new double[points_list.size()];
        KdNodeVector results;
        kd_tree.k_nearest_neighbors(points_list.at(i).point,1500,&results,NULL);
//        log_to_file("logger.txt","AFter knn");
        for (auto neighbour_it = results.begin();neighbour_it!=results.end();neighbour_it++){
            if (double distance = distance_metric(points_list.at(i).point,neighbour_it[0].point,grid)){
//                log_to_file("logger.txt","AFter double assigment");
//                log_to_file("logger.txt", "distance is"+to_string(distance));
//                log_to_file("logger.txt", "i is"+to_string(i));
//                log_to_file("logger.txt", "_id is"+to_string(neighbour_it->_id));

                adj_matrix[i][neighbour_it->_id] = distance;
//                adj_matrix[neighbour_it->_id][i] = distance;
//                log_to_file("logger.txt","AFter distance assigment");

            }
        }
    }
//    log_to_file("logger.txt","Just before return");
    return adj_matrix;
}

double l2_distance(vector<double> source, vector<double> dst, int**grid){
    if (obstacle_in_the_middle(source, dst, grid)){
        return 0;
    }
    return sqrt(pow((source[0]-dst[0]),2) + pow(source[1]-dst[1],2));
}
double l0_distance(vector<double> source, vector<double> dst, int**grid){
    return 2.5;
}

bool is_cell_empty(int row, int col, int **grid) {
    return grid[row][col] == 0;
}


void print_nodes(const Kdtree::KdNodeVector &nodes) {
    size_t i, j;
    cout << "\nstarted printing nodes\n with size " << nodes.size() << '\n';
    for (i = 0; i < nodes.size(); ++i) {
        if (i > 0)
        cout << "(";
        for (j = 0; j < nodes[i].point.size(); j++) {
            if (j > 0)
                cout << ",";
            cout << nodes[i].point[j];
        }
        cout << ")";
    }
    cout << '\n';
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


