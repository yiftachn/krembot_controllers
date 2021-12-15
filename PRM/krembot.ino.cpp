#include "krembot.ino.h"
#include <string>
#include <random>
#include <time.h>
#include <math.h>
#include <iostream>
#include "kdtree.hpp"
#include <stack>

#define NUMBER_OF_VERTICES 1000

using namespace std;
using namespace Kdtree;

void saveGridToFile(string name, int **grid, int _height, int _width);

void clean_logger(string name);

void log_to_file(string name, string msg);

void print_nodes(const Kdtree::KdNodeVector &nodes);

void log_adj_mat_to_file(double **mat);

int **lowerGridResolution(int **grid, int reduction_factor, int grid_height, int grid_width);

vector<double *> sample_n_free_points(int n_points, int max_height, int max_width, int **grid,
                                      void (*sampling_func)(int &, int &, int, int));

KdNodeVector insert_points_to_nodes(vector<double *> points);

double **
calculate_adj_matrix(KdTree &kd_tree, int **&grid, double (*distance_metric)(vector<double>, vector<double>, int **));

double l1_distance(vector<double> source, vector<double> dst, int **grid);

double l2_distance(vector<double> source, vector<double> dst, int **grid);

double l0_distance(vector<double> source, vector<double> dst, int **grid);

void uniform_sampling(int &row, int &col, int max_heigth, int max_width);

bool obstacle_in_the_middle(vector<double> source, vector<double> dst, int **grid);

bool is_cell_empty(int row, int col, int **grid);

stack<int> _find_shortest_path_from_two_nodes(KdNode src,KdNode dst, double** adj_matrix);
int find_index_of_unvisited_min_index(double* distances,int* unvisited);

string node_to_string(KdNode node);

vector<vector<double>> find_shortest_path(vector<double> src, vector<double> dst,double** adj_matrix,KdTree& kdTree);

vector<int> get_neighbours_of_node(int node_index,double **adj_matrix);
vector<vector<double>> convert_path_from_id_to_coords(stack<int> nodes_id_path,KdTree& kd_tree);
string point_to_string(vector<double> point);
void print_path_to_file(vector<vector<double>>path,int** grid);
void print_path_as_list(vector<vector<double>>path);
void print_nodes_in_grid(KdNodeVector nodes,int** grid);
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
    vector<double> starting_pos;
    vector<double> goal_pos;
    starting_pos.push_back(1);
    starting_pos.push_back(1);
    goal_pos.push_back(-2);
    goal_pos.push_back(-2);
    int reduction_factor = 5;
    vector<int> starting_cell = PRM_controller::coord_to_cell(starting_pos,resolution*reduction_factor);
    vector<int> goal_cell = PRM_controller::coord_to_cell(goal_pos,resolution*reduction_factor);
    std::vector<double> starting_cell_double(starting_cell.begin(), starting_cell.end());
    std::vector<double> goal_cell_double(goal_cell.begin(), goal_cell.end());

    saveGridToFile("grid", occupancyGrid, height, width);
    int **new_grid = lowerGridResolution(occupancyGrid, reduction_factor, height, width);
    vector<double *> vec = sample_n_free_points(NUMBER_OF_VERTICES, 50, 50, new_grid, *uniform_sampling);
    KdNodeVector nodes = insert_points_to_nodes(vec);
    Kdtree::KdTree kd_tree(&nodes);
    print_nodes_in_grid(kd_tree.allnodes,new_grid);

//    print_nodes(kd_tree.allnodes);
    double **adj_matrix = calculate_adj_matrix(kd_tree, new_grid, l2_distance);
    log_to_file("logger.txt", "just after adj_mat calc");
    log_to_file("logger.txt",
                "src: " + node_to_string(kd_tree.allnodes[500]) + " dst: " + node_to_string(kd_tree.allnodes[78]) +
                " adj_mat_dist1 " +
                to_string(adj_matrix[kd_tree.allnodes[500]._id][kd_tree.allnodes[78]._id]) + " adj_mat_dist2 " +
                to_string(adj_matrix[kd_tree.allnodes[78]._id][kd_tree.allnodes[500]._id]));
    log_adj_mat_to_file(adj_matrix);
    vector<vector<double>> path = find_shortest_path(starting_cell_double,goal_cell_double,adj_matrix,kd_tree);
    log_to_file("logger.txt", "just after PATH!!");
    print_path_to_file(path,new_grid);
    log_to_file("logger.txt", "Printed PATH!!");
    print_path_as_list(path);
    saveGridToFile("new_grid", new_grid, height / reduction_factor, width / reduction_factor);
    log_to_file("logger.txt", "just after saving grid to file");

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
PRM_controller::posMsg.degreeX;
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

string node_to_string(KdNode node) {
    return "id: " + to_string(node._id) + "point: (" + to_string(node.point[0]) + ',' + to_string(node.point[1]) +
           ")\n";
}
string point_to_string(vector<double> point){
    return "(" + to_string(point[0]) + ',' + to_string(point[1]) +")";
}

void saveGridToFile(string name, int **grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            m_cOutput << grid[row][col] <<',';
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

void clean_logger(string name) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    m_cOutput << endl;
    m_cOutput.close();
}

void log_to_file(string name, string msg) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::app | ios_base::out);
    m_cOutput << msg << endl;
    m_cOutput.close();
}

void log_adj_mat_to_file(double **mat) {
    ofstream m_cOutput;
    m_cOutput.open("adj_mat.txt", ios_base::trunc | ios_base::out);
    for (int i = 0; i < NUMBER_OF_VERTICES; i++) {
        for (int j = 0; j < NUMBER_OF_VERTICES; j++) {
            m_cOutput << mat[i][j] << ' ';
        }
        m_cOutput << '\n';
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

vector<double *> sample_n_free_points(int n_points, int max_height, int max_width, int **grid,
                                      void (*sampling_func)(int &, int &, int, int)) {
    vector<double *> vec;
    for (int i = 0; i < n_points; i++) {
        int col;
        int row;
        sampling_func(row, col, max_height, max_width);
        while (!is_cell_empty(row, col, grid)) {
            sampling_func(row, col, max_height, max_width);
        }


        auto point = new double[2];
        point[0] = row;
        point[1] = col;
        vec.push_back(point);
    }
    return vec;
}

KdNodeVector insert_points_to_nodes(vector<double *> points) {
    KdNodeVector vec;
    for (auto it = points.begin(); it != points.end(); it++) {
        vector<double> point(2);
        point[0] = it[0][0];
        point[1] = it[0][1];
        vec.push_back(KdNode(point));
    }
    return vec;
}

double **
calculate_adj_matrix(KdTree &kd_tree, int **&grid, double (*distance_metric)(vector<double>, vector<double>, int **)) {
    KdNodeVector points_list = kd_tree.allnodes;
    double **adj_matrix = new double *[points_list.size()];
    for (int i = 0; i < points_list.size(); i++) {
        adj_matrix[i] = new double[points_list.size()];
        KdNodeVector results;
        kd_tree.k_nearest_neighbors(points_list.at(i).point, 1500, &results, NULL);
//        log_to_file("logger.txt","AFter knn");
        for (auto neighbour_it = results.begin(); neighbour_it != results.end(); neighbour_it++) {
            if (double distance = distance_metric(points_list.at(i).point, neighbour_it[0].point, grid)) {
//                log_to_file("logger.txt", "AFter double assigment");
//                log_to_file("logger.txt", "distance is"+to_string(distance));
//                log_to_file("logger.txt", "i is"+to_string(i));
//                log_to_file("logger.txt", "_id is"+to_string(neighbour_it->_id));

                adj_matrix[i][neighbour_it->_id] = distance;
//                adj_matrix[neighbour_it->_id][i] = distance;
//                log_to_file("logger.txt", "AFter distance assigment");

            }
        }
    }
//    log_to_file("logger.txt","Just before return");
    return adj_matrix;
}

double l2_distance(vector<double> source, vector<double> dst, int **grid) {
    if(source[0] == dst[0]) {
        if (source[1] == dst[1]) {
            return 0;
        }
    }
    if (obstacle_in_the_middle(source, dst, grid)) {
//        log_to_file("logger.txt","found obstacle between " + point_to_string(source) +" and " + point_to_string(dst));
        return 0;
    }
    return sqrt(pow((source[0] - dst[0]), 2) + pow(source[1] - dst[1], 2));
}


bool obstacle_in_the_middle(vector<double> source, vector<double> dst, int **grid) {
//    log_to_file("logger.txt", "In obstacle");
    double slope = ((source[1] - dst[1]) / (source[0] - dst[0]));
    double n = source[1] - slope * source[0];
    int number_of_points_to_sample = 5;
    double x_diff = dst[0] - source[0];

    if (source[0] != dst[0]){
        for (int i = 0; i < number_of_points_to_sample; i++) {
//        log_to_file("logger.txt", "i = " + to_string(i));

            double new_point_x = source[0] + (i / number_of_points_to_sample) * x_diff;
            double new_point_y = new_point_x * slope + n;
//        log_to_file("logger.txt", "before assigment: x = " + point_to_string(source) + " y = " + point_to_string(dst));
//        vector<double> new_point_vec;
//        new_point_vec.push_back(new_point_x);
//        new_point_vec.push_back(new_point_y);
//        log_to_file("logger.txt", "before assigment: new_point = " + point_to_string(new_point_vec));
            int obstacle = grid[(int) new_point_x][(int) new_point_y];
//        log_to_file("logger.txt", "after assigment");

            if (obstacle == 1) {
                return true;
            }
        }
    }
    else{
        double y_diff = dst[1] - source[1];
        for (int i = 0; i < number_of_points_to_sample; i++){
            double new_point_y = source[1] + (i / number_of_points_to_sample) * y_diff;
            int obstacle = grid[(int)source[0]][(int) new_point_y];
//        log_to_file("logger.txt", "after assigment");

            if (obstacle == 1) {
                return true;
            }
        }
    }
    return false;
}




vector<vector<double>> find_shortest_path(vector<double> src, vector<double> dst,double** adj_matrix,KdTree& kd_tree){
    //todo: You need to make sure obstacle isn't in the way - consider creating a new subclass to KDTree measure distnace with your distance;
    log_to_file("logger.txt","printing kd-tree nodes inside shortest path");

    KdNodeVector first_node;
    log_to_file("logger.txt","before first 1NN Searches");

    kd_tree.k_nearest_neighbors(src,1,&first_node);
    log_to_file("logger.txt","after first 1NN Searches");

    KdNodeVector last_node;
    kd_tree.k_nearest_neighbors(dst,1,&last_node);
    log_to_file("logger.txt","after second 1NN Searches");
    stack<int> nodes_id_path;
    log_to_file("logger.txt","Just after 1NN Searches");
    nodes_id_path = _find_shortest_path_from_two_nodes(first_node.at(0),last_node.at(0),adj_matrix);
    log_to_file("logger.txt","nodes_id_path size " + to_string(nodes_id_path.size()));
    vector<vector<double>> path = convert_path_from_id_to_coords(nodes_id_path,kd_tree);
    return path;
}

stack<int> _find_shortest_path_from_two_nodes(KdNode src,KdNode dst, double** adj_matrix){
    int previous_node[NUMBER_OF_VERTICES];
    double distances[NUMBER_OF_VERTICES];
    int unvisited[NUMBER_OF_VERTICES];
    std::fill_n (previous_node, NUMBER_OF_VERTICES, 99999);
    std::fill_n (distances, NUMBER_OF_VERTICES, 999999);
    std::fill_n (unvisited, NUMBER_OF_VERTICES, 1);
    distances[src._id] = 0;
    //todo: are the IDs alligned to the adj matrix? they should be.
    int counter = 0;
    log_to_file("logger.txt","dst id: " + to_string(dst._id) + "unvisited dst: " + to_string(unvisited[dst._id]));
    while (unvisited[dst._id] == 1){
        log_to_file("logger.txt","DJIKSTRAs counter " + to_string(counter++));
        int next_node = find_index_of_unvisited_min_index(distances,unvisited);
        log_to_file("logger.txt","next node is :" + to_string(next_node));

        unvisited[next_node] = 0;
        vector<int> neighbours_of_node = get_neighbours_of_node(next_node,adj_matrix);
        log_to_file("logger.txt","neighbehood size is :" + to_string(neighbours_of_node.size()));
        for (auto it = neighbours_of_node.begin();it!=neighbours_of_node.end();it++){
            double alternative_distance = distances[next_node] + adj_matrix[next_node][it[0]];
            if (alternative_distance < distances[it[0]]){
                distances[it[0]] = alternative_distance;
                previous_node[it[0]] = next_node;
            }
        }
    }
    log_to_file("logger.txt","Just after All unvisited");

    stack<int> path;
    int previous_step = dst._id;
    while(previous_step != src._id){
        path.push(previous_step);
        previous_step = previous_node[previous_step];
    }
    //Push src _id to path
    path.push(previous_node[previous_step]);
    return path;
}

int find_index_of_unvisited_min_index(double* array,int* unvisited){
    double min = 999999;
    int best_index = -1;
    int i =0;
//    log_to_file("logger.txt","starting to look for min\n");
    for (;i<NUMBER_OF_VERTICES;i++){
//        log_to_file("logger.txt","i: "+ to_string(i)+" min" + to_string(min) + "array[i]: "+to_string(array[i]) +'\n');
        if (array[i] < min && unvisited[i] == 1){
//            log_to_file("logger.txt","found min it's  :" + to_string(array[i])+"< " + to_string(min));
            min = array[i];
            best_index = i;
        }
    }
    return best_index;
}


vector<int> get_neighbours_of_node(int node_index,double **adj_matrix){
    vector<int> neighbours;
    for (int i=0;i < NUMBER_OF_VERTICES;i++){
        if ((adj_matrix[node_index][i] > 0) && adj_matrix[node_index][i] < 3){
            neighbours.push_back(i);

        }
    }
    return neighbours;
}

vector<vector<double>> convert_path_from_id_to_coords(stack<int> nodes_id_path,KdTree& kd_tree){
    vector<vector<double>> path;
    KdNodeVector nodes = kd_tree.allnodes;
    while(!nodes_id_path.empty()){
        int id = nodes_id_path.top();
        nodes_id_path.pop();
        KdNode node_of_id;
        for (auto it = nodes.begin();it!= nodes.end();it++){
            if (it[0]._id == id){
                path.push_back(it[0].point);
                break;
            }
        }
    }
    return path;
}


double l0_distance(vector<double> source, vector<double> dst, int **grid) {
    return 2.5;
}

bool is_cell_empty(int row, int col, int **grid) {
    return grid[row][col] == 0;
}


void print_nodes(const Kdtree::KdNodeVector &nodes) {
    log_to_file("logger.txt","started_printing nodes:\n");
    for (int i = 0; i < nodes.size(); ++i) {
        log_to_file("logger.txt",node_to_string(nodes[i]));
    }
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

vector<int> PRM_controller::coord_to_cell(vector<double> coord,double resolution){
    vector<int> cell;
    row = (coord[0] - origin.GetX())/resolution;
    col = (coord[1] - origin.GetY())/resolution;
    cell.push_back(row);
    cell.push_back(col);
    return cell;
}

void PRM_controller::pos_to_cell(CVector2 position, int *i, int *j, int size) {
    int pixel_i, pixel_j;
    pos_to_cord(position, &pixel_i, &pixel_j);
    *i = (int) ((pixel_i + 0.5 * size) / size) - 1;
    *j = (int) ((pixel_j + 0.5 * size) / size) - 1;
}


void print_path_to_file(vector<vector<double>>path,int** grid){
    int** new_grid = new int*[50];
    for (int i=0;i<50;i++){
        new_grid[i] = new int[50];
        for (int j=0;j<50;j++){
            new_grid[i][j] = grid[i][j];
        }
    }
    int step = 2;
    for (auto it = path.begin();it!=path.end();it++){
        new_grid[(int)it[0][0]][(int)it[0][1]] = step++;
    }
    saveGridToFile("path.txt",new_grid,50,50);


}

void print_path_as_list(vector<vector<double>>path){
    string str = "";
    for (auto it = path.begin();it!=path.end();it++){
        str += point_to_string(*it);
    }
    ofstream m_cOutput;
    m_cOutput.open("path_as_list.txt", ios_base::trunc | ios_base::out);
    m_cOutput << str << endl;
}

void print_nodes_in_grid(KdNodeVector nodes,int** grid){
    int** new_grid = new int*[50];
    for (int i=0;i<50;i++){
        new_grid[i] = new int[50];
        for (int j=0;j<50;j++){
            new_grid[i][j] = grid[i][j];
        }
    }
    for (auto it = nodes.begin();it!=nodes.end();it++){
        new_grid[(int)it[0].point[0]][(int)it[0].point[1]] = it[0]._id;
    }
    saveGridToFile("nodes_in_grid.txt",new_grid,50,50);

}