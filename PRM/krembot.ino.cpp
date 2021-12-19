#include "krembot.ino.h"
#include <string>
#include <random>
#include <ctime>
#include <cmath>
#include <iostream>
#include "kdtree.hpp"
#include <stack>
#include <stdlib.h>

#define NUMBER_OF_VERTICES 12000
#define REDUCTION_FACTOR 1
// Comment DEBUG out in order to prevent logger from being writ.
#define DEBUG
#define K 30
using namespace std;
using namespace Kdtree;
SandTimer timer;

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
calculate_adj_matrix(KdTree &kd_tree, int **&grid, double (*distance_metric)(vector<double>, vector<double>, int **,bool));

double l1_distance(vector<double> source, vector<double> dst, int **grid,bool driving);

double l2_distance(vector<double> source, vector<double> dst, int **grid,bool driving);

void uniform_sampling(int &row, int &col, int max_heigth, int max_width);

bool obstacle_in_the_middle(vector<double> source, vector<double> dst, int **grid);

bool is_cell_empty(int row, int col, int **grid);

stack<int> _find_shortest_path_from_two_nodes(KdNode src, KdNode dst, double **adj_matrix);

int find_index_of_unvisited_min_index(double *distances, int *unvisited);

string node_to_string(KdNode node);

vector<vector<double>> find_shortest_path(vector<double> src, vector<double> dst, double **adj_matrix, KdTree &kdTree);

vector<int> get_neighbours_of_node(int node_index, double **adj_matrix);

vector<vector<double>> convert_path_from_id_to_coords(stack<int> nodes_id_path, KdTree &kd_tree);

string point_to_string(vector<double> point);

void print_path_to_file(vector<vector<double>> path, int **grid);

void print_path_as_list(vector<vector<double>> path);

void print_nodes_in_grid(KdNodeVector nodes, int **grid);

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
    vector<int> starting_cell = PRM_controller::coord_to_cell(starting_pos, resolution * REDUCTION_FACTOR);
    vector<int> goal_cell = PRM_controller::coord_to_cell(goal_pos, resolution * REDUCTION_FACTOR);
    std::vector<double> starting_cell_double(starting_cell.begin(), starting_cell.end());
    std::vector<double> goal_cell_double(goal_cell.begin(), goal_cell.end());

    int ** inflated_obstacles_grid = PRM_controller::create_new_grid(occupancyGrid, height, width);

    this->new_grid = lowerGridResolution(inflated_obstacles_grid,REDUCTION_FACTOR,height,width);
#ifdef DEBUG
    saveGridToFile("grid", occupancyGrid, height, width);
    saveGridToFile("inflated", inflated_obstacles_grid, height, width);
    saveGridToFile("new_grid", new_grid, height/REDUCTION_FACTOR, width/REDUCTION_FACTOR);
    log_to_file("logger.txt", "created new grid:");

#endif
    vector<double *> vec = sample_n_free_points(NUMBER_OF_VERTICES, height / (REDUCTION_FACTOR), width / REDUCTION_FACTOR,
                                                new_grid, *uniform_sampling);
    KdNodeVector nodes = insert_points_to_nodes(vec);
    Kdtree::KdTree kd_tree(&nodes);
    print_nodes_in_grid(kd_tree.allnodes, new_grid);
    double **adj_matrix = calculate_adj_matrix(kd_tree, new_grid, l2_distance);
    this->path = find_shortest_path(starting_cell_double, goal_cell_double, adj_matrix, kd_tree);

#ifdef DEBUG
    log_adj_mat_to_file(adj_matrix);
    print_path_to_file(path, new_grid);
    print_path_as_list(path);
#endif

}

void PRM_controller::loop() {
    krembot.loop();

    if (this->close_enough(PRM_controller::posMsg.pos, this->goal)) {
        log_to_file("logger.txt", "At goal! pos: " + to_string(PRM_controller::posMsg.pos.GetX()) + ',' +
                                  to_string(PRM_controller::posMsg.pos.GetY()));
        krembot.Base.stop();
        krembot.Led.write(255, 150, 130);
        this->reached_goal = true;
    }

    if ((this->close_enough(PRM_controller::posMsg.pos, this->next_stop)) && !reached_goal) {
        krembot.Base.stop();
        this->next_stop = this->get_next_stop_from_path();
        this->right_angle = PRM_controller::get_position_to_destination(PRM_controller::posMsg.pos, this->next_stop);
        this->setup_angle = true;
    }
    if (this->setup_angle) {
        krembot.Base.drive(0, ANGULAR_SPEED);
    }
    if (((NormalizedDifference(PRM_controller::posMsg.degreeX.UnsignedNormalize(),
                               this->right_angle.UnsignedNormalize()).UnsignedNormalize()) < CDegrees(MAXIMAL_DEGREE_DIFF)) &&
        !reached_goal) {
        krembot.Base.drive(100, 0);
        this->setup_angle = false;
    }

}







CVector2 PRM_controller::get_next_stop_from_path() {
    if (path_count == path.size()) {
        return this->goal;
    }
    vector<double> next_grid_coord = this->path.at(this->path_count);
    this->path_count = this->path_count + 1;
    CVector2 next_stop = PRM_controller::cell_to_cord(next_grid_coord[0], next_grid_coord[1], REDUCTION_FACTOR);
    return next_stop;
}

bool PRM_controller::close_enough(CVector2 pos, CVector2 dst) {
    vector<double> src;
    vector<double> destination;
    int pos_i, pos_j, dst_i, dst_j;
    pos_to_cord(pos, &pos_i, &pos_j);
    pos_to_cord(dst, &dst_i, &dst_j);
    destination.push_back(dst_i);
    destination.push_back(dst_j);
    src.push_back(pos_i);
    src.push_back(pos_j);
    double distance = l2_distance(src, destination, this->new_grid,true);
    if ((0 < distance) && (distance <= 2)) {
        return true;
    } else {
        return false;
    }
}

CDegrees PRM_controller::get_position_to_destination(CVector2 pos, CVector2 dst) {
    double dx = dst.GetX() - pos.GetX();
    double dy = dst.GetY() - pos.GetY();
    double angle = atan2(dy, dx);
    CDegrees right_degree = CDegrees(angle * 180 / ARGOS_PI).UnsignedNormalize();
    return right_degree;
}



int **PRM_controller::create_new_grid(int **grid, int grid_height, int grid_width) {

    int **created_grid = new int *[grid_height];
    double robot_size_in_cells = robotSize / resolution;

    for (int column = 0; column < grid_height; column++) {
        created_grid[column] = new int[grid_width];
    }
    for (int i = 0; i < grid_height; i++) {
        for (int j = 0; j < grid_width; j++) {
            created_grid[i][j] = 0;
        }
    }
    for (int _row = height - 1; _row >= 0; _row--) {
        for (int _col = 0; _col < width; _col++) {
            if (grid[_row][_col] == 1) {
                int height_max = _row + robot_size_in_cells + 1;
                int height_min = _row - robot_size_in_cells - 1;
                int width_max = _col + robot_size_in_cells + 1;
                int width_min = _col - robot_size_in_cells - 1;
                for (int i = min(height-1, height_max);
                     i >= max(0, height_min); i--) {
                    for (int j = max(width_min, 0);
                         j < min(width, width_max-1); j++) {
                        created_grid[i][j] = 1;

                    }
                }
            }
        }
    }
    return created_grid;
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
}

string node_to_string(KdNode node) {
    return "id: " + to_string(node._id) + "point: (" + to_string(node.point[0]) + ',' + to_string(node.point[1]) +
           ")\n";
}

string point_to_string(vector<double> point) {
    return "(" + to_string(point[0]) + ',' + to_string(point[1]) + ")";
}

void saveGridToFile(string name, int **grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            m_cOutput << grid[row][col] << ',';
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
// We return the coordinate in the middle of the cell
    double half_cell = reduction_factor * resolution / 2;
    Real x_cord = half_cell + origin.GetX() + reduction_factor * resolution * cell_j;
    Real y_cord = half_cell + origin.GetY() + reduction_factor * resolution * cell_i;
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
calculate_adj_matrix(KdTree &kd_tree, int **&grid,
                     double (*distance_metric)(vector<double>, vector<double>, int **,bool)) {
    KdNodeVector points_list = kd_tree.allnodes;
    double **adj_matrix = new double *[points_list.size()];
    for (int i = 0; i < points_list.size(); i++) {
        adj_matrix[i] = new double[points_list.size()];
        KdNodeVector results;
        kd_tree.k_nearest_neighbors(points_list.at(i).point, K, &results, NULL);
        for (auto neighbour_it = results.begin(); neighbour_it != results.end(); neighbour_it++) {
            if (double distance = distance_metric(points_list.at(i).point, neighbour_it[0].point, grid, false)) {
                adj_matrix[i][neighbour_it->_id] = distance;


            }
        }
    }
    return adj_matrix;
}

double l2_distance(vector<double> source, vector<double> dst, int **grid,bool driving = false) {
    if (source[0] == dst[0]) {
        if (source[1] == dst[1]) {
            return 0.0001;
        }
    }
    if (obstacle_in_the_middle(source, dst, grid) && !driving) {
        return 0;
    }
    return sqrt(pow((source[0] - dst[0]), 2) + pow(source[1] - dst[1], 2));
}
double l1_distance(vector<double> source, vector<double> dst, int **grid,bool driving = false) {
    if (source[0] == dst[0]) {
        if (source[1] == dst[1]) {
            return 0.0001;
        }
    }
    if (obstacle_in_the_middle(source, dst, grid) && !driving) {
        return 0;
    }
    return abs(source[0] - dst[0])+ abs(source[1] - dst[1]);
}

bool obstacle_in_the_middle(vector<double> source, vector<double> dst, int **grid) {


    double distance_between_points = sqrt(pow((source[0] - dst[0]), 2) + pow(source[1] - dst[1], 2));
    int number_of_points_to_sample = 3 * (int) distance_between_points;
    double x_diff = dst[0] - source[0];
    if (source[0] != dst[0]) {
        double slope = ((source[1] - dst[1]) / (source[0] - dst[0]));
        double n = source[1] - slope * source[0];
        int obstacle = 0;
        for (int i = 0; i < number_of_points_to_sample; i++) {
            double new_point_x = source[0] + (i / number_of_points_to_sample) * x_diff;
            double new_point_y = new_point_x * slope + n;

            int x_point_to_check = (int) new_point_x;
            int y_point_to_check = (int) new_point_y;

                    obstacle = grid[x_point_to_check][y_point_to_check];
            if (obstacle == 1) {
                return true;
            }
        }
    } else {
        double y_diff = dst[1] - source[1];
        int obstacle = 0;
        for (int i = 0; i < number_of_points_to_sample; i++) {
            double new_point_y = source[1] + (i / number_of_points_to_sample) * y_diff;
            int x_point_to_check = (int) source[0];
            int y_point_to_check = (int) new_point_y;
                    obstacle = grid[x_point_to_check][y_point_to_check];





            if (obstacle == 1) {
                return true;
            }
        }
    }
    return false;
}


vector<vector<double>>
find_shortest_path(vector<double> src, vector<double> dst, double **adj_matrix, KdTree &kd_tree) {
    // Implementing Dijkstra Shortest Path algorithm in _find_shortest_path_from_two_nodes
    KdNodeVector first_node;
    kd_tree.k_nearest_neighbors(src, 1, &first_node);
    KdNodeVector last_node;
    kd_tree.k_nearest_neighbors(dst, 1, &last_node);
    stack<int> nodes_id_path;

    nodes_id_path = _find_shortest_path_from_two_nodes(first_node.at(0), last_node.at(0), adj_matrix);
    vector<vector<double>> path = convert_path_from_id_to_coords(nodes_id_path, kd_tree);
    return path;
}

stack<int> _find_shortest_path_from_two_nodes(KdNode src, KdNode dst, double **adj_matrix) {
    int previous_node[NUMBER_OF_VERTICES];
    double distances[NUMBER_OF_VERTICES];
    int unvisited[NUMBER_OF_VERTICES];
    std::fill_n(previous_node, NUMBER_OF_VERTICES, 99999);
    std::fill_n(distances, NUMBER_OF_VERTICES, 999999);
    std::fill_n(unvisited, NUMBER_OF_VERTICES, 1);
    distances[src._id] = 0;
    int counter = 0;
    while (unvisited[dst._id] == 1) {
        int next_node = find_index_of_unvisited_min_index(distances, unvisited);
        unvisited[next_node] = 0;
        vector<int> neighbours_of_node = get_neighbours_of_node(next_node, adj_matrix);
        for (auto it = neighbours_of_node.begin(); it != neighbours_of_node.end(); it++) {
            double alternative_distance = distances[next_node] + adj_matrix[next_node][it[0]];
            if (alternative_distance < distances[it[0]]) {
                distances[it[0]] = alternative_distance;
                previous_node[it[0]] = next_node;
            }
        }
    }
    stack<int> path;
    int previous_step = dst._id;
    while (previous_step != src._id) {
        path.push(previous_step);
        previous_step = previous_node[previous_step];
    }
    //Push src _id to path
    path.push(previous_node[previous_step]);
    return path;
}

int find_index_of_unvisited_min_index(double *array, int *unvisited) {
    double min = 999999;
    int best_index = -1;
    int i = 0;
    for (; i < NUMBER_OF_VERTICES; i++) {
        if (array[i] < min && unvisited[i] == 1) {
            min = array[i];
            best_index = i;
        }
    }
    return best_index;
}


vector<int> get_neighbours_of_node(int node_index, double **adj_matrix) {
    vector<int> neighbours;
    for (int i = 0; i < NUMBER_OF_VERTICES; i++) {
        if ((adj_matrix[node_index][i] > 0) && adj_matrix[node_index][i] < 10) {
            neighbours.push_back(i);

        }
    }
    return neighbours;
}

vector<vector<double>> convert_path_from_id_to_coords(stack<int> nodes_id_path, KdTree &kd_tree) {
    vector<vector<double>> path;
    KdNodeVector nodes = kd_tree.allnodes;
    while (!nodes_id_path.empty()) {
        int id = nodes_id_path.top();
        nodes_id_path.pop();
        KdNode node_of_id;
        for (auto it = nodes.begin(); it != nodes.end(); it++) {
            if (it[0]._id == id) {
                path.push_back(it[0].point);
                break;
            }
        }
    }
    return path;
}



bool is_cell_empty(int row, int col, int **grid) {
    return grid[row][col] == 0;
}


void print_nodes(const Kdtree::KdNodeVector &nodes) {
    log_to_file("logger.txt", "started_printing nodes:\n");
    for (int i = 0; i < nodes.size(); ++i) {
        log_to_file("logger.txt", node_to_string(nodes[i]));
    }
}

void PRM_controller::pos_to_cord(CVector2 position, int *i, int *j) {
    *i = (position.GetX() - origin.GetX()) / (resolution * REDUCTION_FACTOR);
    *j = (position.GetY() - origin.GetY()) / (resolution * REDUCTION_FACTOR);
}

vector<int> PRM_controller::coord_to_cell(vector<double> coord, double resolution) {
    vector<int> cell;
    row = (coord[0] - origin.GetX()) / resolution;
    col = (coord[1] - origin.GetY()) / resolution;
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


void print_path_to_file(vector<vector<double>> path, int **grid) {
    int size = width / REDUCTION_FACTOR;
    int **new_grid = new int *[size];
    for (int i = 0; i < size; i++) {
        new_grid[i] = new int[size];
        for (int j = 0; j < size; j++) {
            new_grid[i][j] = grid[i][j];
        }
    }
    int step = 2;
    for (auto it = path.begin(); it != path.end(); it++) {
        new_grid[(int) it[0][0]][(int) it[0][1]] = step++;
    }
    saveGridToFile("path.txt", new_grid, size, size);


}

void print_path_as_list(vector<vector<double>> path) {
    string str = "";
    for (auto it = path.begin(); it != path.end(); it++) {
        str += point_to_string(*it);
    }
    ofstream m_cOutput;
    m_cOutput.open("path_as_list.txt", ios_base::trunc | ios_base::out);
    m_cOutput << str << endl;
}

void print_nodes_in_grid(KdNodeVector nodes, int **grid) {
    int size = width / REDUCTION_FACTOR;
    int **new_grid = new int *[size];
    for (int i = 0; i < size; i++) {
        new_grid[i] = new int[size];
        for (int j = 0; j < size; j++) {
            new_grid[i][j] = grid[i][j];
        }
    }
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        new_grid[(int) it[0].point[0]][(int) it[0].point[1]] = it[0]._id;
    }
    saveGridToFile("nodes_in_grid.txt", new_grid, size, size);

}
