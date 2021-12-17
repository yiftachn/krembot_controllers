#include <Krembot/controller/krembot_controller.h>
#include <queue>

struct MapMsg{
    int ** occupancyGrid;
    Real resolution;
    CVector2 origin;
    int height, width;
};

struct PosMsg{
    CVector2 pos;
    CDegrees degreeX;
};


class PRM_controller : public KrembotController {
private:
    Real robotSize = 0.20;
    bool isFirst = true;
    bool setup_angle = false;
    CVector2 goal;
    CVector2 starting_position;
    CVector2 next_stop;
    CVector2 last_stop = starting_position;
    int** new_grid;
    int path_count =0;
    CDegrees right_angle;
    std::vector<std::vector<double>> path;
     bool reached_goal = false;
public:
    MapMsg mapMsg;
    PosMsg posMsg;

    ParticleObserver Particle;
    ~PRM_controller() = default;
    void setup();
    void loop();

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if ( ! krembot.isInitialized() ) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }
    void ControlStep() override {
        if(isFirst) {

            goal = CVector2(Real(-2),Real(-2));
            starting_position = CVector2(Real(1),Real(1));
            next_stop = starting_position;
            setup();
            isFirst = false;

        }
        loop();
    }
    static void pos_to_cord(CVector2 pos, int *j, int *i);
    static void pos_to_cell(CVector2 position, int *i, int *j, int size);
    static CVector2 cell_to_cord(int cell_i,int cell_j,int reduction_factor);
    static std::vector<int> coord_to_cell(std::vector<double> coord,double resolution);
    static CDegrees get_position_to_destination(CVector2 pos, CVector2 dst);
    bool close_enough(CVector2 pos,CVector2 dst);
    CVector2 get_next_stop_from_path();
    int ** create_new_grid(int **grid, int grid_height, int grid_width);

    };





REGISTER_CONTROLLER(PRM_controller, "PRM_controller")



