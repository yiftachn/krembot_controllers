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
            setup();
            isFirst = false;
        }
        loop();
    }
    static void pos_to_cord(CVector2 pos, int *j, int *i);
    static void pos_to_cell(CVector2 position, int *i, int *j, int size);
    static CVector2 cell_to_cord(int cell_i,int cell_j,int reduction_factor);
    static std::vector<int> coord_to_cell(std::vector<double> coord,double resolution);
};





REGISTER_CONTROLLER(PRM_controller, "PRM_controller")



