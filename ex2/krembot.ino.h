// Shlomi Ben-Shushan 311408264
// Khen Aharon 307947515

// We didn't change anything in this file.

#include <Krembot/controller/krembot_controller.h>

class ex2_controller : public KrembotController {
private:
    bool isFirst = true;
public:
    ParticleObserver Particle;
    ~ex2_controller() = default;
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
};


REGISTER_CONTROLLER(ex2_controller, "ex2_controller")
