#ifndef GRAPHICAL_APP_HPP
#define GRAPHICAL_APP_HPP

#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/entities/SolidEntity.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>

// Forward declarations
class SimulationManager;

namespace sf {
    class MSIS;
}

namespace sim_bridge {
    class SensorBridge;
    class ActuatorBridge;
}

namespace nng_comm {
    class Replier;
}

class GraphicalApp : public sf::GraphicalSimulationApp {
public:
    GraphicalApp(const std::string& dataDirPath,
                 sf::RenderSettings s,
                 sf::HelperSettings h,
                 SimulationManager* sim,
                 const nlohmann::json& net_cfg);

    virtual ~GraphicalApp();

    void DoHUD() override;
    void StartSimulation() override;
    void StopSimulation() override;
    void KeyDown(SDL_Event* event) override;

    void MouseDown(SDL_Event* event) override;
    void MouseUp(SDL_Event* event) override;
    void MouseMove(SDL_Event* event) override;
    void MouseScroll(SDL_Event* event) override;

    void requestShutdown();
    bool isShutdownRequested() const;

private:
    bool initBridges();
    void setupConfigHandler();
    void setupWaypointVisHandler();

    std::unique_ptr<sim_bridge::SensorBridge> sensor_bridge_;
    std::unique_ptr<sim_bridge::ActuatorBridge> actuator_bridge_;
    std::unique_ptr<nng_comm::Replier> config_server_;
    std::unique_ptr<nng_comm::Replier> waypoint_vis_server_;

    std::vector<glm::vec2> waypoint_history_;
    std::mutex waypoint_mutex_;

    nlohmann::json net_config_;
    std::atomic<bool> shutdown_requested_;
    bool show_debug_info_;

    // MSIS sensor pointer for dynamic configuration
    sf::MSIS* msis_sensor_ = nullptr;

    // Follow AUV camera mode
    bool follow_auv_ = false;
    sf::SolidEntity* follow_auv_link_ = nullptr;
};

#endif // GRAPHICAL_APP_HPP
