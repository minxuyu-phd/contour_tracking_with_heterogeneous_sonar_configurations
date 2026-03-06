#include "graphical_app.hpp"
#include "simulation_manager.hpp"
#include "sim_bridge/sensor_bridge.hpp"
#include "sim_bridge/actuator_bridge.hpp"
#include <nng_comm/nng_comm.hpp>
#include "utils/config_loader.hpp"
#include <Stonefish/graphics/IMGUI.h>
#include <Stonefish/graphics/OpenGLPipeline.h>
#include <Stonefish/graphics/OpenGLContent.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/core/Robot.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <atomic>

extern std::atomic<bool> g_running;

GraphicalApp::GraphicalApp(const std::string& dataDirPath,
                           sf::RenderSettings s,
                           sf::HelperSettings h,
                           SimulationManager* sim,
                           const nlohmann::json& net_cfg)
    : sf::GraphicalSimulationApp("AUV Simulation", dataDirPath, s, h, sim),
      net_config_(net_cfg),
      shutdown_requested_(false),
      show_debug_info_(false) {

    auto sim_cfg = utils::ConfigLoader::get_sim_config();
    if (sim_cfg.contains("graphics") && sim_cfg["graphics"].contains("follow_auv")) {
        follow_auv_ = sim_cfg["graphics"]["follow_auv"].get<bool>();
    }

    std::cout << "GraphicalApp: Initializing graphical simulation application..." << std::endl;
    std::cout << "GraphicalApp: Window size: " << s.windowW << "x" << s.windowH << std::endl;
    if (follow_auv_) {
        std::cout << "GraphicalApp: Follow AUV camera mode enabled" << std::endl;
    }

    if (!initBridges()) {
        std::cerr << "GraphicalApp: Warning - Failed to initialize some bridges" << std::endl;
    }
}

GraphicalApp::~GraphicalApp() {
    std::cout << "GraphicalApp: Cleaning up..." << std::endl;

    if (waypoint_vis_server_) {
        waypoint_vis_server_->stop();
        waypoint_vis_server_->close();
    }

    if (config_server_) {
        config_server_->stop();
        config_server_->close();
    }

    if (actuator_bridge_) {
        actuator_bridge_->stop();
    }
}

bool GraphicalApp::initBridges() {
    sensor_bridge_ = std::make_unique<sim_bridge::SensorBridge>(getSimulationManager());
    if (!sensor_bridge_->init(net_config_)) {
        std::cerr << "GraphicalApp: Failed to initialize sensor bridge" << std::endl;
        return false;
    }
    std::cout << "GraphicalApp: Sensor bridge initialized" << std::endl;

    actuator_bridge_ = std::make_unique<sim_bridge::ActuatorBridge>(getSimulationManager());
    if (!actuator_bridge_->init(net_config_)) {
        std::cerr << "GraphicalApp: Failed to initialize actuator bridge" << std::endl;
        return false;
    }
    std::cout << "GraphicalApp: Actuator bridge initialized" << std::endl;

    config_server_ = std::make_unique<nng_comm::Replier>();
    std::string config_addr = utils::ConfigLoader::get_service_address("config_server");
    if (!config_server_->init(config_addr)) {
        std::cerr << "GraphicalApp: Failed to initialize config server" << std::endl;
        return false;
    }

    setupConfigHandler();
    config_server_->start();
    std::cout << "GraphicalApp: Config server listening on " << config_addr << std::endl;

    waypoint_vis_server_ = std::make_unique<nng_comm::Replier>();
    std::string waypoint_addr = "tcp://127.0.0.1:7790";
    if (!waypoint_vis_server_->init(waypoint_addr)) {
        std::cerr << "GraphicalApp: Failed to initialize waypoint visualization server" << std::endl;
        return false;
    }

    setupWaypointVisHandler();
    waypoint_vis_server_->start();
    std::cout << "GraphicalApp: Waypoint visualization server listening on " << waypoint_addr << std::endl;

    return true;
}

void GraphicalApp::setupConfigHandler() {
    config_server_->set_handler([this](const nlohmann::json& req) -> nlohmann::json {
        std::cout << "GraphicalApp: Config request: " << req.dump() << std::endl;

        if (req.contains("command")) {
            std::string command = req["command"].get<std::string>();

            if (command == "pause") {
                std::cout << "GraphicalApp: Pausing simulation" << std::endl;
                return {{"status", "ok"}, {"message", "Simulation paused"}};
            }

            if (command == "resume") {
                std::cout << "GraphicalApp: Resuming simulation - resetting AUV state" << std::endl;
                auto* sim = dynamic_cast<SimulationManager*>(getSimulationManager());
                if (sim) {
                    sim->resetAUV();
                    return {{"status", "ok"}, {"message", "AUV reset to initial state"}};
                } else {
                    return {{"status", "error"}, {"message", "Failed to get simulation manager"}};
                }
            }

            if (command == "shutdown") {
                std::cout << "GraphicalApp: Shutdown requested" << std::endl;
                requestShutdown();
                return {{"status", "ok"}, {"message", "Shutting down"}};
            }

            if (command == "get_status") {
                return {
                    {"status", "ok"},
                    {"simulation_time", getSimulationManager()->getSimulationTime()},
                    {"graphics_enabled", true}
                };
            }

            if (command == "configure_msis") {
                if (!msis_sensor_) {
                    return {{"status", "error"}, {"message", "MSIS sensor not found"}};
                }
                nlohmann::json applied;

                if (req.contains("rotation_min") && req.contains("rotation_max")) {
                    double min_rot = req["rotation_min"];
                    double max_rot = req["rotation_max"];
                    msis_sensor_->setRotationLimits(min_rot, max_rot);
                    applied["rotation_min"] = min_rot;
                    applied["rotation_max"] = max_rot;
                }
                if (req.contains("range_min")) {
                    double r = req["range_min"];
                    msis_sensor_->setRangeMin(r);
                    applied["range_min"] = r;
                }
                if (req.contains("range_max")) {
                    double r = req["range_max"];
                    msis_sensor_->setRangeMax(r);
                    applied["range_max"] = r;
                }
                if (req.contains("gain")) {
                    double g = req["gain"];
                    msis_sensor_->setGain(g);
                    applied["gain"] = g;
                }
                if (req.contains("unidirectional")) {
                    bool u = req["unidirectional"];
                    msis_sensor_->setUnidirectional(u);
                    applied["unidirectional"] = u;
                }
                if (req.contains("step_multiplier")) {
                    int mult = req["step_multiplier"];
                    if (mult >= 1 && mult <= 10) {
                        msis_sensor_->setStepMultiplier(mult);
                        applied["step_multiplier"] = mult;
                    }
                }

                return {{"status", "ok"}, {"message", "MSIS configured"}, {"applied", applied}};
            }

            if (command == "get_msis_config") {
                if (!msis_sensor_) {
                    return {{"status", "error"}, {"message", "MSIS sensor not found"}};
                }
                sf::Scalar rot_min, rot_max;
                msis_sensor_->getRotationLimits(rot_min, rot_max);
                double base_step = msis_sensor_->getRotationStepAngle();
                int step_mult = msis_sensor_->getStepMultiplier();

                return {
                    {"status", "ok"},
                    {"rotation_min", rot_min},
                    {"rotation_max", rot_max},
                    {"range_min", msis_sensor_->getRangeMin()},
                    {"range_max", msis_sensor_->getRangeMax()},
                    {"gain", msis_sensor_->getGain()},
                    {"step", base_step},
                    {"step_multiplier", step_mult},
                    {"effective_step", base_step * step_mult},
                    {"unidirectional", msis_sensor_->isUnidirectional()}
                };
            }

            return {{"status", "error"}, {"message", "Unknown command: " + command}};
        }

        return {{"status", "error"}, {"message", "Missing 'command' field"}};
    });
}

void GraphicalApp::setupWaypointVisHandler() {
    waypoint_vis_server_->set_handler([this](const nlohmann::json& req) -> nlohmann::json {
        if (!req.contains("command")) {
            return {{"status", "error"}, {"message", "Missing command"}};
        }

        std::string cmd = req["command"].get<std::string>();

        if (cmd == "add_waypoint") {
            float x = req.value("x", 0.0f);
            float y = req.value("y", 0.0f);

            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            waypoint_history_.push_back(glm::vec2(x, y));

            std::cout << "GraphicalApp: Received waypoint (" << x << ", " << y
                      << "), total: " << waypoint_history_.size() << std::endl;

            return {{"status", "ok"}, {"waypoint_count", waypoint_history_.size()}};
        }

        if (cmd == "clear_waypoints") {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            waypoint_history_.clear();
            std::cout << "GraphicalApp: Cleared all waypoints" << std::endl;
            return {{"status", "ok"}};
        }

        return {{"status", "error"}, {"message", "Unknown command"}};
    });
}

void GraphicalApp::DoHUD() {
    if (!g_running) {
        Quit();
        return;
    }

    // Update follow-AUV camera rotation each frame
    if (follow_auv_ && follow_auv_link_) {
        sf::OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
        if (trackball) {
            sf::Transform auv_tf = follow_auv_link_->getOTransform();
            btQuaternion btq = auv_tf.getRotation();
            btScalar yaw, pitch, roll;
            btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);

            glm::quat yawRot = glm::angleAxis(-(float)yaw - glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            glm::quat pitchDown = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            trackball->Rotate(pitchDown * yawRot);
        }
    }

    sf::IMGUI* gui = getGUI();

    // Draw World Coordinate System Indicator
    {
        sf::OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
        sf::OpenGLContent* content = getGLPipeline()->getContent();

        if (trackball && content && gui) {
            const GLfloat screenX = 60.f;
            const GLfloat screenY = 60.f;
            const GLfloat axisLen = 40.f;

            int winW = getWindowWidth();
            int winH = getWindowHeight();

            glm::mat4 viewMat = trackball->GetViewMatrix();
            glm::mat3 viewRot = glm::mat3(viewMat);
            glm::mat3 invRot = glm::transpose(viewRot);

            glm::vec3 axisX = invRot * glm::vec3(1.f, 0.f, 0.f);
            glm::vec3 axisY = invRot * glm::vec3(0.f, 1.f, 0.f);
            glm::vec3 axisZ = invRot * glm::vec3(0.f, 0.f, 1.f);

            glm::vec2 origin(screenX, screenY);
            glm::vec2 endX = origin + glm::vec2(axisX.x, -axisX.y) * axisLen;
            glm::vec2 endY = origin + glm::vec2(axisY.x, -axisY.y) * axisLen;
            glm::vec2 endZ = origin + glm::vec2(axisZ.x, -axisZ.y) * axisLen;

            GLint currentVAO;
            glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &currentVAO);

            glm::mat4 orthoProj = glm::ortho(0.f, (float)winW, 0.f, (float)winH, -1.f, 1.f);
            content->SetProjectionMatrix(orthoProj);
            content->SetViewMatrix(glm::mat4(1.f));

            auto screenToGL = [winH](glm::vec2 screenPt) -> glm::vec3 {
                return glm::vec3(screenPt.x, (float)winH - screenPt.y, 0.f);
            };

            glm::vec3 glOrigin = screenToGL(origin);
            glm::vec3 glEndX = screenToGL(endX);
            glm::vec3 glEndY = screenToGL(endY);
            glm::vec3 glEndZ = screenToGL(endZ);

            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
            glLineWidth(2.0f);

            std::vector<glm::vec3> lineX = { glOrigin, glEndX };
            content->DrawPrimitives(sf::PrimitiveType::LINES, &lineX, glm::vec4(1.f, 0.3f, 0.3f, 1.f));

            std::vector<glm::vec3> lineY = { glOrigin, glEndY };
            content->DrawPrimitives(sf::PrimitiveType::LINES, &lineY, glm::vec4(0.3f, 1.f, 0.3f, 1.f));

            std::vector<glm::vec3> lineZ = { glOrigin, glEndZ };
            content->DrawPrimitives(sf::PrimitiveType::LINES, &lineZ, glm::vec4(0.5f, 0.5f, 1.f, 1.f));

            glDisable(GL_LINE_SMOOTH);
            glLineWidth(1.0f);

            glBindVertexArray(currentVAO);

            gui->DoLabel(endX.x + 5.f, endX.y - 5.f, "X", glm::vec4(1.f, 0.3f, 0.3f, 1.f), 0.8f);
            gui->DoLabel(endY.x + 5.f, endY.y - 5.f, "Y", glm::vec4(0.3f, 1.f, 0.3f, 1.f), 0.8f);
            gui->DoLabel(endZ.x + 5.f, endZ.y - 5.f, "Z", glm::vec4(0.5f, 0.5f, 1.f, 1.f), 0.8f);
        }
    }

    // Draw Waypoint Spheres
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);
        sf::OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
        sf::OpenGLContent* content = getGLPipeline()->getContent();

        if (content && trackball && !waypoint_history_.empty()) {
            content->SetProjectionMatrix(trackball->GetProjectionMatrix());
            content->SetViewMatrix(trackball->GetViewMatrix());

            size_t n = waypoint_history_.size();

            for (size_t i = 0; i < n; ++i) {
                const auto& wp = waypoint_history_[i];
                bool is_latest = (i == n - 1);

                glm::mat4 model = glm::translate(glm::mat4(1.f),
                    glm::vec3(wp.x, wp.y, 0.f));

                float radius = is_latest ? 0.5f : 0.3f;
                glm::vec4 color = is_latest ?
                    glm::vec4(1.0f, 0.2f, 0.2f, 0.9f) :
                    glm::vec4(1.0f, 0.5f, 0.5f, 0.6f);

                glEnable(GL_MULTISAMPLE);
                content->DrawEllipsoid(model, glm::vec3(radius), color);
                glDisable(GL_MULTISAMPLE);
            }
        }
    }

    // HUD info panel
    if (!gui || !show_debug_info_) {
        return;
    }

    GLfloat panelX = 10.f;
    GLfloat panelY = 10.f;
    GLfloat panelW = 250.f;
    GLfloat panelH = 100.f;

    gui->DoPanel(panelX, panelY, panelW, panelH);

    gui->DoLabel(panelX + 10.f, panelY + 5.f, "AUV Simulation");

    double sim_time = getSimulationManager()->getSimulationTime();
    char time_str[64];
    snprintf(time_str, sizeof(time_str), "Time: %.2f s", sim_time);
    gui->DoLabel(panelX + 10.f, panelY + 30.f, time_str);

    gui->DoLabel(panelX + 10.f, panelY + 55.f, "Graphics: Enabled");
    gui->DoLabel(panelX + 10.f, panelY + 80.f, "Press F1 to toggle info");
}

void GraphicalApp::StartSimulation() {
    std::cout << "GraphicalApp: Starting simulation..." << std::endl;

    ShowHUD();

    auto* sim = dynamic_cast<SimulationManager*>(getSimulationManager());
    if (sim) {
        if (sensor_bridge_) {
            sim->setSensorBridge(sensor_bridge_.get());
            std::cout << "GraphicalApp: Sensor bridge connected to simulation manager" << std::endl;
        }
        if (actuator_bridge_) {
            sim->setActuatorBridge(actuator_bridge_.get());
            std::cout << "GraphicalApp: Actuator bridge connected to simulation manager" << std::endl;
        }
    }

    sf::GraphicalSimulationApp::StartSimulation();

    // Initialize MSIS sensor pointer for dynamic configuration
    sf::Sensor* sensor = getSimulationManager()->getSensor("sam/msis_link");
    if (sensor) {
        msis_sensor_ = dynamic_cast<sf::MSIS*>(sensor);
        if (msis_sensor_) {
            std::cout << "GraphicalApp: MSIS sensor initialized for dynamic configuration" << std::endl;
        }
    }

    // Setup follow AUV camera mode
    if (follow_auv_) {
        sf::Robot* robot = getSimulationManager()->getRobot("sam");
        if (robot) {
            follow_auv_link_ = robot->getBaseLink();
            sf::OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if (trackball && follow_auv_link_) {
                trackball->GlueToMoving(follow_auv_link_);
                std::cout << "GraphicalApp: Camera following AUV from behind-above" << std::endl;
            }
        } else {
            std::cerr << "GraphicalApp: Warning - Robot 'sam' not found for follow mode" << std::endl;
        }
    }

    if (actuator_bridge_) {
        actuator_bridge_->start();
        std::cout << "GraphicalApp: Actuator bridge started" << std::endl;
    }

    std::cout << "GraphicalApp: Simulation started" << std::endl;
}

void GraphicalApp::StopSimulation() {
    std::cout << "GraphicalApp: Stopping simulation..." << std::endl;

    if (actuator_bridge_) {
        actuator_bridge_->stop();
        std::cout << "GraphicalApp: Actuator bridge stopped" << std::endl;
    }

    if (config_server_) {
        config_server_->stop();
        std::cout << "GraphicalApp: Config server stopped" << std::endl;
    }

    if (waypoint_vis_server_) {
        waypoint_vis_server_->stop();
        std::cout << "GraphicalApp: Waypoint visualization server stopped" << std::endl;
    }

    sf::GraphicalSimulationApp::StopSimulation();

    std::cout << "GraphicalApp: Simulation stopped" << std::endl;
}

void GraphicalApp::KeyDown(SDL_Event* event) {
    switch (event->key.keysym.sym) {
        case SDLK_F1:
            show_debug_info_ = !show_debug_info_;
            std::cout << "GraphicalApp: Debug info " << (show_debug_info_ ? "enabled" : "disabled") << std::endl;
            break;

        case SDLK_F2:
            getHelperSettings().showSensors = !getHelperSettings().showSensors;
            std::cout << "GraphicalApp: Sensor visualization "
                      << (getHelperSettings().showSensors ? "enabled" : "disabled") << std::endl;
            break;

        case SDLK_F3:
            getHelperSettings().showActuators = !getHelperSettings().showActuators;
            std::cout << "GraphicalApp: Actuator visualization "
                      << (getHelperSettings().showActuators ? "enabled" : "disabled") << std::endl;
            break;

        case SDLK_F4:
            getHelperSettings().showCoordSys = !getHelperSettings().showCoordSys;
            std::cout << "GraphicalApp: Coordinate system visualization "
                      << (getHelperSettings().showCoordSys ? "enabled" : "disabled") << std::endl;
            break;

        default:
            sf::GraphicalSimulationApp::KeyDown(event);
            break;
    }
}

void GraphicalApp::requestShutdown() {
    shutdown_requested_ = true;
    g_running = false;
}

bool GraphicalApp::isShutdownRequested() const {
    return shutdown_requested_;
}

void GraphicalApp::MouseDown(SDL_Event* event) {
    if (follow_auv_) return;
    sf::GraphicalSimulationApp::MouseDown(event);
}

void GraphicalApp::MouseUp(SDL_Event* event) {
    if (follow_auv_) return;
    sf::GraphicalSimulationApp::MouseUp(event);
}

void GraphicalApp::MouseMove(SDL_Event* event) {
    if (follow_auv_) return;
    sf::GraphicalSimulationApp::MouseMove(event);
}

void GraphicalApp::MouseScroll(SDL_Event* event) {
    if (follow_auv_) return;
    sf::GraphicalSimulationApp::MouseScroll(event);
}
