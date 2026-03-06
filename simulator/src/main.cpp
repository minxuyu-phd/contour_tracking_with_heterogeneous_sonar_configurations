#include "simulation_manager.hpp"
#include "graphical_app.hpp"
#include "utils/config_loader.hpp"
#include "utils/render_settings_mapper.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running(true);

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

void print_banner() {
    std::cout << "=== AUV Simulation System ===" << std::endl;
    std::cout << "Based on Stonefish + NNG + JSON" << std::endl;
    std::cout << std::endl;
}

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [config_dir]" << std::endl;
    std::cout << "  config_dir: Directory containing config.jsonc (default: config/)" << std::endl;
    std::cout << std::endl;
    std::cout << "Configuration options in config.jsonc:" << std::endl;
    std::cout << "  graphics.resolution: [width, height] - Window resolution" << std::endl;
    std::cout << "  graphics.quality: low/medium/high - Rendering quality preset" << std::endl;
    std::cout << "  graphics.vsync: true/false - Vertical sync" << std::endl;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    print_banner();

    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        print_usage(argv[0]);
        return 0;
    }

    // 1. Load configuration
    std::string config_dir = "config/";
    if (argc > 1) {
        config_dir = argv[1];
    }

    std::cout << "Loading configuration from: " << config_dir << std::endl;
    if (!utils::ConfigLoader::load(config_dir)) {
        std::cerr << "Failed to load configuration" << std::endl;
        return -1;
    }

    auto sim_cfg = utils::ConfigLoader::get_sim_config();
    auto net_cfg = utils::ConfigLoader::get_network_config();

    // 2. Create simulation manager
    std::cout << "Initializing simulation manager..." << std::endl;

    SimulationManager* sim_manager = nullptr;

    try {
        int rate_hz = sim_cfg["simulation"]["rate_hz"].get<int>();
        std::string scene_file = sim_cfg["environment"]["scene_file"];

        sim_manager = new SimulationManager(
            static_cast<sf::Scalar>(rate_hz),
            scene_file
        );

        std::cout << "Simulation manager initialized at " << rate_hz << " Hz" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error initializing simulation: " << e.what() << std::endl;
        return -1;
    }

    // 3. Start graphical simulation
    std::cout << "Starting graphical simulation..." << std::endl;

    sf::RenderSettings render_settings = utils::mapConfigToRenderSettings(sim_cfg["graphics"]);
    sf::HelperSettings helper_settings = utils::mapConfigToHelperSettings(sim_cfg["graphics"]);

    std::cout << "Window size: " << render_settings.windowW << "x" << render_settings.windowH << std::endl;

    GraphicalApp app("data/", render_settings, helper_settings, sim_manager, net_cfg);

    std::cout << "\n=== Starting graphical simulation ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  F1 - Toggle debug info" << std::endl;
    std::cout << "  F2 - Toggle sensor visualization" << std::endl;
    std::cout << "  F3 - Toggle actuator visualization" << std::endl;
    std::cout << "  F4 - Toggle coordinate system" << std::endl;
    std::cout << "  ESC - Exit simulation" << std::endl;
    std::cout << std::endl;

    app.Run();

    std::cout << "Graphical simulation ended" << std::endl;
    std::cout << "Shutdown complete" << std::endl;

    return 0;
}
