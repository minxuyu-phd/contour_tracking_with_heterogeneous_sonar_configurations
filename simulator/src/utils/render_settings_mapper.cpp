#include "utils/render_settings_mapper.hpp"

namespace utils {

sf::RenderQuality stringToRenderQuality(const std::string& quality) {
    if (quality == "disabled") {
        return sf::RenderQuality::DISABLED;
    } else if (quality == "low") {
        return sf::RenderQuality::LOW;
    } else if (quality == "medium") {
        return sf::RenderQuality::MEDIUM;
    } else if (quality == "high") {
        return sf::RenderQuality::HIGH;
    }
    return sf::RenderQuality::MEDIUM;
}

sf::RenderSettings mapConfigToRenderSettings(const nlohmann::json& graphics_cfg) {
    sf::RenderSettings settings;

    if (graphics_cfg.contains("resolution") && graphics_cfg["resolution"].is_array()) {
        auto res = graphics_cfg["resolution"];
        if (res.size() >= 2) {
            settings.windowW = res[0].get<int>();
            settings.windowH = res[1].get<int>();
        }
    }

    if (graphics_cfg.contains("quality") && graphics_cfg["quality"].is_string()) {
        sf::RenderQuality q = stringToRenderQuality(graphics_cfg["quality"].get<std::string>());
        settings.shadows = q;
        settings.ao = q;
        settings.atmosphere = q;
        settings.ocean = q;
        settings.aa = q;
        settings.ssr = q;
    }

    if (graphics_cfg.contains("shadows") && graphics_cfg["shadows"].is_string()) {
        settings.shadows = stringToRenderQuality(graphics_cfg["shadows"].get<std::string>());
    }
    if (graphics_cfg.contains("ao") && graphics_cfg["ao"].is_string()) {
        settings.ao = stringToRenderQuality(graphics_cfg["ao"].get<std::string>());
    }
    if (graphics_cfg.contains("atmosphere") && graphics_cfg["atmosphere"].is_string()) {
        settings.atmosphere = stringToRenderQuality(graphics_cfg["atmosphere"].get<std::string>());
    }
    if (graphics_cfg.contains("ocean") && graphics_cfg["ocean"].is_string()) {
        settings.ocean = stringToRenderQuality(graphics_cfg["ocean"].get<std::string>());
    }
    if (graphics_cfg.contains("aa") && graphics_cfg["aa"].is_string()) {
        settings.aa = stringToRenderQuality(graphics_cfg["aa"].get<std::string>());
    }
    if (graphics_cfg.contains("ssr") && graphics_cfg["ssr"].is_string()) {
        settings.ssr = stringToRenderQuality(graphics_cfg["ssr"].get<std::string>());
    }

    if (graphics_cfg.contains("vsync") && graphics_cfg["vsync"].is_boolean()) {
        settings.verticalSync = graphics_cfg["vsync"].get<bool>();
    }

    return settings;
}

sf::HelperSettings mapConfigToHelperSettings(const nlohmann::json& graphics_cfg) {
    sf::HelperSettings settings;

    if (!graphics_cfg.contains("helper_settings")) {
        return settings;
    }

    const auto& hs = graphics_cfg["helper_settings"];

    if (hs.contains("show_coord_sys") && hs["show_coord_sys"].is_boolean()) {
        settings.showCoordSys = hs["show_coord_sys"].get<bool>();
    }
    if (hs.contains("show_joints") && hs["show_joints"].is_boolean()) {
        settings.showJoints = hs["show_joints"].get<bool>();
    }
    if (hs.contains("show_actuators") && hs["show_actuators"].is_boolean()) {
        settings.showActuators = hs["show_actuators"].get<bool>();
    }
    if (hs.contains("show_sensors") && hs["show_sensors"].is_boolean()) {
        settings.showSensors = hs["show_sensors"].get<bool>();
    }
    if (hs.contains("show_fluid_dynamics") && hs["show_fluid_dynamics"].is_boolean()) {
        settings.showFluidDynamics = hs["show_fluid_dynamics"].get<bool>();
    }
    if (hs.contains("show_ocean_velocity_field") && hs["show_ocean_velocity_field"].is_boolean()) {
        settings.showOceanVelocityField = hs["show_ocean_velocity_field"].get<bool>();
    }
    if (hs.contains("show_forces") && hs["show_forces"].is_boolean()) {
        settings.showForces = hs["show_forces"].get<bool>();
    }
    if (hs.contains("show_bullet_debug_info") && hs["show_bullet_debug_info"].is_boolean()) {
        settings.showBulletDebugInfo = hs["show_bullet_debug_info"].get<bool>();
    }

    return settings;
}

} // namespace utils
