#pragma once

#include <Stonefish/graphics/OpenGLDataStructs.h>
#include <nlohmann/json.hpp>

namespace utils {

sf::RenderSettings mapConfigToRenderSettings(const nlohmann::json& graphics_cfg);
sf::HelperSettings mapConfigToHelperSettings(const nlohmann::json& graphics_cfg);
sf::RenderQuality stringToRenderQuality(const std::string& quality);

} // namespace utils
