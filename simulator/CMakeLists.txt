cmake_minimum_required(VERSION 3.10)
project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(Stonefish REQUIRED)

# nlohmann_json (needed by utils library directly)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp PATHS /usr/local/include /usr/include)
    if(NLOHMANN_JSON_INCLUDE_DIR)
        message(STATUS "nlohmann_json found manually: ${NLOHMANN_JSON_INCLUDE_DIR}")
        add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
        set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}"
        )
    else()
        message(FATAL_ERROR "nlohmann_json not found")
    endif()
endif()

# nng_comm (shared module in demo/nng_comm)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../nng_comm nng_comm_build)

# Utils library
add_library(utils
    src/utils/json_utils.cpp
    src/utils/config_loader.cpp
    src/utils/render_settings_mapper.cpp
)
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(utils nlohmann_json::nlohmann_json Stonefish::Stonefish)

# Sim bridge library
add_library(sim_bridge
    src/sim_bridge/sensor_bridge.cpp
    src/sim_bridge/actuator_bridge.cpp
    src/sim_bridge/message_converter.cpp
)
target_include_directories(sim_bridge PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(sim_bridge nng_comm Stonefish::Stonefish)

# Main executable
add_executable(simulator
    src/main.cpp
    src/simulation_manager.cpp
    src/graphical_app.cpp
)
target_include_directories(simulator PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(simulator sim_bridge utils nng_comm Stonefish::Stonefish)

# Testing
enable_testing()

add_executable(test_message_converter tests/test_message_converter.cpp)
target_link_libraries(test_message_converter sim_bridge)
add_test(NAME TestMessageConverter COMMAND test_message_converter)
