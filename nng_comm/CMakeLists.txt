cmake_minimum_required(VERSION 3.10)
project(nng_comm VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find NNG library
find_package(nng QUIET)
if(NOT nng_FOUND)
    find_path(NNG_INCLUDE_DIR nng/nng.h PATHS /usr/local/include /usr/include)
    find_library(NNG_LIBRARY NAMES nng PATHS /usr/local/lib /usr/lib)
    if(NNG_INCLUDE_DIR AND NNG_LIBRARY)
        message(STATUS "NNG found manually: ${NNG_LIBRARY}")
        add_library(nng::nng UNKNOWN IMPORTED)
        set_target_properties(nng::nng PROPERTIES
            IMPORTED_LOCATION "${NNG_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${NNG_INCLUDE_DIR}"
        )
    else()
        message(FATAL_ERROR "NNG not found")
    endif()
endif()

# Find nlohmann_json library
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

# Build shared library
add_library(nng_comm SHARED
    src/message.cpp
    src/publisher.cpp
    src/subscriber.cpp
    src/requester.cpp
    src/replier.cpp
)

target_include_directories(nng_comm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nng_comm PUBLIC
    nng::nng
    nlohmann_json::nlohmann_json
)

set_target_properties(nng_comm PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Tests
enable_testing()
add_executable(test_nng_comm tests/test_nng_comm.cpp)
target_link_libraries(test_nng_comm nng_comm)
add_test(NAME TestNNGComm COMMAND test_nng_comm)

add_executable(test_cross_lang tests/test_cross_lang.cpp)
target_link_libraries(test_cross_lang nng_comm)
