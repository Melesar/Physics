cmake_minimum_required(VERSION 3.15)
project(Physics C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Platform-specific settings
if(WIN32)
    add_compile_definitions(PLATFORM_DESKTOP)
elseif(APPLE)
    add_compile_definitions(PLATFORM_DESKTOP)
elseif(UNIX)
    add_compile_definitions(PLATFORM_DESKTOP)
endif()

# Build raylib from source (stretch goal)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/raylib)

# Collect all core source files
file(GLOB CORE_SOURCES "core/*.c")

# Collect all scenario source files
file(GLOB SCENARIO_SOURCES "scenarios/*.c")

# Create an executable for each scenario
foreach(SCENARIO_FILE ${SCENARIO_SOURCES})
    # Extract filename without extension
    get_filename_component(SCENARIO_NAME ${SCENARIO_FILE} NAME_WE)
    
    # Create executable
    add_executable(${SCENARIO_NAME} ${SCENARIO_FILE} ${CORE_SOURCES})
    
    # Link raylib
    target_link_libraries(${SCENARIO_NAME} PRIVATE raylib)
    
    # Include directories
    target_include_directories(${SCENARIO_NAME} PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/raylib/src
    )
    
    # Platform-specific linking
    if(WIN32)
        target_link_libraries(${SCENARIO_NAME} PRIVATE winmm)
    elseif(APPLE)
        target_link_libraries(${SCENARIO_NAME} PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
    elseif(UNIX)
        target_link_libraries(${SCENARIO_NAME} PRIVATE m pthread dl rt)
        find_package(X11 REQUIRED)
        target_link_libraries(${SCENARIO_NAME} PRIVATE ${X11_LIBRARIES})
    endif()
    
    # Set output directory
    set_target_properties(${SCENARIO_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endforeach()

# Print summary
message(STATUS "Found ${CMAKE_CURRENT_LIST_LINE} core source files")
message(STATUS "Creating executables for ${CMAKE_CURRENT_LIST_LINE} scenarios")
list(LENGTH CORE_SOURCES CORE_COUNT)
list(LENGTH SCENARIO_SOURCES SCENARIO_COUNT)
message(STATUS "Core files: ${CORE_COUNT}")
message(STATUS "Scenario files: ${SCENARIO_COUNT}")
foreach(SCENARIO_FILE ${SCENARIO_SOURCES})
    get_filename_component(SCENARIO_NAME ${SCENARIO_FILE} NAME_WE)
    message(STATUS "  - ${SCENARIO_NAME}")
endforeach()
