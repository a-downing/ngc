cmake_minimum_required(VERSION 3.28)
project(ngc)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

function(ngc_target_defaults target)
    target_compile_options(${target} PRIVATE -g -O2 -Wall -Wextra)
    target_link_options(${target} PRIVATE -g)
endfunction()

set(IMGUI_DIR imgui)

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
find_package(glm CONFIG REQUIRED)
#find_package(glfw3 REQUIRED)
add_subdirectory(glfw)
add_library(ngc_ruckig STATIC
        src/third_party/ruckig/src/brake.cpp
        src/third_party/ruckig/src/position_first_step1.cpp
        src/third_party/ruckig/src/position_first_step2.cpp
        src/third_party/ruckig/src/position_second_step1.cpp
        src/third_party/ruckig/src/position_second_step2.cpp
        src/third_party/ruckig/src/position_third_step1.cpp
        src/third_party/ruckig/src/position_third_step2.cpp
        src/third_party/ruckig/src/velocity_second_step1.cpp
        src/third_party/ruckig/src/velocity_second_step2.cpp
        src/third_party/ruckig/src/velocity_third_step1.cpp
        src/third_party/ruckig/src/velocity_third_step2.cpp)
target_include_directories(ngc_ruckig PUBLIC src/third_party/ruckig/include)
target_compile_definitions(ngc_ruckig PUBLIC _USE_MATH_DEFINES)
target_compile_options(ngc_ruckig PRIVATE -O2)

set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)
add_subdirectory(tomlplusplus EXCLUDE_FROM_ALL)
unset(BUILD_EXAMPLES)
unset(BUILD_TESTS)

add_library(ngc_core STATIC
        src/ArcInterpolation.cpp
        src/InfiniteJerkTrajectoryTime.cpp
        src/MachineConfiguration.cpp
        src/ExactStopTrajectoryPlanner.cpp
        src/Evaluator.cpp
        src/Machine.cpp
        src/MockMotionBackend.cpp
        src/ToolTable.cpp)
target_include_directories(ngc_core PUBLIC src/include)
target_link_libraries(ngc_core PRIVATE ngc_ruckig tomlplusplus::tomlplusplus)
ngc_target_defaults(ngc_core)

# add_library(utils)
# target_sources(utils PUBLIC FILE_SET CXX_MODULES FILES src/modules/Utils.cppm)

# add_library(parser)
# target_sources(parser PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/parser/LexerSource.cppm
#         src/modules/parser/TokenSource.cppm
#         src/modules/parser/StringTokenSource.cppm
#         src/modules/parser/StringViewTokenSource.cppm
#         src/modules/parser/Token.cppm
#         src/modules/parser/Lexer.cppm
#         src/modules/parser/Visitor.cppm
#         src/modules/parser/Expression.cppm
#         src/modules/parser/Statement.cppm
#         src/modules/parser/SubSignature.cppm
#         src/modules/parser/Parser.cppm)
# target_link_libraries(parser utils)

# add_library(gcode)
# target_sources(gcode PUBLIC FILE_SET CXX_MODULES FILES src/modules/gcode/GCode.cppm)
# target_link_libraries(gcode parser)

# add_library(memory)
# target_sources(memory PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/memory/Vars.cppm
#         src/modules/memory/MemoryCell.cppm
#         src/modules/memory/Memory.cppm)
# target_link_libraries(memory)

# add_library(machine
#         src/modules/machine/ToolTable.cppm)
# target_sources(machine PUBLIC FILE_SET CXX_MODULES FILES src/modules/machine/Machine.cppm src/modules/machine/MachineCommand.cppm src/modules/machine/ToolTable.cppm)
# target_link_libraries(machine utils gcode memory)

# add_library(evaluator)
# target_sources(evaluator PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/evaluator/Preamble.cppm
#         src/modules/evaluator/Program.cppm
#         src/modules/evaluator/Evaluator.cppm)
# target_link_libraries(evaluator parser memory gcode)

# add_executable(ngc main.cpp)
# target_link_libraries(ngc utils parser memory evaluator machine)

add_library(ngc_imgui STATIC
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp)

target_include_directories(ngc_imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
        ${IMGUI_DIR}/misc/cpp)

target_link_libraries(ngc_imgui PUBLIC
        OpenGL::GL
        glfw)

add_executable(imgui_main
        src/imgui_main.cpp
        src/Application.cpp
        src/Application.h)

target_include_directories(imgui_main PRIVATE
        src/
        src/include)

target_link_libraries(imgui_main
        ngc_core
        ngc_imgui
        OpenGL::GL
        OpenGL::GLU
        glm::glm-header-only)
ngc_target_defaults(imgui_main)

add_executable(binary_to_compressed_c ${IMGUI_DIR}/misc/fonts/binary_to_compressed_c.cpp)

add_executable(ngc_tests src/test.cpp)
target_include_directories(ngc_tests PRIVATE src)
target_link_libraries(ngc_tests PRIVATE ngc_core glm::glm-header-only)
ngc_target_defaults(ngc_tests)
add_test(NAME ngc_tests COMMAND ngc_tests)
set_tests_properties(ngc_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(ngc_g64_oracle_export tools/ngc_g64_oracle_export.cpp)
target_include_directories(ngc_g64_oracle_export PRIVATE src)
target_link_libraries(ngc_g64_oracle_export PRIVATE ngc_core)
ngc_target_defaults(ngc_g64_oracle_export)

add_executable(ngc_spline_geometry_analyzer tools/ngc_spline_geometry_analyzer.cpp)
target_link_libraries(ngc_spline_geometry_analyzer PRIVATE ngc_core)
ngc_target_defaults(ngc_spline_geometry_analyzer)
add_test(NAME ngc_spline_geometry_analyzer_test
    COMMAND ngc_spline_geometry_analyzer
        ${CMAKE_SOURCE_DIR}/tools/testdata/spline_geometry_analyzer_snapshot.txt
        --optimize-all 0.005 10.0
        ${CMAKE_CURRENT_BINARY_DIR}/spline_geometry_analyzer_test.csv)
set_tests_properties(ngc_spline_geometry_analyzer_test PROPERTIES
    PASS_REGULAR_EXPRESSION
        "optimized 1 variable-control short-entity splines from 1 captured splines")
add_test(NAME ngc_spline_half_entity_controls_test
    COMMAND ngc_spline_geometry_analyzer
        ${CMAKE_SOURCE_DIR}/tools/testdata/spline_geometry_analyzer_snapshot.txt
        --half-entity-controls
        ${CMAKE_CURRENT_BINARY_DIR}/spline_half_entity_controls_test.csv)
set_tests_properties(ngc_spline_half_entity_controls_test PROPERTIES
    PASS_REGULAR_EXPRESSION
        "reconstructed 1 variable-control short-entity splines with one interior control per two entities")

add_executable(ngc_quintic_spline_analyzer tools/ngc_quintic_spline_analyzer.cpp)
ngc_target_defaults(ngc_quintic_spline_analyzer)
