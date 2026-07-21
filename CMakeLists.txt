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

add_subdirectory(src/pendant)

set(IMGUI_DIR imgui)

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
find_package(glm CONFIG REQUIRED)
add_subdirectory(glfw)

set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)
add_subdirectory(tomlplusplus EXCLUDE_FROM_ALL)
unset(BUILD_EXAMPLES)
unset(BUILD_TESTS)

add_subdirectory(path_tempo)

add_library(ngc_core STATIC
        src/ArcInterpolation.cpp
        src/InfiniteJerkTrajectoryTime.cpp
        src/MachineConfiguration.cpp
        src/PreparedGeometry.cpp
        src/SplineReconstruction.cpp
        src/TrajectoryCompiler.cpp
        src/Evaluator.cpp
        src/Machine.cpp
        src/MockMotionBackend.cpp
        src/ToolTable.cpp)
target_include_directories(ngc_core PUBLIC src/include)
target_link_libraries(ngc_core PRIVATE PathTempo::PathTempo ruckig::ruckig tomlplusplus::tomlplusplus)
ngc_target_defaults(ngc_core)

add_library(ngc_operator STATIC
        src/operator/OperatorJogController.cpp
        src/operator/PendantTouchOffController.cpp)
target_include_directories(ngc_operator PUBLIC src/include)
target_link_libraries(ngc_operator PUBLIC ngc_core ngc_pendant)
ngc_target_defaults(ngc_operator)

add_executable(ngc_operator_tests src/operator/test.cpp)
target_link_libraries(ngc_operator_tests PRIVATE ngc_operator)
ngc_target_defaults(ngc_operator_tests)
add_test(NAME ngc_operator_tests COMMAND ngc_operator_tests)
set_tests_properties(ngc_operator_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_library(ngc_imgui STATIC
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
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
        ngc_operator
        ngc_imgui
        OpenGL::GL
        OpenGL::GLU
        glm::glm-header-only)
ngc_target_defaults(imgui_main)

add_executable(ngc_tests src/test.cpp)
target_include_directories(ngc_tests PRIVATE src)
target_link_libraries(ngc_tests PRIVATE ngc_core glm::glm-header-only)
ngc_target_defaults(ngc_tests)
add_test(NAME ngc_tests COMMAND ngc_tests)

add_executable(ngc_simulation_diagnostic tools/ngc_simulation_diagnostic.cpp)
target_include_directories(ngc_simulation_diagnostic PRIVATE src)
target_link_libraries(ngc_simulation_diagnostic PRIVATE ngc_core)
ngc_target_defaults(ngc_simulation_diagnostic)

add_executable(vistacnc_p2s_probe tools/vistacnc_p2s_probe.cpp)
ngc_target_defaults(vistacnc_p2s_probe)
target_link_libraries(vistacnc_p2s_probe PRIVATE ngc_pendant)

add_executable(ngc_spline_geometry_analyzer tools/ngc_spline_geometry_analyzer.cpp)
target_link_libraries(ngc_spline_geometry_analyzer PRIVATE ngc_core)
ngc_target_defaults(ngc_spline_geometry_analyzer)
add_test(NAME ngc_spline_geometry_analyzer_test
    COMMAND ngc_spline_geometry_analyzer
        --self-test-fixture
        --optimize-all 0.005 10.0
        ${CMAKE_CURRENT_BINARY_DIR}/spline_geometry_analyzer_test.csv)
set_tests_properties(ngc_spline_geometry_analyzer_test PROPERTIES
    PASS_REGULAR_EXPRESSION
        "optimized 1 variable-control short-entity splines from 1 captured splines")
add_test(NAME ngc_spline_half_entity_controls_test
    COMMAND ngc_spline_geometry_analyzer
        --self-test-fixture
        --half-entity-controls
        ${CMAKE_CURRENT_BINARY_DIR}/spline_half_entity_controls_test.csv)
set_tests_properties(ngc_spline_half_entity_controls_test PROPERTIES
    PASS_REGULAR_EXPRESSION
        "reconstructed 1 variable-control short-entity splines with one interior control per two entities")

add_executable(ngc_quintic_spline_analyzer tools/ngc_quintic_spline_analyzer.cpp)
ngc_target_defaults(ngc_quintic_spline_analyzer)
