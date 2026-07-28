cmake_minimum_required(VERSION 3.28)
project(ngc)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

function(ngc_target_defaults target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra)
    endif()
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

add_library(ngc_configuration STATIC
        src/TomlConfiguration.cpp)
target_include_directories(ngc_configuration PUBLIC src/include)
target_link_libraries(
        ngc_configuration PUBLIC tomlplusplus::tomlplusplus)
ngc_target_defaults(ngc_configuration)

add_library(ngc_core STATIC
        src/ArcInterpolation.cpp
        src/DigitalIoProgram.cpp
        src/ExecutionItemOperations.cpp
        src/ExternalRealtimeRuntime.cpp
        src/HomingController.cpp
        src/InfiniteJerkTrajectoryTime.cpp
        src/InProcessSimulationRuntime.cpp
        src/IpcExecutorBridge.cpp
        src/IpcPlatform.cpp
        src/JoggingController.cpp
        src/MachineSession.cpp
        src/MachineConfiguration.cpp
        src/ProgramExecutionController.cpp
        src/ProductionExecutorCore.cpp
        src/ProductionExecutorRuntime.cpp
        src/RealtimeHost.cpp
        src/PreparedGeometry.cpp
        src/SplineReconstruction.cpp
        src/TrajectoryCompiler.cpp
        src/Evaluator.cpp
        src/Machine.cpp
        src/MockMotionBackend.cpp
        src/ParameterStore.cpp
        src/PhysicalProductionExecutorIo.cpp
        src/PresentationTracker.cpp
        src/SpindleHardware.cpp
        src/ToolTable.cpp)
target_include_directories(ngc_core PUBLIC src/include)
target_link_libraries(ngc_core
    PUBLIC ruckig::ruckig
    PRIVATE PathTempo::PathTempo ngc_configuration)
ngc_target_defaults(ngc_core)

add_library(ngc_mesa STATIC
        src/mesa/HostMot2CyclicIo.cpp
        src/mesa/HostMot2Discovery.cpp
        src/mesa/HostMot2Latency.cpp
        src/mesa/Lbp16CyclicTransaction.cpp
        src/mesa/Lbp16UdpTransport.cpp
        src/mesa/SevenI96Capabilities.cpp
        src/mesa/SevenI96CyclicLayout.cpp)
target_include_directories(ngc_mesa PUBLIC src/include)
if(WIN32)
    target_link_libraries(ngc_mesa PRIVATE ws2_32)
endif()
ngc_target_defaults(ngc_mesa)

add_library(ngc_mesa_executor STATIC
        src/mesa/MesaProductionExecutorIo.cpp)
target_include_directories(ngc_mesa_executor PUBLIC src/include)
target_link_libraries(ngc_mesa_executor PUBLIC ngc_core ngc_mesa)
ngc_target_defaults(ngc_mesa_executor)

add_library(ngc_mesa_configuration STATIC
        src/mesa/MesaBackendConfiguration.cpp)
target_include_directories(
        ngc_mesa_configuration PUBLIC src/include)
target_link_libraries(
        ngc_mesa_configuration
        PUBLIC ngc_mesa ngc_configuration)
ngc_target_defaults(ngc_mesa_configuration)

add_library(ngc_physical_configuration STATIC
        src/physical/PhysicalBackendConfiguration.cpp)
target_include_directories(
        ngc_physical_configuration PUBLIC src/include)
target_link_libraries(
        ngc_physical_configuration
        PUBLIC ngc_mesa_configuration ngc_configuration)
ngc_target_defaults(ngc_physical_configuration)

add_library(ngc_spindle_huanyang STATIC
        src/physical/HuanyangSpindleHardware.cpp
        src/physical/SerialTransport.cpp)
target_include_directories(
        ngc_spindle_huanyang PUBLIC src/include)
target_link_libraries(
        ngc_spindle_huanyang
        PUBLIC ngc_core ngc_physical_configuration)
ngc_target_defaults(ngc_spindle_huanyang)

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

add_executable(ngc_tests
    src/BackendConformanceTests.cpp
    src/test.cpp
)
target_include_directories(ngc_tests PRIVATE src)
target_link_libraries(ngc_tests PRIVATE ngc_core glm::glm-header-only PathTempo::PathTempo)
ngc_target_defaults(ngc_tests)
add_test(NAME ngc_tests COMMAND ngc_tests)

add_executable(ngc_production_executor_tests
    src/ProductionExecutorCoreTests.cpp
)
target_link_libraries(ngc_production_executor_tests PRIVATE ngc_core)
ngc_target_defaults(ngc_production_executor_tests)
add_test(NAME ngc_production_executor_tests COMMAND ngc_production_executor_tests)

add_executable(ngc_mesa_tests src/MesaHostMot2Tests.cpp)
target_compile_definitions(
    ngc_mesa_tests PRIVATE NGC_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(
    ngc_mesa_tests PRIVATE
    ngc_mesa_executor ngc_mesa_configuration)
ngc_target_defaults(ngc_mesa_tests)
add_test(NAME ngc_mesa_tests COMMAND ngc_mesa_tests)

add_executable(
    ngc_physical_backend_tests
    src/PhysicalBackendTests.cpp)
target_compile_definitions(
    ngc_physical_backend_tests
    PRIVATE NGC_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(
    ngc_physical_backend_tests
    PRIVATE ngc_spindle_huanyang)
ngc_target_defaults(ngc_physical_backend_tests)
add_test(
    NAME ngc_physical_backend_tests
    COMMAND ngc_physical_backend_tests)

add_executable(ngc_ipc_backend tools/ngc_ipc_backend.cpp)
target_include_directories(ngc_ipc_backend PRIVATE src)
target_link_libraries(ngc_ipc_backend PRIVATE ngc_core)
ngc_target_defaults(ngc_ipc_backend)
if(NOT WIN32)
    set_target_properties(ngc_ipc_backend PROPERTIES SUFFIX ".exe")
endif()

add_executable(ngc_ipc_tests src/IpcTests.cpp)
target_include_directories(ngc_ipc_tests PRIVATE src)
target_link_libraries(ngc_ipc_tests PRIVATE ngc_core)
ngc_target_defaults(ngc_ipc_tests)
add_test(NAME ngc_ipc_tests COMMAND ngc_ipc_tests $<TARGET_FILE:ngc_ipc_backend>)
set_tests_properties(ngc_ipc_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(ngc_simulation_diagnostic tools/ngc_simulation_diagnostic.cpp)
target_include_directories(ngc_simulation_diagnostic PRIVATE src)
target_link_libraries(ngc_simulation_diagnostic PRIVATE ngc_core)
ngc_target_defaults(ngc_simulation_diagnostic)

add_executable(ngc_mesa_discover tools/ngc_mesa_discover.cpp)
target_link_libraries(ngc_mesa_discover PRIVATE ngc_mesa)
ngc_target_defaults(ngc_mesa_discover)

add_executable(ngc_mesa_latency tools/ngc_mesa_latency.cpp)
target_link_libraries(ngc_mesa_latency PRIVATE ngc_mesa)
ngc_target_defaults(ngc_mesa_latency)

add_executable(
    ngc_mesa_stepgen_diagnostic
    tools/ngc_mesa_stepgen_diagnostic.cpp)
target_link_libraries(
    ngc_mesa_stepgen_diagnostic
    PRIVATE ngc_core ngc_mesa_configuration)
ngc_target_defaults(ngc_mesa_stepgen_diagnostic)

add_executable(ngc_mesa_backend tools/ngc_mesa_backend.cpp)
target_include_directories(ngc_mesa_backend PRIVATE src)
target_link_libraries(
    ngc_mesa_backend
    PRIVATE ngc_mesa_executor ngc_spindle_huanyang)
ngc_target_defaults(ngc_mesa_backend)
add_test(
    NAME ngc_mesa_backend_configuration
    COMMAND ngc_mesa_backend
        --machine-configuration ${CMAKE_SOURCE_DIR}/machine.toml
        --backend-configuration
            ${CMAKE_SOURCE_DIR}/physical_backend.toml
        --validate-config-only)

add_executable(
    ngc_mesa_backend_smoke
    tools/ngc_mesa_backend_smoke.cpp)
target_link_libraries(
    ngc_mesa_backend_smoke PRIVATE ngc_core)
ngc_target_defaults(ngc_mesa_backend_smoke)

add_executable(ngc_g64_geometry_export tools/ngc_g64_geometry_export.cpp)
target_include_directories(ngc_g64_geometry_export PRIVATE src)
target_link_libraries(ngc_g64_geometry_export PRIVATE ngc_core)
ngc_target_defaults(ngc_g64_geometry_export)

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
