#include <chrono>
#include <cstdio>
#include <expected>
#include <fstream>
#include <print>
#include <string>
#include <string_view>
#include <utility>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Application.h"
#include "machine/MachineConfiguration.h"

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

#include <GLFW/glfw3.h>

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static double scroll_delta = 0.0;

static void application_scroll_callback(GLFWwindow *, double, double yoffset) {
    scroll_delta += yoffset;
}

static bool load_window_maximized() {
    std::ifstream state("window_state.ini");
    std::string value;
    return std::getline(state, value) && value == "maximized=1";
}

static void save_window_maximized(GLFWwindow *window) {
    std::ofstream state("window_state.ini", std::ios::trunc);
    state << "maximized=" << (glfwGetWindowAttrib(window, GLFW_MAXIMIZED) == GLFW_TRUE ? 1 : 0) << '\n';
}

struct CommandLineOptions {
    ngc::spline_detail::SplineFitSolver smoother =
        ngc::spline_detail::continuousSplineFitSolver();
    ngc::ContinuousBoundaryAccelerationMode mode =
        ngc::ContinuousBoundaryAccelerationMode::Optimized;
    ngc::ContinuousConstraintCheckMode continuousCheck =
        ngc::ContinuousConstraintCheckMode::Materialized;
};

static std::expected<CommandLineOptions, std::string> parseOptions(const int argc, char **argv) {
    CommandLineOptions result;
    for (auto argument = 1; argument < argc; ++argument) {
        auto option = std::string_view{argv[argument]};
        constexpr std::string_view smootherPrefix = "--smoother=";
        constexpr std::string_view modePrefix = "--mode=";
        constexpr std::string_view checkPrefix = "--continuous-check=";
        if (option.starts_with(smootherPrefix)) {
            option.remove_prefix(smootherPrefix.size());
            using ngc::spline_detail::SplineFitSolver;
            if (option == "none") {
                result.smoother = SplineFitSolver::None;
            } else if (option == "coordinate") {
                result.smoother = SplineFitSolver::CoordinateSearch;
            } else if (option == "uniform") {
                result.smoother = SplineFitSolver::UniformBandedFairness;
            } else if (option == "peak-targeted") {
                result.smoother = SplineFitSolver::PeakTargetedBandedFairness;
            } else if (option == "velocity-targeted") {
                result.smoother = SplineFitSolver::VelocityTargetedBandedFairness;
            } else {
                return std::unexpected(
                    "unknown smoother; expected none, coordinate, uniform, peak-targeted, or "
                    "velocity-targeted");
            }
        } else if (option == "--mode" || option.starts_with(modePrefix)) {
            if (option == "--mode") {
                if (++argument == argc) {
                    return std::unexpected("mode option requires zero or optimized");
                }
                option = argv[argument];
            } else {
                option.remove_prefix(modePrefix.size());
            }
            if (option == "zero") {
                result.mode = ngc::ContinuousBoundaryAccelerationMode::Zero;
            } else if (option == "optimized") {
                result.mode = ngc::ContinuousBoundaryAccelerationMode::Optimized;
            } else {
                return std::unexpected("unknown mode; expected zero or optimized");
            }
        } else if (option.starts_with(checkPrefix)) {
            option.remove_prefix(checkPrefix.size());
            if (option == "materialized") {
                result.continuousCheck = ngc::ContinuousConstraintCheckMode::Materialized;
            } else if (option == "sampled") {
                result.continuousCheck = ngc::ContinuousConstraintCheckMode::Sampled;
            } else {
                return std::unexpected(
                    "unknown continuous check; expected materialized or sampled");
            }
        } else {
            return std::unexpected("unknown option '" + std::string{option} + "'");
        }
    }

    return result;
}

int main(const int argc, char **argv) {
    const auto options = parseOptions(argc, argv);
    if (!options) {
        std::println(stderr, "{}", options.error());
        std::println(stderr, "usage: imgui_main [--smoother=none|coordinate|uniform|"
            "peak-targeted|velocity-targeted] "
            "[--mode=zero|optimized] "
            "[--continuous-check=materialized|sampled]");
        return 2;
    }
    const auto configuration = ngc::loadMachineConfiguration("machine.toml");
    if(!configuration) {
        std::println(stderr, "Failed to load machine configuration: {}", configuration.error());
        return 1;
    }

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
        return 1;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    // Create window with graphics context
    glfwWindowHint(GLFW_MAXIMIZED, load_window_maximized() ? GLFW_TRUE : GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "ngc", nullptr, nullptr);

    if (window == nullptr) {
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetScrollCallback(window, application_scroll_callback);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    Application app(window, *configuration, options->smoother, options->mode,
                    options->continuousCheck);
    app.init();

    while (!glfwWindowShouldClose(window)) {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();
        const auto renderStart=std::chrono::steady_clock::now();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        app.addScroll(std::exchange(scroll_delta, 0.0));
        app.preRender();
        app.render();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        app.render3D();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        const auto renderEnd=std::chrono::steady_clock::now();
        app.setLastFrameRenderSeconds(
            std::chrono::duration<double>(renderEnd-renderStart).count());

        glfwSwapBuffers(window);
    }

    save_window_maximized(window);
    app.terminate();

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
