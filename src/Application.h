#pragma once

#include <memory>

#include "machine/MachineConfiguration.h"
#include "machine/SplineReconstruction.h"
#include "machine/TrajectoryCompiler.h"

struct GLFWwindow;

class ApplicationImpl;

class Application final {
    std::unique_ptr<ApplicationImpl> m_impl;

public:
    Application() = delete;
    Application(GLFWwindow *window, const ngc::MachineConfiguration &configuration,
                ngc::spline_detail::SplineFitSolver splineFitSolver,
                ngc::ContinuousConstraintCheckMode continuousCheckMode);
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void init();
    void preRender();
    void setLastFrameRenderSeconds(double seconds);
    void addScroll(double delta);
    void terminate();
    void render();
    void render3D();
};
