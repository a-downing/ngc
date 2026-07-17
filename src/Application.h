#pragma once

#include <memory>

#include "machine/MachineConfiguration.h"

struct GLFWwindow;

class ApplicationImpl;

class Application final {
    std::unique_ptr<ApplicationImpl> m_impl;

public:
    Application() = delete;
    Application(GLFWwindow *window, const ngc::MachineConfiguration &configuration);
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
