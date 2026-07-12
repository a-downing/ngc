#pragma once

#include <memory>

struct GLFWwindow;

class ApplicationImpl;

class Application final {
    std::unique_ptr<ApplicationImpl> m_impl;

public:
    Application() = delete;
    explicit Application(GLFWwindow *window);
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void init();
    void preRender();
    void addScroll(double delta);
    void terminate();
    void render();
    void render3D();
};
