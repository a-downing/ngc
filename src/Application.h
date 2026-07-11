#pragma once

#include <filesystem>
#include <algorithm>
#include <print>
#include <string>
#include <cmath>
#include <cstdint>
#include <ranges>
#include <numbers>
#include <limits>
#include <optional>
#include <utility>

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/ext/quaternion_common.hpp>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_stdlib.h"
#include "gui/imgui_custom.h"

#include "utils.h"
#include "SimulationWorker.h"
#include "Worker.h"

#include "machine/ToolTable.h"
#include "machine/MachineCommand.h"

#include "gui/tool_table_strings_t.h"

class Application final {
    GLFWwindow *m_window;
    std::filesystem::path m_path = std::filesystem::absolute(".").lexically_normal();
    ngc::ToolTable m_tools;
    std::vector<tool_table_strings_t> m_toolStrings;
    std::vector<std::tuple<std::string, std::string>> m_programSource;

    // windows
    bool m_enableOpenDialog = false;
    bool m_enableProgramWindow = false;
    bool m_enableMemoryWindow = false;
    bool m_enableToolWindow = false;
    bool m_enableErrorWindow = false;
    bool m_enableMessagesWindow = false;

    std::string m_errorMessage;

    Worker m_worker{};
    SimulationWorker m_simulation{};
    double m_simulationRate = 1.0;
    double m_simulatedRapidSpeed = 100.0;

    double m_time = 0.0;
    double m_dt = 0.0;
    double m_mouseX = 0.0;
    double m_mouseY = 0.0;
    double m_mouseDX = 0.0;
    double m_mouseDY = 0.0;
    double m_scrollDelta = 0.0;

    glm::dquat m_modelOrientation { 1.0, 0.0, 0.0, 0.0 };
    glm::dvec2 m_viewPan { 0.0, 0.0 };
    double m_viewHalfHeight = 6.0;
    double m_sceneScale = 1.0;
    glm::dvec3 m_modelPivot = { 0.0, 0.0, 0.0 };

    static constexpr double MACHINE_TRIAD_LENGTH = 2.0;
    static constexpr double WORK_TRIAD_LENGTH = 1.0;

    static glm::dvec3 displayOffset(const glm::dvec3 &offset) {
        return { offset.x, -offset.y, offset.z };
    }

    void drawToolWireframe(const ngc::ToolPose &tool) const {
        const glm::dvec3 top {
            tool.spindlePosition.x,
            tool.spindlePosition.y,
            tool.spindlePosition.z,
        };
        const glm::dvec3 tip {
            tool.tipPosition.x,
            tool.tipPosition.y,
            tool.tipPosition.z,
        };
        const auto axis = tip - top;
        if(tool.geometry.number == 0 || glm::length2(axis) < 1e-18) return;

        const auto axisUnit = glm::normalize(axis);
        const auto cameraDirection = displayOffset(glm::inverse(m_modelOrientation) * glm::dvec3(0.0, 1.0, 0.0));
        auto radial = glm::cross(axisUnit, cameraDirection);
        if(glm::length2(radial) < 1e-18) {
            radial = glm::cross(axisUnit, glm::dvec3(1.0, 0.0, 0.0));
        }
        radial = glm::normalize(radial);
        const auto radius = std::max(tool.geometry.diameter * 0.5, 0.001);
        const auto edge0 = top + radial * radius;
        const auto edge1 = top - radial * radius;

        auto circleBasis0 = glm::cross(axisUnit, glm::dvec3(0.0, 0.0, 1.0));
        if(glm::length2(circleBasis0) < 1e-18) {
            circleBasis0 = glm::cross(axisUnit, glm::dvec3(0.0, 1.0, 0.0));
        }
        circleBasis0 = glm::normalize(circleBasis0);
        const auto circleBasis1 = glm::normalize(glm::cross(axisUnit, circleBasis0));

        glDisable(GL_DEPTH_TEST);
        glLineWidth(2.0f);
        glColor3f(1.0f, 0.85f, 0.05f);
        glBegin(GL_LINES);
        glVertex3dv(glm::value_ptr(edge0));
        glVertex3dv(glm::value_ptr(tip));
        glVertex3dv(glm::value_ptr(edge1));
        glVertex3dv(glm::value_ptr(tip));
        glEnd();

        constexpr int CIRCLE_SEGMENTS = 48;
        glBegin(GL_LINE_LOOP);
        for(int segment = 0; segment < CIRCLE_SEGMENTS; segment++) {
            const auto angle = 2.0 * std::numbers::pi * static_cast<double>(segment) / CIRCLE_SEGMENTS;
            const auto point = top + radius * (std::cos(angle) * circleBasis0 + std::sin(angle) * circleBasis1);
            glVertex3dv(glm::value_ptr(point));
        }
        glEnd();

        glEnable(GL_POINT_SMOOTH);
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        glVertex3dv(glm::value_ptr(top));
        glEnd();
        glPointSize(1.0f);
        glLineWidth(1.0f);
    }

public:
    Application() = delete;
    explicit Application(GLFWwindow *window) : m_window(window) { }

    void init() {
        auto result = m_tools.load();

        if(!result) {
            m_errorMessage = "failed to load tool table";
            m_enableErrorWindow = true;
        }
    }

    void preRender() {
        m_dt = ImGui::GetTime() - m_time;
        m_time = ImGui::GetTime();
        auto pos = ImGui::GetMousePos();
        m_mouseDX = pos.x - m_mouseX;
        m_mouseDY = pos.y - m_mouseY;
        m_mouseX = pos.x;
        m_mouseY = pos.y;
    }

    void addScroll(const double delta) {
        m_scrollDelta += delta;
    }

    void terminate() {
        m_simulation.join();
        m_worker.join();
    }

    void initToolTableStrings() {
        m_toolStrings.clear();

        for(const auto &[num, tool] : m_tools) {
            m_toolStrings.emplace_back(tool_table_strings_t::from(tool));
        }
    }

    void saveToolTableStrings() {
        for(const auto &toolStrings : m_toolStrings) {
            auto tool = toolStrings.to();

            if(!tool) {
                m_errorMessage = std::format("tool #{}: {}", toolStrings.number, tool.error());
                m_enableErrorWindow = true;
                return;
            }

            m_tools.set(tool->number, *tool);
        }

        auto result = m_tools.save();

        if(!result) {
            m_errorMessage = result.error();
            m_enableErrorWindow = true;
            return;
        }

        if(!m_worker.setToolTable(m_tools)) {
            m_errorMessage = "tool table was saved, but cannot be applied while the worker is busy";
            m_enableErrorWindow = true;
            return;
        }

        initToolTableStrings();
    }

    void reloadToolTable() {
        const auto result = m_tools.load();

        if(!result) {
            m_errorMessage = result.error();
            m_enableErrorWindow = true;
            return;
        }

        if(!m_worker.setToolTable(m_tools)) {
            m_errorMessage = "tool table cannot be reloaded while the worker is busy";
            m_enableErrorWindow = true;
            return;
        }

        initToolTableStrings();
    }

    bool noWindowHasFocus() {
        auto ctx = ImGui::GetCurrentContext();
        return ctx->NavWindow == nullptr && ctx->MovingWindow == nullptr && ctx->WheelingWindow == nullptr;
    }

    void fitToolpathToView(const int width, const int height) {
        glm::dvec3 minimum(std::numeric_limits<double>::max());
        glm::dvec3 maximum(std::numeric_limits<double>::lowest());
        bool hasGeometry = false;

        const auto include = [&](const glm::dvec3 &point) {
            minimum = glm::min(minimum, point);
            maximum = glm::max(maximum, point);
            hasGeometry = true;
        };

        m_worker.lock([&] {
            m_worker.toolpath().foreachCommand([&](const ngc::MachineCommand &command) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;

                    if constexpr(std::same_as<T, ngc::MoveLine>) {
                        include({ value.from().x, value.from().y, value.from().z });
                        include({ value.to().x, value.to().y, value.to().z });
                    } else if constexpr(std::same_as<T, ngc::ProbeMove>) {
                        include({ value.from().x, value.from().y, value.from().z });
                        include({ value.target().x, value.target().y, value.target().z });
                    } else if constexpr(std::same_as<T, ngc::MoveArc>) {
                        const glm::dvec3 from { value.from().x, value.from().y, value.from().z };
                        const glm::dvec3 to { value.to().x, value.to().y, value.to().z };
                        const glm::dvec3 center { value.center().x, value.center().y, value.center().z };
                        const auto radius = std::max(glm::length(from - center), glm::length(to - center));
                        include(from);
                        include(to);
                        include(center - glm::dvec3(radius));
                        include(center + glm::dvec3(radius));
                    }
                }, command);
            });
        });

        if(!hasGeometry) {
            minimum = { -1.0, -1.0, -1.0 };
            maximum = { 1.0, 1.0, 1.0 };
        }

        m_modelPivot = (minimum + maximum) * 0.5;
        m_viewPan = { 0.0, 0.0 };
        const auto radius = std::max(glm::length(maximum - minimum) * 0.5, 0.1);
        m_sceneScale = radius;
        const auto aspect = static_cast<double>(std::max(width, 1)) / std::max(height, 1);
        m_viewHalfHeight = radius * 1.1 / std::min(aspect, 1.0);
    }

    void pickRotationPivot(const int width, const int height) {
        constexpr double PICK_RADIUS_PIXELS = 30.0;
        const auto aspect = static_cast<double>(width) / height;
        const glm::dvec2 mouse { m_mouseX, m_mouseY };
        auto bestDistanceSquared = PICK_RADIUS_PIXELS * PICK_RADIUS_PIXELS;
        std::optional<glm::dvec3> bestPoint;

        const auto project = [&](const glm::dvec3 &point) {
            const auto transformed = m_modelOrientation * displayOffset(point - m_modelPivot);
            const auto ndcX = (transformed.x + m_viewPan.x) / (m_viewHalfHeight * aspect);
            const auto ndcY = (transformed.z + m_viewPan.y) / m_viewHalfHeight;
            return glm::dvec2((ndcX + 1.0) * width * 0.5, (1.0 - ndcY) * height * 0.5);
        };

        const auto segmentHit = [&](const glm::dvec3 &from, const glm::dvec3 &to) {
            const auto screenFrom = project(from);
            const auto screenTo = project(to);
            const auto screenSegment = screenTo - screenFrom;
            const auto lengthSquared = glm::dot(screenSegment, screenSegment);
            const auto t = lengthSquared > 0.0
                ? std::clamp(glm::dot(mouse - screenFrom, screenSegment) / lengthSquared, 0.0, 1.0)
                : 0.0;
            const auto difference = mouse - glm::mix(screenFrom, screenTo, t);
            return std::pair { glm::dot(difference, difference), t };
        };

        const auto considerSegment = [&](const glm::dvec3 &from, const glm::dvec3 &to) {
            const auto [distanceSquared, t] = segmentHit(from, to);

            if(distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestPoint = glm::mix(from, to, t);
            }
        };

        const auto considerTriadAxis = [&](const glm::dvec3 &origin, const glm::dvec3 &axisEnd) {
            const auto [distanceSquared, _] = segmentHit(origin, axisEnd);

            // Triads are drawn over the toolpath, so give them priority when
            // their projected geometry is equally close to the cursor.
            if(distanceSquared <= bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestPoint = origin;
            }
        };

        m_worker.lock([&] {
            m_worker.toolpath().foreachCommand([&](const ngc::MachineCommand &command) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;

                    if constexpr(std::same_as<T, ngc::MoveLine>) {
                        considerSegment(
                            { value.from().x, value.from().y, value.from().z },
                            { value.to().x, value.to().y, value.to().z });
                    } else if constexpr(std::same_as<T, ngc::ProbeMove>) {
                        considerSegment(
                            { value.from().x, value.from().y, value.from().z },
                            { value.target().x, value.target().y, value.target().z });
                    } else if constexpr(std::same_as<T, ngc::MoveArc>) {
                        interpolate(value, 60, [&](const glm::dvec3 &from, const glm::dvec3 &to, bool, bool) {
                            considerSegment(from, to);
                        });
                    }
                }, command);
            });

            const glm::dvec3 machineOrigin { 0.0, 0.0, 0.0 };
            const auto &workOffset = m_worker.machine().workOffset();
            const glm::dvec3 workOrigin { workOffset.x, workOffset.y, workOffset.z };

            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(MACHINE_TRIAD_LENGTH, 0.0, 0.0));
            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(0.0, MACHINE_TRIAD_LENGTH, 0.0));
            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(0.0, 0.0, MACHINE_TRIAD_LENGTH));
            considerTriadAxis(workOrigin, workOrigin + glm::dvec3(WORK_TRIAD_LENGTH, 0.0, 0.0));
            considerTriadAxis(workOrigin, workOrigin + glm::dvec3(0.0, WORK_TRIAD_LENGTH, 0.0));
            considerTriadAxis(workOrigin, workOrigin + glm::dvec3(0.0, 0.0, WORK_TRIAD_LENGTH));
        });

        if(bestPoint) {
            // Preserve the current model transform while changing the point
            // about which subsequent rotations are applied.
            const auto compensation = m_modelOrientation * displayOffset(*bestPoint - m_modelPivot);
            m_viewPan.x += compensation.x;
            m_viewPan.y += compensation.z;
            m_modelPivot = *bestPoint;
        }
    }

    void render3D() {
        int width, height;
        glfwGetWindowSize(m_window, &width, &height);
        width = std::max(width, 1);
        height = std::max(height, 1);

        const auto navigationActive = noWindowHasFocus();
        const auto middleDown = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
        const auto ctrl = ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl);
        const auto shift = ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift);

        if(navigationActive && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            fitToolpathToView(width, height);
        }

        if(navigationActive && ImGui::IsMouseClicked(ImGuiMouseButton_Middle) && !ctrl && !shift) {
            pickRotationPivot(width, height);
        }

        if(navigationActive && middleDown && !ctrl && !shift) {
            const auto horizontal = glm::angleAxis(-m_mouseDX * 0.01, glm::dvec3(0.0, 0.0, 1.0));
            const auto vertical = glm::angleAxis(-m_mouseDY * 0.01, glm::dvec3(1.0, 0.0, 0.0));
            m_modelOrientation = glm::normalize(vertical * horizontal * m_modelOrientation);
        }

        if(navigationActive && middleDown && ctrl) {
            const auto worldUnitsPerPixel = 2.0 * m_viewHalfHeight / height;
            m_viewPan.x += m_mouseDX * worldUnitsPerPixel;
            m_viewPan.y -= m_mouseDY * worldUnitsPerPixel;
        }

        const auto scrollDelta = std::exchange(m_scrollDelta, 0.0);
        auto scaleFactor = 1.0;
        if(navigationActive && middleDown && shift) {
            scaleFactor *= std::exp(-m_mouseDY * 0.01);
        }
        if(scrollDelta != 0.0) {
            scaleFactor *= std::exp(scrollDelta * 0.15);
        }

        const auto aspect = static_cast<double>(width) / height;
        if(scaleFactor != 1.0) {
            const auto oldHalfHeight = m_viewHalfHeight;
            const auto newHalfHeight = std::clamp(oldHalfHeight * scaleFactor, m_sceneScale * 1e-9, m_sceneScale * 1e9);
            const auto mouseNdcX = 2.0 * m_mouseX / width - 1.0;
            const auto mouseNdcY = 1.0 - 2.0 * m_mouseY / height;

            // Changing an orthographic projection normally zooms around the
            // screen center. Shift the view so the point under the cursor
            // remains under the cursor as the projection scale changes.
            m_viewPan.x += mouseNdcX * aspect * (newHalfHeight - oldHalfHeight);
            m_viewPan.y += mouseNdcY * (newHalfHeight - oldHalfHeight);
            m_viewHalfHeight = newHalfHeight;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-m_viewHalfHeight * aspect, m_viewHalfHeight * aspect,
                -m_viewHalfHeight, m_viewHalfHeight, -m_sceneScale * 1e6, m_sceneScale * 1e6);

        const auto view = glm::lookAt(glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(0.0), glm::dvec3(0.0, 0.0, 1.0));
        auto model = glm::translate(glm::dmat4(1.0), glm::dvec3(m_viewPan.x, 0.0, m_viewPan.y));
        model *= glm::mat4_cast(m_modelOrientation);
        model = glm::scale(model, glm::dvec3(1.0, -1.0, 1.0));
        model = glm::translate(model, -m_modelPivot);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixd(glm::value_ptr(view * model));

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

        struct CoordinateDisplay {
            ngc::position_t workOffset;
            std::string workCoordinateName;
        };

        const auto coordinates = m_worker.lock([&] {
            const auto &machine = m_worker.machine();
            return CoordinateDisplay {
                .workOffset = machine.workOffset(),
                .workCoordinateName = std::string(ngc::name(*machine.state().modeCoordSys)),
            };
        });

        const auto drawTriad = [](const glm::dvec3 &origin, const double length, const bool dashed) {
            if(dashed) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(1, 0x3333);
            }

            const auto intensity = dashed ? 1.0f : 0.55f;
            glBegin(GL_LINES);
            glColor3f(intensity, 0.2f * intensity, 0.2f * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x + length, origin.y, origin.z);
            glColor3f(0.2f * intensity, intensity, 0.2f * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y + length, origin.z);
            glColor3f(0.25f * intensity, 0.45f * intensity, intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y, origin.z + length);
            glEnd();

            if(dashed) {
                glDisable(GL_LINE_STIPPLE);
            }

            glEnable(GL_POINT_SMOOTH);
            glPointSize(dashed ? 7.0f : 9.0f);
            glBegin(GL_POINTS);
            glColor3f(dashed ? 1.0f : 0.95f, dashed ? 0.75f : 0.95f, dashed ? 0.15f : 0.95f);
            glVertex3d(origin.x, origin.y, origin.z);
            glEnd();
        };

        glLineWidth(2.0f);
        drawTriad({ 0.0, 0.0, 0.0 }, MACHINE_TRIAD_LENGTH, false);
        glLineWidth(5.0f);
        drawTriad({ coordinates.workOffset.x, coordinates.workOffset.y, coordinates.workOffset.z }, WORK_TRIAD_LENGTH, true);
        glLineWidth(1.0f);
        glPointSize(1.0f);

        const auto projectPoint = [&](const glm::dvec3 &point) {
            const auto transformed = m_modelOrientation * displayOffset(point - m_modelPivot);
            const auto ndcX = (transformed.x + m_viewPan.x) / (m_viewHalfHeight * aspect);
            const auto ndcY = (transformed.z + m_viewPan.y) / m_viewHalfHeight;
            return ImVec2(
                static_cast<float>((ndcX + 1.0) * width * 0.5),
                static_cast<float>((1.0 - ndcY) * height * 0.5));
        };

        const auto machineOrigin = projectPoint({ 0.0, 0.0, 0.0 });
        const auto workOrigin = projectPoint({ coordinates.workOffset.x, coordinates.workOffset.y, coordinates.workOffset.z });
        const auto screenDifference = ImVec2(workOrigin.x - machineOrigin.x, workOrigin.y - machineOrigin.y);
        const auto originsOverlap = screenDifference.x * screenDifference.x + screenDifference.y * screenDifference.y < 24.0f * 24.0f;
        const auto machineLabelPosition = ImVec2(machineOrigin.x + 8.0f, machineOrigin.y - (originsOverlap ? 28.0f : 18.0f));
        const auto workLabelPosition = ImVec2(workOrigin.x + 8.0f, workOrigin.y + (originsOverlap ? 6.0f : -18.0f));

        auto *drawList = ImGui::GetForegroundDrawList(ImGui::GetMainViewport());
        drawList->AddText(machineLabelPosition, IM_COL32(180, 180, 180, 255), "MCS");
        drawList->AddText(
            workLabelPosition,
            IM_COL32(255, 205, 45, 255),
            std::format("{} WCS", coordinates.workCoordinateName).c_str());

        glLineWidth(2.0f);
        m_worker.lock([&] {
            m_worker.toolpath().foreachCommand([&](const ngc::MachineCommand &cmd) {
                if(auto moveLine = std::get_if<ngc::MoveLine>(&cmd)) {
                    if(moveLine->machineCoordinates()) {
                        glColor3f(1.0, 1.0, 0.4);
                    } else {
                        glColor3f(0.4, 0.4, 1.0);
                    }

                    const auto rapid = moveLine->speed() == -1.0;
                    if(rapid) {
                        glEnable(GL_LINE_STIPPLE);
                        glLineStipple(6, 0x5555);
                    }

                    auto &v = moveLine->from();
                    auto &w = moveLine->to();

                    glBegin(GL_LINES);
                    glVertex3d(v.x, v.y, v.z);
                    glVertex3d(w.x, w.y, w.z);
                    glEnd();

                    if(rapid) glDisable(GL_LINE_STIPPLE);
                    
                    glEnable(GL_POINT_SMOOTH);
                    glPointSize(2.0);

                    glBegin(GL_POINTS);
                    glColor3f(0.0, 0.0, 0.0);
                    glVertex3d(v.x, v.y, v.z);
                    glVertex3d(w.x, w.y, w.z);
                    glEnd();
                }

                if(auto probeMove = std::get_if<ngc::ProbeMove>(&cmd)) {
                    const auto &from = probeMove->from();
                    const auto &target = probeMove->target();
                    glBegin(GL_LINES);
                    glColor3f(1.0, 0.4, 0.2);
                    glVertex3d(from.x, from.y, from.z);
                    glVertex3d(target.x, target.y, target.z);
                    glEnd();
                }

                if(auto moveArc = std::get_if<ngc::MoveArc>(&cmd)) {
                    interpolate(*moveArc, 60, [&](const glm::dvec3 &start, const glm::dvec3 &end, const bool startPoint, const bool endPoint) {
                        glBegin(GL_LINES);
                        glColor3f(0.4, 1.0, 0.4);
                        glVertex3d(start.x, start.y, start.z);
                        glVertex3d(end.x, end.y, end.z);
                        glEnd();

                        if(startPoint) {
                            glEnable(GL_POINT_SMOOTH);
                            glPointSize(2.0);
                            glBegin(GL_POINTS);
                            glColor3f(0.0, 0.0, 0.0);
                            glVertex3d(start.x, start.y, start.z);
                            glEnd();
                        }

                        if(endPoint) {
                            glEnable(GL_POINT_SMOOTH);
                            glPointSize(2.0);
                            glBegin(GL_POINTS);
                            glColor3f(1.0, 1.0, 1.0);
                            glVertex3d(end.x, end.y, end.z);
                            glEnd();
                        }
                    });
                }
            });
        });
        glLineWidth(1.0f);

        const auto simulation = m_simulation.snapshot();
        if(simulation.status != ngc::SimulationStatus::Stopped) {
            drawToolWireframe(simulation.toolPose);
        }
    }

    void interpolate(const ngc::MoveArc &arc, int circleResolution, const std::function<void(const glm::dvec3 &start, const glm::dvec3 &end, const bool startPoint, const bool endPoint)> &callback) {
        const auto geometry = ngc::simulation_detail::arcGeometry(arc);
        const auto sweep = geometry ? geometry->sweep : 0.0;
        const auto segments = geometry && circleResolution > 0
            ? std::max(1, static_cast<int>(std::ceil(sweep * circleResolution / (2.0 * std::numbers::pi))))
            : 1;

        auto previous = arc.from();
        for(int i = 1; i <= segments; i++) {
            const auto current = ngc::simulation_detail::interpolate(arc, static_cast<double>(i) / segments);
            callback(
                { previous.x, previous.y, previous.z },
                { current.x, current.y, current.z },
                i == 1,
                i == segments);
            previous = current;
        }
    }

    void renderErrorWindow() {
        ImGui::OpenPopup("Error");

        if(ImGui::BeginPopupModal("Error", &m_enableErrorWindow, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
            ImGui::TextUnformatted(m_errorMessage.c_str());
            ImGui::PopStyleColor();
        }

        ImGui::EndPopup();
    }

    void render() {
        auto drawList = ImGui::GetForegroundDrawList(ImGui::GetMainViewport());
        drawList->AddText({ 10, 20 }, IM_COL32(0, 255, 0, 255), std::format("fps: {:.1f}", 1.0 / m_dt).c_str());

        static double lastTime = 0.0;
        static double avgFps = 0.0;

        constexpr auto alpha = 0.999;
        avgFps = avgFps * alpha + (1.0 - alpha) * (1.0 / m_dt);
        
        if(std::isnan(avgFps)) {
            avgFps = 1.0;
        }

        if(lastTime == 0.0 || m_time - lastTime > 1.0) {
            lastTime = m_time;
            std::println("fps: {:.1f}", avgFps);
        }

        if(ImGui::BeginMainMenuBar()) {
            if(ImGui::BeginMenu("File")) {
                ImGui::MenuItem("Open", nullptr, &m_enableOpenDialog);
                ImGui::MenuItem("System Parameters", nullptr, &m_enableMemoryWindow);

                if(ImGui::MenuItem("Tool Table", nullptr, &m_enableToolWindow)) {
                    initToolTableStrings();
                }

                ImGui::MenuItem("Messages", nullptr, &m_enableMessagesWindow);
                
                if(ImGui::MenuItem("Exit")) { glfwSetWindowShouldClose(m_window, GL_TRUE); }

                ImGui::EndMenu();
            }
        }

        ImGui::EndMainMenuBar();

        if(m_enableOpenDialog) { renderOpenDialog(); }
        if(m_enableProgramWindow) { renderProgramWindow(); }
        if(m_enableMemoryWindow) { renderMemoryWindow(); }
        if(m_enableToolWindow) { renderToolWindow(); }
        if(m_enableErrorWindow) { renderErrorWindow(); }
        if(m_enableMessagesWindow) { renderMessagesWindow(); }
    }

    void renderMessagesWindow() {
        if(ImGui::Begin("Messages", &m_enableMessagesWindow)) {
            for(const auto &msg : m_worker.printMessages()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Blueish);
                ImGui::TextUnformatted(std::format("PRINT: {}", msg).c_str());
                ImGui::PopStyleColor();
            }
        }

        ImGui::End();
    }

    void renderToolWindow() {
        ImGui::SetNextWindowSize({ 950.0f, 360.0f }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints({ 500.0f, 220.0f }, { FLT_MAX, FLT_MAX });

        if(ImGui::Begin("Tool Table", &m_enableToolWindow)) {
            ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, { 0, 0 });
            const auto tableHeight = -ImGui::GetFrameHeightWithSpacing();

            if(ImGui::BeginTable("##tool_table", 9,
                                 ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                 ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable |
                                 ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY,
                                 { 0.0f, tableHeight })) {
                ImGui::TableSetupColumn("Tool Number");
                ImGui::TableSetupColumn("X");
                ImGui::TableSetupColumn("Y");
                ImGui::TableSetupColumn("Z");
                ImGui::TableSetupColumn("A");
                ImGui::TableSetupColumn("B");
                ImGui::TableSetupColumn("C");
                ImGui::TableSetupColumn("Diameter");
                ImGui::TableSetupColumn("Comment");
                ImGui::TableHeadersRow();

                for(std::size_t row = 0; row < m_toolStrings.size(); row++) {
                    auto &tool = m_toolStrings[row];
                    ImGui::PushID(static_cast<int>(row));
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##number", &tool.number);

                    ImGui::TableSetColumnIndex(1);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##x", &tool.x);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##y", &tool.y);

                    ImGui::TableSetColumnIndex(3);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##z", &tool.z);

                    ImGui::TableSetColumnIndex(4);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##a", &tool.a);

                    ImGui::TableSetColumnIndex(5);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##b", &tool.b);

                    ImGui::TableSetColumnIndex(6);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##c", &tool.c);

                    ImGui::TableSetColumnIndex(7);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##diameter", &tool.diameter);

                    ImGui::TableSetColumnIndex(8);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##comment", &tool.comment);
                    ImGui::PopID();
                }

                ImGui::EndTable();
            }

            ImGui::PopStyleVar();

            if(ImGui::Button("Reload")) {
                reloadToolTable();
            }

            ImGui::SameLine();

            if(ImGui::Button("Save")) {
                saveToolTableStrings();
            }

            ImGui::SameLine();

            if(ImGui::Button("New")) {
                int nextToolNumber = 1;

                for(const auto &tool : m_toolStrings) {
                    if(const auto number = ngc::fromChars(tool.number)) {
                        nextToolNumber = std::max(nextToolNumber, static_cast<int>(*number) + 1);
                    }
                }

                m_toolStrings.emplace_back(tool_table_strings_t {
                    .number = ngc::toChars(nextToolNumber),
                    .x = "0",
                    .y = "0",
                    .z = "0",
                    .a = "0",
                    .b = "0",
                    .c = "0",
                    .diameter = "0",
                    .comment = {},
                });
            }
        }

        ImGui::End();
    }

    void renderMemoryWindow() {
        if(ImGui::Begin("System Parameters", &m_enableMemoryWindow)) {
            if(ImGui::BeginTable("", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Address");
                ImGui::TableSetupColumn("Internal Name");
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Value");
                ImGui::TableHeadersRow();

                uint32_t address = 0;

                for(const auto &[var, name, addr, _flags, _value] : ngc::gVars) {
                    if(addr != 0) {
                        address = addr;
                    }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(std::format("{}", address).c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(std::format("{}", ngc::name(var)).c_str());

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(std::format("{}", name).c_str());

                    ImGui::TableSetColumnIndex(3);
                    auto value = m_worker.read(var);
                    ImGui::TextUnformatted(std::format("{}", value).c_str());

                    address++;
                }
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

    void renderProgramWindow() {
        if(ImGui::Begin("Program", &m_enableProgramWindow)) {
            if(ImGui::BeginChild("top", { 0, 0 }, ImGuiChildFlags_Border | ImGuiChildFlags_ResizeY)) {
                if(ImGui::Button("Compile Programs", {0,0}, m_worker.busy())) {
                    m_worker.compile(m_programSource);
                }

                if(m_worker.compiled()) {
                    if(ImGui::Button("Build Preview")) {
                        m_worker.execute();
                    }
                }

                const auto simulation = m_simulation.snapshot();
                const auto simulationActive = simulation.status == ngc::SimulationStatus::Running
                    || simulation.status == ngc::SimulationStatus::Paused;

                ImGui::SameLine();
                ImGui::BeginDisabled(m_programSource.empty() || simulationActive);
                if(ImGui::Button("Run Simulation")) {
                    m_simulation.setPlaybackRate(m_simulationRate);
                    m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
                    m_simulation.start(m_programSource, m_tools);
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!simulationActive);
                if(simulation.status == ngc::SimulationStatus::Paused) {
                    if(ImGui::Button("Resume")) m_simulation.resume();
                } else {
                    if(ImGui::Button("Pause")) m_simulation.pause();
                }
                ImGui::SameLine();
                if(ImGui::Button("Stop")) m_simulation.stop();
                ImGui::EndDisabled();

                ImGui::SetNextItemWidth(120.0f);
                if(ImGui::InputDouble("Playback rate", &m_simulationRate, 0.1, 1.0, "%.2fx")) {
                    m_simulationRate = std::clamp(m_simulationRate, 0.01, 1000.0);
                    m_simulation.setPlaybackRate(m_simulationRate);
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0f);
                if(ImGui::InputDouble("Rapid speed", &m_simulatedRapidSpeed, 10.0, 100.0, "%.2f")) {
                    m_simulatedRapidSpeed = std::max(m_simulatedRapidSpeed, 1e-6);
                    m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
                }

                const auto statusText = [&] {
                    switch(simulation.status) {
                        case ngc::SimulationStatus::Stopped: return "Stopped";
                        case ngc::SimulationStatus::Running: return "Running";
                        case ngc::SimulationStatus::Paused: return "Paused";
                        case ngc::SimulationStatus::Completed: return "Completed";
                        case ngc::SimulationStatus::Error: return "Error";
                    }
                    return "Unknown";
                }();
                ImGui::Text("Simulation: %s  XYZ: %.4f, %.4f, %.4f",
                            statusText, simulation.toolPosition.x, simulation.toolPosition.y, simulation.toolPosition.z);
                if(!simulation.error.empty()) {
                    ImGui::TextColored(ImVec4_Redish, "%s", simulation.error.c_str());
                }

                if(ImGui::BeginTabBar("programs")) {
                    for(auto &[source, name] : m_programSource) {
                        if(ImGui::BeginTabItem(name.c_str())) {
                            ImGui::InputTextMultiline("##source", &source, { -1, -1 });
                            ImGui::EndTabItem();
                        }
                    }
                }

                ImGui::EndTabBar();
            }

            ImGui::EndChild();

            if(ImGui::BeginChild("middle", {0, 0 }, ImGuiChildFlags_Border | ImGuiChildFlags_ResizeY)) {
                for(const auto &error : m_worker.parserErrors()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
                    ImGui::Selectable(error.text().c_str());
                    ImGui::PopStyleColor();
                }
            }

            ImGui::EndChild();

            if(ImGui::BeginChild("bottom", {0, 0 }, ImGuiChildFlags_Border)) {
                for(const auto &block : m_worker.blockMessages()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Greenish);
                    ImGui::Selectable(block.c_str());
                    ImGui::PopStyleColor();
                }
            }

            ImGui::EndChild();
        }

        ImGui::End();
    }

    void renderOpenDialog() {
        ImGui::OpenPopup("Open File");

        if(ImGui::BeginPopupModal("Open File", &m_enableOpenDialog)) {
            ImGui::Text("path: %s", m_path.string().c_str());
            ImGui::Separator();

            if(ImGui::Selectable("..")) {
                m_path = std::filesystem::absolute(m_path.append("..")).lexically_normal();
            }

            for(const auto &entry : std::filesystem::directory_iterator(m_path)) {
                if(ImGui::Selectable(std::format("{}", entry.path().filename().string()).c_str())) {
                    if(entry.is_directory()) {
                        m_path = std::filesystem::absolute(entry.path()).lexically_normal();
                    }

                    if(entry.is_regular_file()) {
                        auto result = ngc::readFile(entry.path().string());

                        if(!result) {
                            m_programSource.clear();
                            m_errorMessage = result.error().what();
                            m_enableErrorWindow = true;
                            break;
                        }

                        m_programSource.clear();

                        for(const auto &entry : std::filesystem::directory_iterator("autoload")) {
                            auto result = ngc::readFile(entry.path().string());

                            if(!result) {
                                m_programSource.clear();
                                m_errorMessage = result.error().what();
                                m_enableErrorWindow = true;
                                break;
                            }

                            m_programSource.emplace_back(*result, entry.path().string());
                        }

                        m_programSource.emplace_back(*result, entry.path().string());
                        m_enableOpenDialog = false;
                        m_enableProgramWindow = true;
                    }
                }
            }

            ImGui::EndPopup();
        }
    }
};
