#pragma once

#include <filesystem>
#include <print>
#include <string>
#include <cmath>
#include <cstdint>
#include <ranges>
#include <numbers>

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

    uint64_t m_frames = 0;
    double m_time;
    double m_dt;
    double m_mouseX;
    double m_mouseY;
    double m_mouseDX;
    double m_mouseDY;

    double m_yaw = 0;
    double m_pitch = 0;
    glm::dvec3 m_cameraPosition = { 0.0, -12.0, 0.0 };
    glm::dvec3 m_cameraVelocity = { 0.0, 0.0, 0.0 };
    double m_cameraAcceleration = 1.0;

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
        m_frames++;
        m_dt = ImGui::GetTime() - m_time;
        m_time = ImGui::GetTime();
        auto pos = ImGui::GetMousePos();
        m_mouseDX = pos.x - m_mouseX;
        m_mouseDY = pos.y - m_mouseY;
        m_mouseX = pos.x;
        m_mouseY = pos.y;
    }

    void terminate() {
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
                break;
            }

            m_tools.set(tool->number, *tool);
        }

        auto result = m_tools.save();

        if(!result) {
            m_errorMessage = result.error();
            m_enableErrorWindow = true;
            return;
        }

        initToolTableStrings();
    }

    bool noWindowHasFocus() {
        auto ctx = ImGui::GetCurrentContext();
        return ctx->NavWindow == nullptr && ctx->MovingWindow == nullptr && ctx->WheelingWindow == nullptr;
    }

    void render3D() {
        int width, height;
        glfwGetWindowSize(m_window, &width, &height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, static_cast<double>(width) / height, 0.001, 1000.0);

        auto down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        auto notFocused = noWindowHasFocus();

        auto w = ImGui::IsKeyDown(ImGuiKey_W);
        auto a = ImGui::IsKeyDown(ImGuiKey_A);
        auto s = ImGui::IsKeyDown(ImGuiKey_S);
        auto d = ImGui::IsKeyDown(ImGuiKey_D);
        auto ctrl = ImGui::IsKeyDown(ImGuiKey_LeftCtrl);
        auto space = ImGui::IsKeyDown(ImGuiKey_Space);
        auto shift = ImGui::IsKeyDown(ImGuiKey_LeftShift);

        if(down && notFocused) {
            m_yaw += m_mouseDX * 0.001;
            m_pitch += -m_mouseDY * 0.001;
        }

        glm::dvec3 front;
        front.x = sin(m_yaw) * cos(m_pitch);
        front.y = cos(m_yaw) * cos(m_pitch);
        front.z = sin(m_pitch);
        
        glm::dvec3 right;
        right.x = cos(m_yaw);
        right.y = -sin(m_yaw);
        right.z = 0.0f;

        auto cameraFront = glm::normalize(front);
        auto cameraRight = glm::normalize(right);
        auto cameraUp = glm::cross(cameraRight, cameraFront);

        if(down && notFocused) {
            auto boost = shift ? 10.0 : 1.0;

            if(w) {
                m_cameraVelocity += cameraFront * m_cameraAcceleration * boost * m_dt;
            }

            if(a) {
                m_cameraVelocity -= cameraRight * m_cameraAcceleration * boost * m_dt;
            }

            if(s) {
                m_cameraVelocity -= cameraFront * m_cameraAcceleration * boost * m_dt;
            }

            if(d) {
                m_cameraVelocity += cameraRight * m_cameraAcceleration * boost * m_dt;
            }

            if(ctrl) {
                m_cameraVelocity -= cameraUp * m_cameraAcceleration * boost * m_dt;
            }

            if(space) {
                m_cameraVelocity += cameraUp * m_cameraAcceleration * boost * m_dt;
            }

            if(!w && !a && !s && !d && !ctrl && !space) {
                m_cameraVelocity = { 0, 0, 0 };
            }

            if(glm::length(m_cameraVelocity) > boost) {
                m_cameraVelocity = glm::normalize(m_cameraVelocity) * boost;
            }

            m_cameraPosition += m_cameraVelocity * m_dt;
        }

        auto mat = glm::lookAt(m_cameraPosition, m_cameraPosition + cameraFront, cameraUp);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixd(glm::value_ptr(mat));

        glBegin(GL_LINES);

        //X
        glColor3f(1.0, 0.4, 0.4);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(2.0, 0.0, 0.0);

        //Y
        glColor3f(0.4, 2.0, 0.4);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 2.0, 0.0);

        //Z
        glColor3f(0.4, 0.4, 2.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 2.0);

        auto workOffset = m_worker.lock([&] {
            return m_worker.machine().workOffset();
        });

        //X
        glColor3f(1.0, 0.4, 0.4);
        glVertex3d(workOffset.x, workOffset.y, workOffset.z);
        glVertex3d(workOffset.x + 1.0, workOffset.y, workOffset.z);

        //Y
        glColor3f(0.4, 1.0, 0.4);
        glVertex3d(workOffset.x, workOffset.y, workOffset.z);
        glVertex3d(workOffset.x, workOffset.y + 1.0, workOffset.z);

        //Z
        glColor3f(0.4, 0.4, 1.0);
        glVertex3d(workOffset.x, workOffset.y, workOffset.z);
        glVertex3d(workOffset.x, workOffset.y, workOffset.z + 1.0);

        glEnd();

        m_worker.lock([&] {
            m_worker.machine().foreachCommand([&](const ngc::MachineCommand &cmd) {
                if(auto moveLine = cmd.as<ngc::MoveLine>()) {
                    if(moveLine->speed() == -1.0) {
                        glColor3f(1.0, 1.0, 0.4);
                    } else {
                        glColor3f(0.4, 0.4, 1.0);
                    }

                    auto &v = moveLine->from();
                    auto &w = moveLine->to();

                    glBegin(GL_LINES);
                    glVertex3d(v.x, v.y, v.z);
                    glVertex3d(w.x, w.y, w.z);
                    glEnd();
                    
                    glEnable(GL_POINT_SMOOTH);
                    glPointSize(2.0);

                    glBegin(GL_POINTS);
                    glColor3f(0.0, 0.0, 0.0);
                    glVertex3d(v.x, v.y, v.z);
                    glVertex3d(w.x, w.y, w.z);
                    glEnd();
                }

                if(auto moveArc = cmd.as<ngc::MoveArc>()) {
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
    }

    void interpolate(const ngc::MoveArc &arc, int circleResolution, const std::function<void(const glm::dvec3 &start, const glm::dvec3 &end, const bool startPoint, const bool endPoint)> &callback) {
        glm::dvec3 start = { arc.from().x, arc.from().y, arc.from().z };
        glm::dvec3 end = { arc.to().x, arc.to().y, arc.to().z };
        glm::dvec3 center = { arc.center().x, arc.center().y, arc.center().z };
        glm::dvec3 axis = { arc.axis().x, arc.axis().y, arc.axis().z };

        auto startArm = start - center - glm::proj(start - center, axis);
        auto endArm = end - center - glm::proj(end - center, axis);
        auto axial = glm::proj(end - start, axis);

        auto angle = glm::angle(glm::normalize(startArm), glm::normalize(endArm));

        if(angle <= 2.0 * std::numbers::pi / circleResolution) {
            callback(start, end, true, true);
            return;
        }

        int segments = static_cast<int>(angle / (2.0 * std::numbers::pi / circleResolution)) + 1;

        glm::dvec3 prev;

        for(int i = 0; i < segments + 1; i++) {
            auto s = static_cast<double>(i) / segments;
            auto startAngle = glm::lerp(0.0, angle, s);
            auto v1 = glm::rotate(startArm, startAngle, axis);
            auto v2 = glm::rotate(endArm, -(angle - startAngle), axis);
            auto v = center + glm::lerp(v1, v2, s) + axial * s;

            if(i > 0) {
                callback(prev, v, i == 1, i == segments);
            }
            
            prev = v;
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
        if(ImGui::Begin("Tool Table", &m_enableToolWindow, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, { 0, 0 });

            if(ImGui::BeginTable("", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable)) {
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

                for(auto &tool : m_toolStrings) {
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##number{}", tool.number).c_str(), &tool.number);

                    ImGui::TableSetColumnIndex(1);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##x{}", tool.number).c_str(), &tool.x);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##y{}", tool.number).c_str(), &tool.y);

                    ImGui::TableSetColumnIndex(3);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##z{}", tool.number).c_str(), &tool.z);

                    ImGui::TableSetColumnIndex(4);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##a{}", tool.number).c_str(), &tool.a);

                    ImGui::TableSetColumnIndex(5);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##b{}", tool.number).c_str(), &tool.b);

                    ImGui::TableSetColumnIndex(6);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##c{}", tool.number).c_str(), &tool.c);

                    ImGui::TableSetColumnIndex(7);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##diameter{}", tool.number).c_str(), &tool.diameter);

                    ImGui::TableSetColumnIndex(8);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText(std::format("##comment{}", tool.number).c_str(), &tool.comment);
                }

                ImGui::PopStyleVar();
            }

            ImGui::EndTable();

            if(ImGui::Button("Reload")) {
                initToolTableStrings();
            }

            ImGui::SameLine();

            if(ImGui::Button("Save")) {
                saveToolTableStrings();
            }

            ImGui::SameLine();

            if(ImGui::Button("New")) {
                m_toolStrings.emplace_back(tool_table_strings_t {});
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
            auto size = ImGui::GetWindowSize();

            if(ImGui::BeginChild("top", { 0, 0 }, ImGuiChildFlags_Border | ImGuiChildFlags_ResizeY)) {
                if(ImGui::Button("Compile Programs", {0,0}, m_worker.busy())) {
                    m_worker.compile(m_programSource);
                }

                if(m_worker.compiled()) {
                    if(ImGui::Button("Execute")) {
                        m_worker.execute();
                    }
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
            ImGui::Text("path: %s", m_path.c_str());
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