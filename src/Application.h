#pragma once

#include "parser/Statement.h"
#include <cstdint>

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <filesystem>
#include <print>
#include <string>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_stdlib.h"

#include "utils.h"
#include "Worker.h"

#include "gui/imgui_custom.h"
#include "memory/Vars.h"
#include "memory/Memory.h"
#include "machine/Machine.h"
#include "machine/ToolTable.h"
#include "evaluator/Evaluator.h"

#include "gui/tool_table_strings_t.h"

class Application final : public ngc::EvaluatorMessageVisitor {
    GLFWwindow *m_window;
    std::filesystem::path m_path = std::filesystem::absolute(".").lexically_normal();
    ngc::Memory m_mem;
    ngc::Machine m_machine;
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

    Worker m_worker;

public:
    Application(GLFWwindow *window) : m_window(window), m_machine(m_mem), m_worker(m_mem) { }

    void init() {
        auto result = m_tools.load();

        if(!result) {
            m_errorMessage = "failed to load tool table";
            m_enableErrorWindow = true;
        }
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

    void renderErrorWindow() {
        ImGui::OpenPopup("Error");

        if(ImGui::BeginPopupModal("Error", &m_enableErrorWindow, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
            ImGui::TextUnformatted(m_errorMessage.c_str());
            ImGui::PopStyleColor();
            ImGui::EndPopup();
        }
    }

    void render() {
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

            ImGui::EndMainMenuBar();
        }

        if(m_enableOpenDialog) { renderOpenDialog(); }
        if(m_enableProgramWindow) { renderProgramWindow(); }
        if(m_enableMemoryWindow) { renderMemoryWindow(); }
        if(m_enableToolWindow) { renderToolWindow(); }
        if(m_enableErrorWindow) { renderErrorWindow(); }
        if(m_enableMessagesWindow) { renderMessagesWindow(); }
    }

    void renderMessagesWindow() {
        if(ImGui::Begin("Messages", &m_enableMessagesWindow)) {
            // m_worker.foreachEvaluatorMessage([](const ngc::EvaluatorMessage *msg) {
            //     if(auto blockMsg = msg->as<ngc::BlockMessage>()) {
            //         ImGui::TextUnformatted(blockMsg->block().statement()->text().c_str());
            //         return;
            //     }

            //     if(auto printMsg = msg->as<ngc::PrintMessage>()) {
            //         ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Blueish);
            //         ImGui::TextUnformatted(std::format("PRINT: {}", printMsg->text()).c_str());
            //         ImGui::PopStyleColor();
            //     }
            // });

            ImGui::End();
        }
    }

    void visit(const ngc::BlockMessage &msg) override {
        //std::println("BlockMessage: {}", msg.block().statement()->text());
        //ImGui::TextUnformatted(msg.block().statement()->text().c_str());
    }

    void visit(const ngc::PrintMessage &msg) override {
        //std::println("PrintMessage: {}", msg.text());
        //ImGui::TextUnformatted(std::format("PRINT: {}", msg.text()).c_str());
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

                ImGui::EndTable();
                ImGui::PopStyleVar();
            }

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

            ImGui::End();
        }
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
                    auto value = m_mem.read(var);
                    ImGui::TextUnformatted(std::format("{}", value).c_str());

                    address++;
                }

                ImGui::EndTable();
            }

            ImGui::End();
        }
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

                    ImGui::EndTabBar();
                }

                ImGui::EndChild();
            }

            if(ImGui::BeginChild("bottom", {0, 0 }, ImGuiChildFlags_Border)) {
                for(const auto &error : m_worker.parserErrors()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
                    ImGui::Selectable(error.text().c_str());
                    ImGui::PopStyleColor();
                }

                ImGui::EndChild();
            }

            ImGui::End();
        }
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
                        //m_worker.reset(std::move(programs));
                        m_enableOpenDialog = false;
                        m_enableProgramWindow = true;
                    }
                }
            }

            ImGui::EndPopup();
        }
    }
};