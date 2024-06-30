#pragma once

#include <syncstream>
#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <optional>
#include <filesystem>
#include <print>
#include <string>

#include "imgui.h"
#include "imgui_stdlib.h"

#include "utils.h"
#include "parser/LexerSource.h"
#include "parser/Program.h"

#include "memory/Vars.h"
#include "memory/Memory.h"
#include "machine/Machine.h"
#include "machine/ToolTable.h"
#include "evaluator/Preamble.h"
#include "evaluator/Evaluator.h"

struct tool_table_strings_t {
    std::string number;
    std::string x, y, z, a, b, c;
    std::string diameter;
    std::string comment;

    static tool_table_strings_t from(const ngc::ToolTable::tool_entry_t &tool) {
        return {
            ngc::toChars(tool.number),
            ngc::toChars(tool.x),
            ngc::toChars(tool.y),
            ngc::toChars(tool.z),
            ngc::toChars(tool.a),
            ngc::toChars(tool.b),
            ngc::toChars(tool.c),
            ngc::toChars(tool.diameter),
            tool.comment
        };
    }

    std::expected<ngc::ToolTable::tool_entry_t, std::string> to() const {
        auto _number = ngc::fromChars(number);
        if(!_number) {  return std::unexpected(std::format("failed to convert '{}' to number", number)); }

        auto _x = ngc::fromChars(x);
        if(!_x) {  return std::unexpected(std::format("failed to convert '{}' to number", x)); }

        auto _y = ngc::fromChars(y);
        if(!_y) {  return std::unexpected(std::format("failed to convert '{}' to number", y)); }

        auto _z = ngc::fromChars(z);
        if(!_z) {  return std::unexpected(std::format("failed to convert '{}' to number", z)); }

        auto _a = ngc::fromChars(a);
        if(!_a) {  return std::unexpected(std::format("failed to convert '{}' to number", a)); }

        auto _b = ngc::fromChars(b);
        if(!_b) {  return std::unexpected(std::format("failed to convert '{}' to number", b)); }

        auto _c = ngc::fromChars(c);
        if(!_c) {  return std::unexpected(std::format("failed to convert '{}' to number", c)); }

        auto _diameter = ngc::fromChars(diameter);
        if(!_diameter) {  return std::unexpected(std::format("failed to convert '{}' to number", diameter)); }
        
        return ngc::ToolTable::tool_entry_t { static_cast<int>(*_number), *_x, *_y, *_z, *_a, *_b, *_c, *_diameter, comment };
    }
};

class Application {
    std::filesystem::path m_path = std::filesystem::absolute(".").lexically_normal();
    ngc::Memory m_mem;
    ngc::Machine m_machine;
    ngc::ToolTable m_tools;
    std::vector<tool_table_strings_t> m_toolStrings;

    std::optional<ngc::Preamble> m_preamble;
    std::vector<ngc::Program> m_programs;
    bool m_compiled = false;

    // windows
    bool m_enableOpenDialog = false;
    bool m_enableProgramWindow = false;
    bool m_enableMemoryWindow = false;
    bool m_enableToolWindow = false;
    bool m_enableErrorWindow = false;

    std::string m_errorMessage;

public:
    Application() : m_machine(m_mem) { }

    void init() {
        auto result = m_tools.load();

        if(!result) {
            m_errorMessage = "failed to load tool table";
            m_enableErrorWindow = true;
        }
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
            ImGui::PushStyleColor(ImGuiCol_Text, { 1.0f, 0.4f, 0.4f, 1.0f });
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
                
                if(ImGui::MenuItem("Exit")) { std::exit(0); }
                
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        if(m_enableOpenDialog) { renderOpenDialog(); }
        if(m_enableProgramWindow) { renderProgramWindow(); }
        if(m_enableMemoryWindow) { renderMemoryWindow(); }
        if(m_enableToolWindow) { renderToolWindow(); }
        if(m_enableErrorWindow) { renderErrorWindow(); }
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

                for(const auto &[var, name, addr, _, _] : ngc::gVars) {
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
        if(m_programs.empty()) {
            m_enableProgramWindow = false;
            return;
        }

        if(ImGui::Begin("Program", &m_enableProgramWindow)) {
            if(!m_programs.empty() && !m_compiled) {
                if(ImGui::Button("Compile Programs")) {
                    for(auto &program : m_programs) {
                        auto result = program.compile();

                        if(!result) {
                            m_compiled = false;
                            m_errorMessage = result.error().text();
                            m_enableErrorWindow = true;
                            break;
                        }
                    }

                    m_compiled = true;
                }
            }

            if(m_compiled) {
                //TODO: this stuff
                if(ImGui::Button("Execute")) {
                    auto callback = [this] (const ngc::Block &block, ngc::Evaluator &eval) {
                        // if(block.blockDelete()) {
                        //     std::println("DELETED BLOCK: {}", block.statement()->text());
                        // } else{
                        //     std::println("BLOCK: {}", block.statement()->text());
                        // }

                        ngc::GCodeState state;

                        for(const auto &word : block.words()) {
                            state.affectState(word);
                        }

                        if(state.modeToolChange()) {
                            eval.call("_tool_change", state.T());
                        }

                        //m_machine.executeBlock(block);
                    };

                    auto eval = ngc::Evaluator(m_mem, callback);

                    std::println("first pass: preamble");
                    eval.executeFirstPass(m_preamble->statements());

                    for(const auto &program : m_programs) {
                        std::println("first pass: {}", program.source().name());
                        eval.executeFirstPass(program.statements());
                    }

                    for(const auto &program : m_programs) {
                        std::println("executing: {}", program.source().name());
                        eval.executeSecondPass(program.statements());
                    }
                }
            }

            if(ImGui::BeginTabBar("programs")) {
                m_preamble = ngc::Preamble(m_mem);
                std::string preambleText;

                for(const auto &stmt : m_preamble->statements()) {
                    preambleText += stmt->text() + '\n';
                }

                if(ImGui::BeginTabItem("preamble")) {
                    ImGui::InputTextMultiline("##source", &preambleText, { -1, -1 }, ImGuiInputTextFlags_ReadOnly);
                    ImGui::EndTabItem();
                }

                for(const auto &program : m_programs) {
                    if(ImGui::BeginTabItem(program.source().name().c_str())) {
                        ImGui::InputTextMultiline("##source", const_cast<char *>(program.source().text().data()), program.source().text().size(), { -1, -1 }, ImGuiInputTextFlags_ReadOnly);
                        ImGui::EndTabItem();
                    }
                }

                ImGui::EndTabBar();
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

            for(const auto entry : std::filesystem::directory_iterator(m_path)) {
                if(ImGui::Selectable(std::format("{}", entry.path().filename().string()).c_str())) {
                    if(entry.is_directory()) {
                        m_path = std::filesystem::absolute(entry.path()).lexically_normal();
                    }

                    if(entry.is_regular_file()) {
                        auto result = ngc::readFile(entry.path().string());

                        if(result) {
                            auto program = ngc::Program(ngc::LexerSource(*result, entry.path().string()));
                            m_programs.clear();
                            m_compiled = false;

                            for(const auto entry : std::filesystem::directory_iterator("autoload")) {
                                m_programs.emplace_back(ngc::LexerSource(*ngc::readFile(entry.path().string()), entry.path().string()));
                            }

                            m_programs.emplace_back(std::move(program));
                            m_enableOpenDialog = false;
                            m_enableProgramWindow = true;
                        }
                    }
                }
            }

            ImGui::EndPopup();
        }
    }
};