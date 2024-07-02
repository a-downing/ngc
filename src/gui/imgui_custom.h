#pragma once

#include "imgui.h"

constexpr ImVec4 ImVec4_Redish = { 1.0f, 0.4f, 0.4f, 1.0f };
constexpr ImVec4 ImVec4_Greenish = { 0.4f, 1.0f, 0.4f, 1.0f };
constexpr ImVec4 ImVec4_Blueish = { 0.4f, 0.4f, 1.0f, 1.0f };

namespace ImGui {
    inline bool Button(const char* label, const ImVec2& size_arg, bool disabled) {
        if(disabled) {
            ImGui::BeginDisabled();
        }
        
        auto result = ImGui::Button(label, size_arg);

        if(disabled) {
            ImGui::EndDisabled();
        }

        return result;
    }
}