cmake_minimum_required(VERSION 3.28)
project(ngc)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-g -O0 -Wall -Wextra)
#add_compile_options(-g -O2 -fsanitize=undefined -Wall -Wextra)
#add_link_options(-fsanitize=undefined)

set(IMGUI_DIR imgui)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
#find_package(glfw3 REQUIRED)
add_subdirectory(glfw)

# add_library(utils)
# target_sources(utils PUBLIC FILE_SET CXX_MODULES FILES src/modules/Utils.cppm)

# add_library(parser)
# target_sources(parser PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/parser/LexerSource.cppm
#         src/modules/parser/TokenSource.cppm
#         src/modules/parser/StringTokenSource.cppm
#         src/modules/parser/StringViewTokenSource.cppm
#         src/modules/parser/Token.cppm
#         src/modules/parser/Lexer.cppm
#         src/modules/parser/Visitor.cppm
#         src/modules/parser/Expression.cppm
#         src/modules/parser/Statement.cppm
#         src/modules/parser/SubSignature.cppm
#         src/modules/parser/Parser.cppm)
# target_link_libraries(parser utils)

# add_library(gcode)
# target_sources(gcode PUBLIC FILE_SET CXX_MODULES FILES src/modules/gcode/GCode.cppm)
# target_link_libraries(gcode parser)

# add_library(memory)
# target_sources(memory PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/memory/Vars.cppm
#         src/modules/memory/MemoryCell.cppm
#         src/modules/memory/Memory.cppm)
# target_link_libraries(memory)

# add_library(machine
#         src/modules/machine/ToolTable.cppm)
# target_sources(machine PUBLIC FILE_SET CXX_MODULES FILES src/modules/machine/Machine.cppm src/modules/machine/MachineCommand.cppm src/modules/machine/ToolTable.cppm)
# target_link_libraries(machine utils gcode memory)

# add_library(evaluator)
# target_sources(evaluator PUBLIC FILE_SET CXX_MODULES FILES
#         src/modules/evaluator/Preamble.cppm
#         src/modules/evaluator/Program.cppm
#         src/modules/evaluator/Evaluator.cppm)
# target_link_libraries(evaluator parser memory gcode)

# add_executable(ngc main.cpp)
# target_link_libraries(ngc utils parser memory evaluator machine)

add_executable(imgui_main
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        src/imgui_main.cpp
        src/Application.h)

target_include_directories(imgui_main PUBLIC 
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
        ${IMGUI_DIR}/misc/cpp
        src/
        src/include)

target_link_libraries(imgui_main OpenGL::GL glfw)

add_executable(binary_to_compressed_c ${IMGUI_DIR}/misc/fonts/binary_to_compressed_c.cpp)

add_executable(test src/test.cpp)
target_include_directories(test PUBLIC src/ src/include)