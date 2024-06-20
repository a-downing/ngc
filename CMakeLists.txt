cmake_minimum_required(VERSION 3.28)
project(ngc)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-O2")

add_library(utils)
target_sources(utils PUBLIC FILE_SET CXX_MODULES FILES modules/Utils.cppm)

add_library(parser)
target_sources(parser PUBLIC FILE_SET CXX_MODULES FILES
        modules/parser/LexerSource.cppm
        modules/parser/TokenSource.cppm
        modules/parser/StringTokenSource.cppm
        modules/parser/StringViewTokenSource.cppm
        modules/parser/Token.cppm
        modules/parser/Lexer.cppm
        modules/parser/Visitor.cppm
        modules/parser/Expression.cppm
        modules/parser/Statement.cppm
        modules/parser/SubSignature.cppm
        modules/parser/Parser.cppm)
target_link_libraries(parser utils)

add_library(gcode)
target_sources(gcode PUBLIC FILE_SET CXX_MODULES FILES modules/gcode/GCode.cppm)
target_link_libraries(gcode parser)

add_library(memory)
target_sources(memory PUBLIC FILE_SET CXX_MODULES FILES
        modules/memory/Vars.cppm
        modules/memory/MemoryCell.cppm
        modules/memory/Memory.cppm)
target_link_libraries(memory)

add_library(machine
        modules/machine/ToolTable.cppm)
target_sources(machine PUBLIC FILE_SET CXX_MODULES FILES modules/machine/Machine.cppm modules/machine/MachineCommand.cppm modules/machine/ToolTable.cppm)
target_link_libraries(machine utils gcode memory)

add_library(evaluator)
target_sources(evaluator PUBLIC FILE_SET CXX_MODULES FILES
        modules/evaluator/Preamble.cppm
        modules/evaluator/Program.cppm
        modules/evaluator/Evaluator.cppm)
target_link_libraries(evaluator parser memory gcode)

add_executable(ngc main.cpp)
target_link_libraries(ngc utils parser memory evaluator machine)
