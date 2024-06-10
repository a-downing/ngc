cmake_minimum_required(VERSION 3.28)
project(ngc)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-g")

message(STATUS "${CMAKE_CXX_COMPILER}")

add_executable(ngc main.cpp
        include/Lexer.h
        include/Token.h
        include/CharacterSource.h
        include/Parser.h
        include/Statement.h
        include/Expression.h
        include/Utils.h
        include/Visitor.h
        include/VisitorContext.h
)
target_include_directories(ngc PUBLIC "${CMAKE_SOURCE_DIR}/include")
