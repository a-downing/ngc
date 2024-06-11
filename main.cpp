#include <print>
#include <fstream>
#include <filesystem>

#include <Utils.h>
#include <Token.h>
#include <Lexer.h>
#include <Parser.h>
#include <Memory.h>
#include <Preamble.h>
#include <SemanticAnalyzer.h>

#include "Program.h"

int main(const int argc, const char **argv) {
    if(argc < 2) {
        std::println(stderr, "usage: {} <filename>", argv[0]);
        return 0;
    }

    const std::string filename = argv[1];
    auto program = ngc::Program(ngc::CharacterSource(ngc::readFile(filename), filename));

    try {
        program.compile();
    } catch (const ngc::Parser::Error &err) {
        if(err.lexerError()) {
            std::println(stderr, "{}: {}: {}", err.lexerError()->location(), err.what(), err.lexerError()->message());
        } else if(err.token()) {
            std::println(stderr, "{}: {}: {} '{}'", err.token()->location(), err.what(), err.token()->name(), err.token()->text());
        } else if(err.expression()) {
            std::println(stderr, "{}: {}", err.expression()->token().location(), err.what());
        } else {
            std::println(stderr, "{}", err.what());
        }

        std::println(stderr, "{}:{}:{}", err.sourceLocation().file_name(), err.sourceLocation().line(), err.sourceLocation().column());
        throw;
    }

    if(!program.statements()) {
        std::println("empty program");
        return 0;
    }

    ngc::Memory mem;
    const auto addrs = mem.init(ngc::VARS);
    const auto preamble = ngc::Preamble::make(ngc::VARS, addrs);

    // TODO: finish if, while, and return in SemanticAnalyzer
    ngc::SemanticAnalyzer sa;
    sa.addGlobalSub(ngc::SubSignature("sin", 1));
    sa.processPreamble(preamble);
    sa.processProgram(program.statements());

    if(!sa.errors().empty()) {
        for(const auto &error : sa.errors()) {
            std::println(stderr, "{}", error.message());
        }

        return 1;
    }
}
