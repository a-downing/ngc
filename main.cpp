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

    std::vector<ngc::Program> programs;

    for(const auto &entry  : std::filesystem::directory_iterator("autoload")) {
        auto filename = entry.path().string();
        auto text = ngc::readFile(filename);
        programs.emplace_back(ngc::CharacterSource(std::move(text), std::move(filename)));
    }

    const std::string filename = argv[1];
    programs.emplace_back(ngc::CharacterSource(ngc::readFile(filename), filename));

    for(auto &program : programs) {
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
    }

    ngc::Memory mem;
    const auto addrs = mem.init(ngc::VARS);
    const auto preamble = ngc::Preamble::make(ngc::VARS, addrs);

    ngc::SemanticAnalyzer sa;
    sa.addGlobalSub(ngc::SubSignature("sin", 1));
    sa.processPreamble(preamble);

    // kind of a hack, but good enough for now
    for(auto &program : programs) {
        sa.processProgram(program.statements(), true);
    }

    sa.clearErrors();

    for(auto &program : programs) {
        std::println("analyzing: {}", program.source().name());
        sa.processProgram(program.statements(), false);

        if(!sa.errors().empty()) {
            for(const auto &error : sa.errors()) {
                std::println(stderr, "{}", error.message());
            }

            return 1;
        }
    }
}
