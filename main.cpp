#include "GCode.h"
#include <print>
#include <filesystem>

#include <Utils.h>
#include <Token.h>
#include <Lexer.h>
#include <Parser.h>
#include <Memory.h>
#include <SemanticAnalyzer.h>
#include <Program.h>
#include <Evaluator.h>
#include <Preamble.h>
#include <Statement.h>

int main(const int argc, const char **argv) {
    if(argc < 2) {
        std::println(stderr, "usage: {} <filename>", argv[0]);
        return 0;
    }

    std::vector<ngc::Program> programs;

    for(const auto &entry  : std::filesystem::directory_iterator("autoload")) {
        auto filename = entry.path().string();
        auto text = ngc::readFile(filename);
        programs.emplace_back(ngc::LexerSource(std::move(text), std::move(filename)));
    }

    const std::string filename = argv[1];
    programs.emplace_back(ngc::LexerSource(ngc::readFile(filename), filename));

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
    const auto preamble = ngc::buildPreamble(addrs);

    // ngc::SemanticAnalyzer sa;
    // sa.addGlobalSub(ngc::SubSignature("sin", 1));
    // sa.processProgram(preamble);
    //
    // for(auto &program : programs) {
    //     std::println("analyzing: {}", program.source().name());
    //     sa.processProgram(program.statements());
    //
    //     if(!sa.errors().empty()) {
    //         for(const auto &error : sa.errors()) {
    //             std::println(stderr, "{}", error.message());
    //         }
    //
    //         return 1;
    //     }
    // }

    auto callback = [&] (std::queue<const ngc::BlockStatement *> &blocks) {
        ngc::MachineState machineState;

        std::println("CALLBACK: {} blocks", blocks.size());


        while(!blocks.empty()) {
            auto block = blocks.front();
            blocks.pop();
        }
    };

    auto eval = ngc::Evaluator(mem, callback);
    std::println("executing: preamble");
    eval.executeProgram(preamble);

    for(auto &program : programs) {
        std::println("executing: {}", program.source().name());
        eval.executeProgram(program.statements());
    }
}
