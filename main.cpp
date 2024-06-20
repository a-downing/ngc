#include <print>
#include <vector>
#include <queue>
#include <filesystem>

import utils;
import parser;
import memory;
import gcode;
import evaluator;
import machine;

int main(const int argc, const char **argv) {
    if(argc < 2) {
        std::println(stderr, "usage: {} <filename>", argv[0]);
        return 0;
    }

    std::vector<ngc::Program> programs;

    for(const auto &entry  : std::filesystem::directory_iterator("autoload")) {
        auto filename = entry.path().string();
        auto result = ngc::readFile(filename);

        if(!result) {
            throw result.error();
        }

        programs.emplace_back(ngc::LexerSource(std::move(*result), std::move(filename)));
    }

    const std::string filename = argv[1];
    auto result = ngc::readFile(filename);

    if(!result) {
        throw result.error();
    }

    programs.emplace_back(ngc::LexerSource(std::move(*result), filename));

    for(auto &program : programs) {
        try {
            program.compile();

            for(const auto &stmt : program.statements()) {
                //std::println("{}", stmt->text());
            }
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
    auto machine = ngc::Machine(mem);
    const auto preamble = ngc::buildPreamble(addrs);

    auto callback = [&machine] (const ngc::Block &block, ngc::Evaluator &eval) {
        if(block.blockDelete()) {
            //std::println("DELETED BLOCK: {}", block.statement()->text());
        } else{
            //std::println("BLOCK: {}", block.statement()->text());
        }

        ngc::GCodeState state;

        for(const auto &word : block.words()) {
            state.affectState(word);
        }

        if(state.modeToolChange()) {
            eval.call("_tool_change", state.T());
        }

        machine.executeBlock(block);
    };

    auto eval = ngc::Evaluator(mem, callback);

    std::println("first pass: preamble");
    eval.executeFirstPass(preamble);

    for(auto &program : programs) {
        std::println("first pass: {}", program.source().name());
        eval.executeFirstPass(program.statements());
    }

    for(auto &program : programs) {
        std::println("executing: {}", program.source().name());
        eval.executeSecondPass(program.statements());
    }

    for(const auto &cmd : machine.commands()) {
        std::println("command: {}", cmd->text());
    }
}
