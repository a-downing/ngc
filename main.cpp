#include <print>
#include <fstream>
#include <filesystem>

#include <Lexer.h>

#include "Parser.h"

std::string readFile(const std::filesystem::path& filePath) {
    std::ifstream file(filePath);

    if (!file) {
        throw std::ios_base::failure("Failed to open file");
    }

    file.seekg(0, std::ios::end);
    const auto fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::string fileContent(fileSize, '\0');
    file.read(&fileContent[0], fileSize);

    return fileContent;
}

int main(int argc, char **argv) {
    const std::string filename = argv[1];
    const auto text = readFile(filename);
    auto source = ngc::CharacterSource(text, filename);
    auto lexer = ngc::Lexer(source);
    auto parser = ngc::Parser(lexer);

    // TODO: finish adding while and repeat loops
    // TODO: maybe add address of operator to enable pointers to named variables
    // TODO: add basic semantic analysis to parser like use before assignment of named variables
    // TODO: make separate g-code semantic analyzer that operates on parser output


    try {
        parser.parse();
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
    }
}
