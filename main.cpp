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

    // TODO: make separate semantic analyzer that operates on parser output

    std::optional<std::unique_ptr<ngc::CompoundStatement>> statements;

    try {
        statements = parser.parse();
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

    if(!statements) {
        std::println("empty program");
        return 0;
    }

    std::println("program has {} statements", (*statements)->statements().size());
}
