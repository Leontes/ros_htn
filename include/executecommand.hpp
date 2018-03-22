#ifndef EXECUTECOMMAND_HPP
#define EXECUTECOMMAND_HPP


#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>


std::string exec(std::string cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}


#endif
