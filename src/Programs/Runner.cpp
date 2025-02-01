#include "Utils/DLHandler.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Logger.hpp"
#include "Utils/Misc.hpp"
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

using namespace Antipatrea;

int main(int argc, char **argv) {
    typedef int (*MainFcn)(int, char **);

    OutputFormat(Logger::m_out);

    RandomSeed();

    Logger::m_out << "Starting motion simulation" << std::endl;

    printf("Runner:\n");
    for (int i = 0; i < argc; ++i)
        printf("arg %d : <%s>\n", i, argv[i]);

    if (argc < 2)
        Logger::m_out << "usage: Runner <program_name> [list_of_program_args]" << std::endl;
    else {
        auto fcn = (MainFcn) Antipatrea::DLHandler::GetSymbol(argv[1]);

        if (fcn) {
            Logger::m_out << "Running program <" << argv[1] << ">" << std::endl << std::endl;
            return fcn(argc - 1, &(argv[1]));
        } else
            Logger::m_out << "Program <" << argv[1] << "> not found" << std::endl;
    }
}
