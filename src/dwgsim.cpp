#include "dwgsimDefs.h"
#include "dwgsimReader.h"
#include <fstream>

int main(int argc, char *argv[])
{
    int dwgError = 0;

    argparse::ArgumentParser argparser("dwgsim", DNDS_MACRO_TO_STRING(DWGSIM_CURRENT_COMMIT_HASH));
    argparser.add_argument("input").help("path to the dwg input");
    argparser.add_argument("-o").help("path to output");

    try
    {
        argparser.parse_args(argc, argv);
    }
    catch (const std::exception &err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << argparser;
        return 1;
    }

    std::cout << "reading from " << argparser.get("input") << std::endl;
    if (argparser.is_used("-o"))
        std::cout << "writing to" << argparser.get("-o") << std::endl;
    else
        std::cout << "writing to stdout" << std::endl;
    std::string filename_in = argparser.get("input");

    try
    {
        DwgSim::Reader reader(filename_in);
        // reader.DebugPrint();
        reader.DebugPrint1();

        reader.CollectModelSpaceEntities();
        reader.CollectBlockSpaceEntities();
        if (argparser.is_used("-o"))
        {
            auto o = std::ofstream(argparser.get("-o"));
            reader.PrintDoc(o, 2);
        }
        else 
            reader.PrintDoc(std::cout, 2);
    }
    catch (const std::exception &err)
    {
        std::cerr << err.what() << std::endl;
        return 2;
    }

    return 0;
}