
#include "dwgsimDefs.h"
#include "dwgsimReader.h"
#include "splineUtil.h"
#include <fstream>

int main(int argc, char *argv[])
{
    int dwgError = 0;

    int dupWarn = 0;
    int dupDel = 0;

    argparse::ArgumentParser argparser("dwgsim", DNDS_MACRO_TO_STRING(DWGSIM_CURRENT_COMMIT_HASH));
    argparser.add_argument("input").help("path to the dwg input");
    argparser.add_argument("-o").help("path to output");
    argparser.add_argument("-O").default_value("JSON").help("output format");
    argparser.add_argument("--dupWarn").default_value(0).store_into(dupWarn);
    argparser.add_argument("--dupDel").default_value(0).store_into(dupDel);
    argparser.add_argument("--clear").flag().help("clear stdout");

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

    if (argparser["--clear"] == false)
    {
        std::cout << "reading from: " << argparser.get("input") << std::endl;
        if (argparser.is_used("-o"))
            std::cout << "writing to: " << argparser.get("-o") << std::endl;
        else
            std::cout << "writing to stdout" << std::endl;
    }
    std::string filename_in = argparser.get("input");

    try
    {
        DwgSim::Reader reader(filename_in);
        // reader.DebugPrint();
        // reader.DebugPrint1();

        reader.CollectModelSpaceEntities();
        reader.CollectBlockSpaceEntities();
        reader.ReformSplines();
        reader.CleanLineEntityDuplication(1e-8, 1e-5, dupWarn, dupDel);

        if (argparser.get("-O") == "JSON")
        {
            if (argparser.is_used("-o"))
            {
                auto o = std::ofstream(argparser.get("-o"));
                reader.PrintDoc(o, 2);
            }
            else
                reader.PrintDoc(std::cout, 2);
        }
        else if (argparser.get("-O") == "DXF")
        {
            if (argparser.is_used("-o"))
            {
                auto o = std::ofstream(argparser.get("-o"));
                reader.PrintDocDXF(o);
            }
            else
                reader.PrintDocDXF(std::cout);
        }
        else
            throw std::runtime_error("no such -O format choice");
    }
    catch (const std::exception &err)
    {
        std::cerr << err.what() << std::endl;
        std::abort();
    }

    return 0;
}