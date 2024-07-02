#include "dwgsimDefs.h"


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
    Dwg_Data dwg;
    std::memset(&dwg, 0, sizeof(dwg));
    dwgError = dwg_read_file(filename_in.c_str(), &dwg);
    if (dwgError >= DWG_ERR_CRITICAL)
    {
        std::cerr << "READ ERROR 0x" << std::hex << dwgError << std::endl;
        throw std::runtime_error("file read error");
    }

    dwg_free(&dwg);

    return 0;
}