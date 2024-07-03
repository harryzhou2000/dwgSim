#pragma once

#include "dwgsimDefs.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <dwg_api.h>

#include <set>

class DwgSimReader
{
    rapidjson::Document doc;
    Dwg_Data dwg;
    int dwgError{ 0 };

public:
    DwgSimReader(const std::string& filename_in)
    {

        std::memset(&dwg, 0, sizeof(dwg));
        dwgError = dwg_read_file(filename_in.c_str(), &dwg);
        if (dwgError >= DWG_ERR_CRITICAL)
        {
            std::cerr << "DWG READ/DECODE ERROR 0x" << std::hex << dwgError << std::endl;
            throw std::runtime_error("dwg file read and decode error");
        }
        // dwg_api_init_version(&dwg);
    }

    void TraverseEntities(std::function<void(Dwg_Object*, int)> process_object)
    {
        auto process_BLOCK_HEADER = [&](Dwg_Object_Ref* ref, int space)
        {
            if (!ref)
                return;
            if (!ref->obj)
                return;
            Dwg_Object* obj = get_first_owned_entity(ref->obj);
            while (obj)
            {
                process_object(obj, space);
                obj = get_next_owned_entity(ref->obj, obj);
            }
        };

        process_BLOCK_HEADER(dwg_model_space_ref(&dwg), 0);
        process_BLOCK_HEADER(dwg_paper_space_ref(&dwg), 1);
        Dwg_Object_BLOCK_CONTROL* block_control = dwg_block_control(&dwg);
        for (int i = 0; i < block_control->num_entries; i++)
        {
            process_BLOCK_HEADER(block_control->entries[i], 2);
        }
    }

    void DebugPrint()
    {
        int64_t allObjCount{ 0 };
        std::set<void*> objs;
        auto printOne = [](auto v)
        { std::cout << v << std::endl; };
        TraverseEntities([&](Dwg_Object* obj, int space)
            {
                allObjCount++;
                objs.insert(obj);
                std::cout << std::hex << obj << " space: " << std::dec << space << std::endl;

                if (!obj || !obj->parent)
                    throw std::runtime_error("obj not valid");
                uint32_t type = obj->fixedtype;
                switch (type)
                {
                case DWG_TYPE_BLOCK:
                    printOne("BLOCK");
                    break;
                case DWG_TYPE_LINE:
                    printOne("LINE");
                    break;
                case DWG_TYPE_ARC:
                    printOne("ARC");
                    break;
                case DWG_TYPE_SPLINE:
                    printOne("SPLINE");
                    break;
                case DWG_TYPE_CIRCLE:
                    printOne("CIRCLE");
                    break;
                case DWG_TYPE_LWPOLYLINE:
                    printOne("LWPOLYLINE");
                    break;
                case DWG_TYPE_POLYLINE_2D:
                    printOne("POLYLINE_2D");
                    break;
                case DWG_TYPE_MTEXT:
                    printOne("MTEXT");
                    break;
                case DWG_TYPE_HATCH:
                    printOne("HATCH");
                    break;
                case DWG_TYPE_DIMENSION_LINEAR:
                    printOne("DIMENSION_LINEAR");
                    break;
                case DWG_TYPE_DIMENSION_ALIGNED:
                    printOne("DIMENSION_ALIGNED");
                    break;
                case DWG_TYPE_INSERT:
                    printOne("INSERT");
                    break;
                case DWG_TYPE_POINT:
                    printOne("POINT");
                    break;
                case DWG_TYPE_SOLID: //? 
                    printOne("SOLID");
                    break;
                case DWG_TYPE_ATTDEF: //?
                    printOne("ATTDEF");
                    break;
                default:
                {
                    if (type >= DWG_TYPE_ACDSRECORD)
                        break;
                    std::cout
                        << "UNKNOWN " << std::hex
                        << "0x" << type
                        << std::dec << std::endl;
                }
                break;
                } });
        std::cout << "=== " << allObjCount << ", " << objs.size() << std::endl;
    }

    ~DwgSimReader()
    {
        dwg_free(&dwg);
    }
};