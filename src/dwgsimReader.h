#pragma once

#include "dwgsimDefs.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <dwg_api.h>

#include <set>
#include <unordered_map>
#include <string>

namespace DwgSim
{

#define IS_FROM_TU_DWG(dwg) \
    (dwg->header.from_version >= R_2007) && !(dwg->opts & DWG_OPTS_IN)
    //! copied from libreDWG

    enum EntitySpaceType
    {
        ModelSpace = 0,
        PaperSpace = 1,
        BlockSpace = 2,
    };

    using ObjectName = std::string;

    const struct __ObjNameMapping
    {
        std::unordered_map<Dwg_Object_Type, ObjectName> map;
        __ObjNameMapping()
        {
#define __MAP_DWG_TYPE(NAME) map[DWG_TYPE_##NAME] = #NAME;

            __MAP_DWG_TYPE(BLOCK)
            __MAP_DWG_TYPE(LINE)
            __MAP_DWG_TYPE(ARC)
            __MAP_DWG_TYPE(SPLINE)
            __MAP_DWG_TYPE(CIRCLE)
            __MAP_DWG_TYPE(LWPOLYLINE)
            __MAP_DWG_TYPE(POLYLINE_2D)
            __MAP_DWG_TYPE(POLYLINE_3D)
            __MAP_DWG_TYPE(MTEXT)
            __MAP_DWG_TYPE(HATCH)
            __MAP_DWG_TYPE(DIMENSION_LINEAR)
            __MAP_DWG_TYPE(DIMENSION_ALIGNED)
            __MAP_DWG_TYPE(INSERT)
            __MAP_DWG_TYPE(POINT)
            __MAP_DWG_TYPE(SOLID)
            __MAP_DWG_TYPE(ATTDEF)

#undef __MAP_DWG_TYPE
        }
    } objNameMapping;

    class unhandled_class_error : public std::runtime_error
    {
    public:
        using t_base = std::runtime_error;
        using t_base::t_base;
    };

    class field_query_error : public std::runtime_error
    {
    public:
        using t_base = std::runtime_error;
        using t_base::t_base;
    };

    struct LayerRecord
    {
        size_t readId = 0;
        std::string name;
    };

    class Reader
    {
        rapidjson::Document doc;
        Dwg_Data dwg;
        int dwgError{0};
        std::map<BITCODE_RLL, LayerRecord> layerNames;

    public:
        Reader(const std::string &filename_in)
        {
            std::memset(&dwg, 0, sizeof(dwg));
            dwgError = dwg_read_file(filename_in.c_str(), &dwg);
            if (dwgError >= DWG_ERR_CRITICAL)
            {
                std::cerr << "DWG READ/DECODE ERROR 0x" << std::hex << dwgError << std::endl;
                throw std::runtime_error("dwg file read and decode error");
            }
            // dwg_api_init_version(&dwg);
            doc.SetObject();
        }

        void PrintDoc(std::ostream &o, int nIndent = 0)
        {
            rapidjson::OStreamWrapper osw(o);
            if (nIndent)
            {
                rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
                writer.SetIndent(' ', nIndent);
                doc.Accept(writer);
            }
            else
            {
                rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
                doc.Accept(writer);
            }
        }

        void recordLayerName(dwg_obj_ent *entGen)
        {
            auto layerId = entGen->layer->absolute_ref;
            if (layerNames.count(layerId))
                return;
            int err{0};
            auto layer_name = dwg_ent_get_layer_name(entGen, &err);
            if (err)
                throw field_query_error("dwg_ent_get_layer_name failed");
            if (layer_name)
                layerNames[layerId] = LayerRecord{layerNames.size(), layer_name};
            else
                layerNames[layerId] = LayerRecord{layerNames.size(), "UNKNOWN_LAYER"};
            if (IS_FROM_TU_DWG((&dwg)) && std::string(layer_name) != "0") //!
                free(layer_name);
        }

        void TraverseEntities(std::function<void(Dwg_Object *, EntitySpaceType)> process_object);

        void TraverseEntitiesInSpace(
            std::function<void(Dwg_Object *, Dwg_Object *, const ObjectName &, Dwg_Object_Type)> process_object,
            EntitySpaceType space);

        void DebugPrint()
        {
            int64_t allObjCount{0};
            std::set<void *> objs;
            auto printOne = [](auto v)
            { std::cout << v << std::endl; };
            TraverseEntities(
                [&](Dwg_Object *obj, EntitySpaceType space)
                {
                    allObjCount++;
                    objs.insert(obj);
                    std::cout << std::hex << obj << " space: " << std::dec << space << std::endl;
                    if (!obj || !obj->parent)
                        throw std::runtime_error("obj not valid");
                    uint32_t type = obj->fixedtype;
                    if (objNameMapping.map.count((Dwg_Object_Type)type))
                        printOne(objNameMapping.map.at((Dwg_Object_Type)type));
                    else
                    {
                        if (type >= DWG_TYPE_ACDSRECORD)
                            return;
                        std::cout
                            << "UNKNOWN " << std::hex
                            << "0x" << type
                            << std::dec << std::endl;
                    }
                });
            std::cout << "=== " << allObjCount << ", " << objs.size() << std::endl;
        }

        void DebugPrint1()
        {
            TraverseEntitiesInSpace(
                [&](Dwg_Object *blk_obj, Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type)
                {
                    int err{0};
                    auto entGen = dwg_object_to_entity(obj, &err);
                    if (err)
                        throw field_query_error("dwg_object_to_entity failed");
                    recordLayerName(entGen);
                    if (type == DWG_TYPE_LINE)
                    {
                        auto ent = dwg_object_to_LINE(obj);
                        std::cout << layerNames.at(entGen->layer->absolute_ref).name << std::endl;
                        std::cout << ent->start.x << "," << ent->start.y << "," << ent->start.z << ", ";
                        std::cout << ent->end.x << "," << ent->end.y << "," << ent->end.z << std::endl;
                    }
                },
                ModelSpace);
        }


        void CollectModelSpaceEntities()
        {
            {
                rapidjson::Value modelSpaceArr(rapidjson::kArrayType);
                doc.AddMember("modelSpaceEntities", modelSpaceArr, doc.GetAllocator());
            }
            auto &modelSpaceArr = doc["modelSpaceEntities"];
            auto process_object = [&](Dwg_Object *blk_obj, Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type)
            {
                rapidjson::Value entJson(rapidjson::kObjectType);
                fillEntityJson(obj, name, type, entJson);
                modelSpaceArr.PushBack(entJson, doc.GetAllocator());
            };
            TraverseEntitiesInSpace(process_object, ModelSpace);
        }

        void CollectBlockSpaceEntities()
        {
            {
                rapidjson::Value blocks(rapidjson::kObjectType);
                doc.AddMember("blocks", blocks, doc.GetAllocator());
            }
            auto &blocks = doc["blocks"];

            auto process_object = [&](Dwg_Object *blk_obj, Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type)
            {
                int err{0};
                auto blk_id = blk_obj->handle.value;
                auto blk_id_name = std::to_string(blk_id);
                if (!blocks.HasMember(blk_id_name.c_str()))
                {
                    rapidjson::Value blk_id_nameJson;
                    blk_id_nameJson.SetString(blk_id_name.c_str(), doc.GetAllocator());
                    blocks.AddMember(blk_id_nameJson, rapidjson::Value(rapidjson::kObjectType), doc.GetAllocator());
                    auto &blockJson = blocks[blk_id_name.c_str()];
                    auto block_hdr = dwg_object_to_BLOCK_HEADER(blk_obj);
                    blockJson.AddMember("blkisxref", block_hdr->blkisxref, doc.GetAllocator());
                    blockJson.AddMember("id", blk_id, doc.GetAllocator());
                    char *blockName = dwg_obj_block_header_get_name(block_hdr, &err);
                    {
                        rapidjson::Value strJson;
                        strJson.SetString((blockName), ((int)std::strlen(blockName)), doc.GetAllocator());
                        blockJson.AddMember("name", strJson, doc.GetAllocator());
                    }
                    if (IS_FROM_TU_DWG((&dwg)))
                        free(blockName);

                    blockJson.AddMember("entities", rapidjson::kArrayType, doc.GetAllocator());
                }
                auto &blockJson = blocks[blk_id_name.c_str()];
                auto &blockEntJson = blockJson["entities"];

                rapidjson::Value entJson(rapidjson::kObjectType);
                fillEntityJson(obj, name, type, entJson);
                blockEntJson.PushBack(entJson, doc.GetAllocator());
            };
            TraverseEntitiesInSpace(process_object, BlockSpace);
        }

        void fillEntityJson(Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type, rapidjson::Value &entJson);

        ~Reader()
        {
            dwg_free(&dwg);
        }
    };
}