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
            if (layerNames.count(entGen->layer->absolute_ref))
                return;
            int err{0};
            auto layer_name = dwg_ent_get_layer_name(entGen, &err);
            if (err)
                throw field_query_error("dwg_ent_get_layer_name failed");
            if (layer_name)
                layerNames[entGen->layer->absolute_ref] = LayerRecord{layerNames.size(), layer_name};
            else
                layerNames[entGen->layer->absolute_ref] = LayerRecord{layerNames.size(), "UNKNOWN_LAYER"};
            if (IS_FROM_TU_DWG((&dwg))) //!
                free(layer_name);
        }

        void TraverseEntities(std::function<void(Dwg_Object *, EntitySpaceType)> process_object)
        {
            auto process_BLOCK_HEADER = [&](Dwg_Object_Ref *ref, EntitySpaceType space)
            {
                if (!ref)
                    return;
                if (!ref->obj)
                    return;
                Dwg_Object *obj = get_first_owned_entity(ref->obj);
                while (obj)
                {
                    process_object(obj, space);
                    obj = get_next_owned_entity(ref->obj, obj);
                }
            };

            process_BLOCK_HEADER(dwg_model_space_ref(&dwg), ModelSpace);
            process_BLOCK_HEADER(dwg_paper_space_ref(&dwg), PaperSpace);
            Dwg_Object_BLOCK_CONTROL *block_control = dwg_block_control(&dwg);
            for (int i = 0; i < block_control->num_entries; i++)
                process_BLOCK_HEADER(block_control->entries[i], BlockSpace);
        }

        void TraverseEntitiesInSpace(
            std::function<void(Dwg_Object *, const ObjectName &, Dwg_Object_Type)> process_object,
            EntitySpaceType space)
        {
            auto process_object_type = [&](Dwg_Object *obj, EntitySpaceType space)
            {
                if (!obj || !obj->parent)
                    throw std::runtime_error("obj not valid");
                uint32_t type = obj->fixedtype;
                if (objNameMapping.map.count((Dwg_Object_Type)type))
                {
                    process_object(obj, objNameMapping.map.at((Dwg_Object_Type)type), (Dwg_Object_Type)type);
                }
                else
                {
                    if (type >= DWG_TYPE_ACDSRECORD)
                        return;
                    throw unhandled_class_error("DWG Class: " + std::to_string(type));
                }
            };
            auto process_BLOCK_HEADER = [&](Dwg_Object_Ref *ref, EntitySpaceType space)
            {
                if (!ref || !ref->obj)
                    return;
                Dwg_Object *obj = get_first_owned_entity(ref->obj);
                while (obj)
                {
                    process_object_type(obj, space);
                    obj = get_next_owned_entity(ref->obj, obj);
                }
            };

            if (space == ModelSpace)
                process_BLOCK_HEADER(dwg_model_space_ref(&dwg), ModelSpace);
            if (space == PaperSpace)
                process_BLOCK_HEADER(dwg_paper_space_ref(&dwg), PaperSpace);
            if (space == BlockSpace)
            {
                Dwg_Object_BLOCK_CONTROL *block_control = dwg_block_control(&dwg);
                for (int i = 0; i < block_control->num_entries; i++)
                    process_BLOCK_HEADER(block_control->entries[i], BlockSpace);
            }
        }

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
                [&](Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type)
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

#define __CREATE_RAPIDJSON_FIELD_VECTOR3(name)    \
                                                  \
    rapidjson::Value name(rapidjson::kArrayType); \
    name.Reserve(3, alloc);                       \
    name.PushBack(ent->name.x, alloc);            \
    name.PushBack(ent->name.y, alloc);            \
    name.PushBack(ent->name.z, alloc);

#define __FILL_RAPIDJSON_FIELD_VECTOR3(name)          \
    {                                                 \
        rapidjson::Value name(rapidjson::kArrayType); \
        name.Reserve(3, alloc);                       \
        name.PushBack(ent->name.x, alloc);            \
        name.PushBack(ent->name.y, alloc);            \
        name.PushBack(ent->name.z, alloc);            \
        entJson.AddMember(#name, name, alloc);        \
    }

#define __FILL_RAPIDJSON_FIELD_DOUBLE(name)         \
    {                                               \
        entJson.AddMember(#name, ent->name, alloc); \
    }
            auto traverseHandler = [&](Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type)
            {
                rapidjson::Document::AllocatorType &alloc = doc.GetAllocator();
                int err{0};
                auto entGen = dwg_object_to_entity(obj, &err);
                if (err)
                    throw field_query_error("dwg_object_to_entity failed");

                recordLayerName(entGen);

                rapidjson::Value entJson(rapidjson::kObjectType);
                entJson.AddMember("type", rapidjson::GenericStringRef<char>(name.c_str()), alloc);
                entJson.AddMember("layerId", (size_t)(entGen->layer->absolute_ref), alloc);

                if (type == DWG_TYPE_LINE)
                {
                    auto ent = dwg_object_to_LINE(obj);
                    __FILL_RAPIDJSON_FIELD_VECTOR3(start)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(end)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_ARC)
                {
                    auto ent = dwg_object_to_ARC(obj);
                    __FILL_RAPIDJSON_FIELD_VECTOR3(center)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(radius)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(start_angle)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(end_angle)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_CIRCLE)
                {
                    auto ent = dwg_object_to_CIRCLE(obj);
                    __FILL_RAPIDJSON_FIELD_VECTOR3(center)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(radius)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_ELLIPSE)
                {
                    auto ent = dwg_object_to_ELLIPSE(obj);
                    __FILL_RAPIDJSON_FIELD_VECTOR3(center)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(sm_axis)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(axis_ratio)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(start_angle)
                    __FILL_RAPIDJSON_FIELD_DOUBLE(end_angle)
                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_POLYLINE_3D)
                {
                    auto ent = dwg_object_to_POLYLINE_3D(obj);
                    auto entP = ent;

                    rapidjson::Value vertsJson(rapidjson::kArrayType);
                    rapidjson::Value bulgesJson(rapidjson::kArrayType);
                    vertsJson.Reserve(ent->num_owned, alloc);
                    bulgesJson.Reserve(ent->num_owned, alloc);
                    if (ent->has_vertex)
                    {
                        for (uint32_t i = 0; i < ent->num_owned; i++)
                        {
                            auto ent = dwg_object_to_VERTEX_3D(entP->vertex[i]->obj);
                            __CREATE_RAPIDJSON_FIELD_VECTOR3(point)
                            vertsJson.PushBack(point, alloc);
                            bulgesJson.PushBack(0, alloc);
                        }
                    }
                    entJson.AddMember("vertex", vertsJson, alloc);
                    entJson.AddMember("bulge", bulgesJson, alloc);

                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_POLYLINE_2D) // all 3D terms to 2D and adding bulges
                {
                    auto ent = dwg_object_to_POLYLINE_2D(obj);
                    auto entP = ent;

                    rapidjson::Value vertsJson(rapidjson::kArrayType);
                    rapidjson::Value bulgesJson(rapidjson::kArrayType);
                    vertsJson.Reserve(ent->num_owned, alloc);
                    bulgesJson.Reserve(ent->num_owned, alloc);
                    if (ent->has_vertex)
                    {
                        for (uint32_t i = 0; i < ent->num_owned; i++)
                        {
                            auto ent = dwg_object_to_VERTEX_2D(entP->vertex[i]->obj);
                            __CREATE_RAPIDJSON_FIELD_VECTOR3(point)
                            vertsJson.PushBack(point, alloc);
                            bulgesJson.PushBack(ent->bulge, alloc);
                        }
                    }
                    entJson.AddMember("vertex", vertsJson, alloc);
                    entJson.AddMember("bulge", bulgesJson, alloc);

                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }
                if (type == DWG_TYPE_LWPOLYLINE)
                {
                    auto ent = dwg_object_to_LWPOLYLINE(obj);
                    auto entP = ent;

                    rapidjson::Value vertsJson(rapidjson::kArrayType);
                    rapidjson::Value bulgesJson(rapidjson::kArrayType);
                    vertsJson.Reserve(ent->num_points, alloc);
                    bulgesJson.Reserve(ent->num_bulges, alloc);
                    for (uint32_t i = 0; i < ent->num_points; i++)
                    {
                        rapidjson::Value point(rapidjson::kArrayType);
                        point.Reserve(3, alloc);
                        point.PushBack(ent->points[i].x, alloc);
                        point.PushBack(ent->points[i].y, alloc);
                        vertsJson.PushBack(point, alloc);
                    }
                    for (uint32_t i = 0; i < ent->num_bulges; i++)
                    {
                        bulgesJson.PushBack(ent->bulges[i], alloc);
                    }

                    entJson.AddMember("vertex", vertsJson, alloc);
                    entJson.AddMember("bulge", bulgesJson, alloc);

                    __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
                }

                modelSpaceArr.PushBack(entJson, alloc);
            };
            TraverseEntitiesInSpace(traverseHandler, ModelSpace);
        }

        ~Reader()
        {
            dwg_free(&dwg);
        }
    };
}