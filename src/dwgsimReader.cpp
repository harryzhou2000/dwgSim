#include "dwgsimReader.h"

namespace DwgSim
{

    void Reader::TraverseEntities(std::function<void(Dwg_Object *, EntitySpaceType)> process_object)
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

    void Reader::TraverseEntitiesInSpace(
        std::function<void(Dwg_Object *, Dwg_Object *, const ObjectName &, Dwg_Object_Type)> process_object,
        EntitySpaceType space)
    {
        auto process_object_type = [&](Dwg_Object *blk_obj, Dwg_Object *obj, EntitySpaceType space)
        {
            if (!obj || !obj->parent)
                throw std::runtime_error("obj not valid");
            uint32_t type = obj->fixedtype;
            if (objNameMapping.map.count((Dwg_Object_Type)type))
            {
                process_object(blk_obj, obj, objNameMapping.map.at((Dwg_Object_Type)type), (Dwg_Object_Type)type);
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
                process_object_type(ref->obj, obj, space);
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

#define __OUTPUT_SUBCLASS_NAME(name) \
    o << "  100\n"                   \
      << #name << "\n";
#define __OUTPUT_ENTITY_VECTOR3(name, code0, codeJ) \
    for (int i = 0; i < 3; i++)                     \
        o << "  " << ##codeJ * i + ##code0 << "\n"  \
          << entJson[#name][i].GetDouble() << "\n";
#define __OUTPUT_ENTITY_STRING(name, code) \
    o << "  " << ##code << "\n"            \
      << entJson[#name].GetString() << "\n";
#define __OUTPUT_ENTITY_DOUBLE(name, code) \
    o << "  " << ##code << "\n"            \
      << entJson[#name].GetDouble() << "\n";
#define __OUTPUT_ENTITY_INT(name, code) \
    o << "  " << ##code << "\n"         \
      << entJson[#name].GetInt() << "\n";
#define __EXTRUSION_IS_FINITE()                           \
    (std::abs(entJson["extrusion"][0].GetDouble()) > 0 && \
     std::abs(entJson["extrusion"][1].GetDouble()) > 0 && \
     std::abs(entJson["extrusion"][2].GetDouble()) > 0)

    void Reader::PrintDocDXF(std::ostream &o)
    {
        using std::dec;
        using std::hex;
        using std::nouppercase;
        using std::uppercase;
        using namespace std::string_literals;
        const auto secStart = "  0\nSECTION\n";
        const auto secEnd = "  0\nENDSEC\n";
        o << "999\n";
        o << "dwgSim\n";
        o << std::setprecision(16);

        o << secStart;
        o << "  2\nHEADER\n";
        o << "  9\n$ACADVER\n";
        o << "  1\nAC1027\n";
        o << "  9\n$HANDSEED\n";
        o << "  5\n";
        o << hex << uppercase << dwg.header_vars.HANDSEED->handleref.value << dec << nouppercase << "\n";
        o << secEnd;

        o << secStart;
        o << "  2\nBLOCKS\n";
        for (auto it = doc["blocks"].MemberBegin(); it != doc["blocks"].MemberEnd(); ++it)
        {
            o << "  0\nBLOCK\n";
            o << "  5\n"
              << hex << uppercase << it->value["id"].GetUint64() << dec << nouppercase << "\n";
            __OUTPUT_SUBCLASS_NAME(AcDbEntity)
            auto &entJson = it->value;
            o << "  8\n0\n";
            __OUTPUT_SUBCLASS_NAME(AcDbBlockBegin)
            __OUTPUT_ENTITY_STRING(name, 2)
            __OUTPUT_ENTITY_INT(flag, 70)
            __OUTPUT_ENTITY_VECTOR3(base_pt, 10, 10)
            __OUTPUT_ENTITY_STRING(name, 3)
            if (entJson["blkisxref"].GetInt())
                throw std::runtime_error("external ref not considered");
            o << "  1\n\n";

            for (int64_t i = 0; i < (int64_t)entJson["entities"].Size(); i++)
            {
                auto &entJsonEnt = entJson["entities"][i];
                outEntityDXF(o, entJsonEnt);
            }

            o << "  0\nENDBLK\n";
            o << "  5\n"
              << hex << uppercase << it->value["endBlkId"].GetUint64() << dec << nouppercase << "\n";
            __OUTPUT_SUBCLASS_NAME(AcDbEntity)
            o << "  8\n0\n";
            __OUTPUT_SUBCLASS_NAME(AcDbBlockEnd)
        }
        o << secEnd;

        o << secStart;
        o << "  2\nENTITIES\n";
        for (int64_t i = 0; i < (int64_t)doc["modelSpaceEntities"].Size(); i++)
        {
            auto &entJson = doc["modelSpaceEntities"][i];
            outEntityDXF(o, entJson);
        }
        o << secEnd;

        o << "  0\nEOF\n";
    }

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
#define __FILL_RAPIDJSON_FIELD_INT(name)                     \
    {                                                        \
        entJson.AddMember(#name, int32_t(ent->name), alloc); \
    }
#define __FILL_RAPIDJSON_FIELD_STRING(str, len)     \
    {                                               \
        rapidjson::Value strJson;                   \
        strJson.SetString((##str), (##len), alloc); \
        entJson.AddMember(#str, strJson, alloc);    \
    }

    void Reader::fillEntityJson(Dwg_Object *obj, const ObjectName &name, Dwg_Object_Type type, rapidjson::Value &entJson)
    {
        rapidjson::Document::AllocatorType &alloc = doc.GetAllocator();
        int err{0};
        auto entGen = dwg_object_to_entity(obj, &err);
        if (err)
            throw field_query_error("dwg_object_to_entity failed");

        recordLayerName(entGen);

        entJson.AddMember("type", rapidjson::GenericStringRef<char>(name.c_str()), alloc);
        entJson.AddMember("handle", (size_t)(obj->handle.value), alloc);
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
            __FILL_RAPIDJSON_FIELD_INT(flag)

            rapidjson::Value vertsJson(rapidjson::kArrayType);
            rapidjson::Value bulgesJson(rapidjson::kArrayType);
            rapidjson::Value vertHandlesJson(rapidjson::kArrayType);
            vertsJson.Reserve(ent->num_owned, alloc);
            bulgesJson.Reserve(ent->num_owned, alloc);
            vertHandlesJson.Reserve(ent->num_owned, alloc);
            if (ent->has_vertex)
            {
                for (uint32_t i = 0; i < ent->num_owned; i++)
                {
                    auto ent = dwg_object_to_VERTEX_3D(entP->vertex[i]->obj);
                    __CREATE_RAPIDJSON_FIELD_VECTOR3(point)
                    vertsJson.PushBack(point, alloc);
                    bulgesJson.PushBack(0, alloc);
                    vertHandlesJson.PushBack(entP->vertex[i]->obj->handle.value, alloc);
                }
            }
            entJson.AddMember("vertex", vertsJson, alloc);
            entJson.AddMember("bulge", bulgesJson, alloc);
            entJson.AddMember("vertexHandles", vertHandlesJson, alloc);
            entJson.AddMember("seqendHandle", ent->seqend->absolute_ref, alloc);

            __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
        }
        if (type == DWG_TYPE_POLYLINE_2D) // all 3D terms to 2D and adding bulges
        {
            auto ent = dwg_object_to_POLYLINE_2D(obj);
            auto entP = ent;
            __FILL_RAPIDJSON_FIELD_INT(flag)

            rapidjson::Value vertsJson(rapidjson::kArrayType);
            rapidjson::Value bulgesJson(rapidjson::kArrayType);
            rapidjson::Value vertHandlesJson(rapidjson::kArrayType);
            vertsJson.Reserve(ent->num_owned, alloc);
            bulgesJson.Reserve(ent->num_owned, alloc);
            vertHandlesJson.Reserve(ent->num_owned, alloc);
            if (ent->has_vertex)
            {
                for (uint32_t i = 0; i < ent->num_owned; i++)
                {
                    auto ent = dwg_object_to_VERTEX_2D(entP->vertex[i]->obj);
                    __CREATE_RAPIDJSON_FIELD_VECTOR3(point)
                    vertsJson.PushBack(point, alloc);
                    bulgesJson.PushBack(ent->bulge, alloc);
                    vertHandlesJson.PushBack(entP->vertex[i]->obj->handle.value, alloc);
                }
            }
            entJson.AddMember("vertex", vertsJson, alloc);
            entJson.AddMember("bulge", bulgesJson, alloc);
            entJson.AddMember("vertexHandles", vertHandlesJson, alloc);
            entJson.AddMember("seqendHandle", ent->seqend->absolute_ref, alloc);

            __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
        }
        if (type == DWG_TYPE_LWPOLYLINE)
        {
            auto ent = dwg_object_to_LWPOLYLINE(obj);
            auto entP = ent;
            __FILL_RAPIDJSON_FIELD_INT(flag)

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
        if (type == DWG_TYPE_SPLINE)
        {
            auto ent = dwg_object_to_SPLINE(obj);
            auto entP = ent;
            __FILL_RAPIDJSON_FIELD_INT(flag)
            __FILL_RAPIDJSON_FIELD_INT(splineflags)
            __FILL_RAPIDJSON_FIELD_INT(periodic)
            __FILL_RAPIDJSON_FIELD_INT(rational)
            __FILL_RAPIDJSON_FIELD_INT(weighted)
            __FILL_RAPIDJSON_FIELD_INT(knotparam)
            __FILL_RAPIDJSON_FIELD_DOUBLE(ctrl_tol)
            __FILL_RAPIDJSON_FIELD_DOUBLE(fit_tol)
            __FILL_RAPIDJSON_FIELD_DOUBLE(knot_tol)
            __FILL_RAPIDJSON_FIELD_INT(degree)
            __FILL_RAPIDJSON_FIELD_VECTOR3(beg_tan_vec)
            __FILL_RAPIDJSON_FIELD_VECTOR3(end_tan_vec)

            rapidjson::Value ctrlPtsJson(rapidjson::kArrayType);
            rapidjson::Value fitPtsJson(rapidjson::kArrayType);
            rapidjson::Value knotsJson(rapidjson::kArrayType);
            ctrlPtsJson.Reserve(ent->num_ctrl_pts, alloc);
            fitPtsJson.Reserve(ent->num_fit_pts, alloc);
            knotsJson.Reserve(ent->num_knots, alloc);

            for (uint32_t i = 0; i < ent->num_ctrl_pts; i++)
            {
                rapidjson::Value point(rapidjson::kArrayType);
                point.Reserve(4, alloc);
                point.PushBack(ent->ctrl_pts[i].x, alloc);
                point.PushBack(ent->ctrl_pts[i].y, alloc);
                point.PushBack(ent->ctrl_pts[i].z, alloc);
                point.PushBack(ent->ctrl_pts[i].w, alloc); // weights here
                ctrlPtsJson.PushBack(point, alloc);
            }
            for (uint32_t i = 0; i < ent->num_fit_pts; i++)
            {
                rapidjson::Value point(rapidjson::kArrayType);
                point.Reserve(3, alloc);
                point.PushBack(ent->fit_pts[i].x, alloc);
                point.PushBack(ent->fit_pts[i].y, alloc);
                point.PushBack(ent->fit_pts[i].z, alloc);
                fitPtsJson.PushBack(point, alloc);
            }
            for (uint32_t i = 0; i < ent->num_knots; i++)
            {
                knotsJson.PushBack(ent->knots[i], alloc);
            }

            entJson.AddMember("ctrl_pts", ctrlPtsJson, alloc);
            entJson.AddMember("fit_pts", fitPtsJson, alloc);
            entJson.AddMember("knots", knotsJson, alloc);
        }
        if (type == DWG_TYPE_INSERT)
        {
            auto ent = dwg_object_to_INSERT(obj);
            entJson.AddMember("blockId", (size_t)(ent->block_header->absolute_ref), alloc);
            char *blockName = dwg_obj_block_header_get_name(dwg_object_to_BLOCK_HEADER(ent->block_header->obj), &err);
            __FILL_RAPIDJSON_FIELD_STRING(blockName, (int)std::strlen(blockName))
            if (IS_FROM_TU_DWG((&dwg)))
                free(blockName);
            __FILL_RAPIDJSON_FIELD_VECTOR3(ins_pt)
            __FILL_RAPIDJSON_FIELD_VECTOR3(scale)
            __FILL_RAPIDJSON_FIELD_DOUBLE(rotation)
            entJson.AddMember("num_cols", std::max(int(ent->num_cols), 1), alloc); //! libredwg sets this to 0! could be its bug
            entJson.AddMember("num_rows", std::max(int(ent->num_rows), 1), alloc);
            __FILL_RAPIDJSON_FIELD_DOUBLE(col_spacing)
            __FILL_RAPIDJSON_FIELD_DOUBLE(row_spacing)
            __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
        }
    }

    void Reader::outEntityDXF(std::ostream &o, rapidjson::Value &entJson)
    {
        using std::dec;
        using std::hex;
        using std::nouppercase;
        using std::uppercase;
        using namespace std::string_literals;
        auto &layersJson = doc["layers"];

        if (!objName2DxfNameMapping.map.count(entJson["type"].GetString()))
            return;

        o << "  0\n"
          << objName2DxfNameMapping.map.at(entJson["type"].GetString()) << "\n";
        o << "  5\n"
          << hex << uppercase << entJson["handle"].GetUint64() << dec << nouppercase << "\n";
        __OUTPUT_SUBCLASS_NAME(AcDbEntity)
        auto &layerJson = layersJson[std::to_string(entJson["layerId"].GetUint64()).c_str()];
        o << "  8\n"
          << layerJson["name"].GetString() << "\n";

        auto type = entJson["type"].GetString();
        if (type == "LINE"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbLine)
            __OUTPUT_ENTITY_VECTOR3(start, 10, 10)
            __OUTPUT_ENTITY_VECTOR3(end, 11, 10)
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "ARC"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbArc)
            __OUTPUT_ENTITY_VECTOR3(center, 10, 10)
            __OUTPUT_ENTITY_DOUBLE(radius, 40)
            o << "  " << 50 << "\n"
              << entJson["start_angle"].GetDouble() * 180. / pi << "\n";
            o << "  " << 51 << "\n"
              << entJson["end_angle"].GetDouble() * 180. / pi << "\n"; //! in degree
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "CIRCLE"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbCircle)
            __OUTPUT_ENTITY_VECTOR3(center, 10, 10)
            __OUTPUT_ENTITY_DOUBLE(radius, 40)
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "ELLIPSE"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbEllipse)
            __OUTPUT_ENTITY_VECTOR3(center, 10, 10)
            __OUTPUT_ENTITY_VECTOR3(sm_axis, 11, 10)
            __OUTPUT_ENTITY_DOUBLE(axis_ratio, 40)
            __OUTPUT_ENTITY_DOUBLE(start_angle, 41)
            __OUTPUT_ENTITY_DOUBLE(end_angle, 42) //! in radius
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "POLYLINE_3D"s || type == "POLYLINE_2D"s)
        {
            if (type == "POLYLINE_3D"s)
                __OUTPUT_SUBCLASS_NAME(AcDb3dPolyline)
            else
                __OUTPUT_SUBCLASS_NAME(AcDb2dPolyline)
            o << "  10\n0\n  20\n0\n";
            o << "  30\n"
              << 0 << "\n"; //? elevation is what
            __OUTPUT_ENTITY_INT(flag, 70)
            for (int64_t i = 0; i < (int64_t)entJson["vertex"].Size(); i++)
            {
                o << "  0\n"
                  << "VERTEX" << "\n";
                o << "  5 \n"
                  << hex << uppercase << entJson["vertexHandles"][i].GetUint64() << dec << nouppercase << "\n";
                __OUTPUT_SUBCLASS_NAME(AcDbEntity)
                o << "  8\n"
                  << layerJson["name"].GetString() << "\n"; // forcing to use polyline's layer
                __OUTPUT_SUBCLASS_NAME(AcDbVertex)
                if (type == "POLYLINE_3D"s)
                    __OUTPUT_SUBCLASS_NAME(AcDb3dPolylineVertex)
                else
                    __OUTPUT_SUBCLASS_NAME(AcDb2dVertex)
                for (int j = 0; j < 3; j++)
                    o << "  " << 10 * j + 10 << "\n"
                      << entJson["vertex"][i][j].GetDouble() << "\n";
                if (type == "POLYLINE_2D"s)
                    o << "  42\n"
                      << entJson["bulge"][i].GetDouble() << "\n";
                if (type == "POLYLINE_3D"s)
                    o << "  70\n32\n";
                else
                    o << "  70\n0\n"; //! not verified
            }
            o << "  0\n"
              << "SEQEND" << "\n";
            o << "  5 \n"
              << hex << uppercase << entJson["seqendHandle"].GetUint64() << dec << nouppercase << "\n";
            __OUTPUT_SUBCLASS_NAME(AcDbEntity)
            o << "  8\n"
              << layerJson["name"].GetString() << "\n"; // forcing to use polyline's layer
        }
        else if (type == "LWPOLYLINE"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbPolyline)
            o << "  90\n"
              << entJson["vertex"].Size() << "\n";
            __OUTPUT_ENTITY_INT(flag, 70)
            for (int64_t i = 0; i < (int64_t)entJson["vertex"].Size(); i++)
                o << "  10\n"
                  << entJson["vertex"][i][0].GetDouble() << "\n"
                  << "  20\n"
                  << entJson["vertex"][i][1].GetDouble() << "\n";
            for (int64_t i = 0; i < (int64_t)entJson["bulge"].Size(); i++)
                o << "  42\n"
                  << entJson["bulge"][i].GetDouble() << "\n";
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "SPLINE"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbSpline)
            o << "  210\n0\n  220\n0\n  230\n1\n"; // cant read extrusion, where?
            o << "  2\nANSI31\n";
            __OUTPUT_ENTITY_INT(flag, 70) // no using splineflags
            __OUTPUT_ENTITY_INT(degree, 71)
            o << "  72\n"
              << entJson["knots"].Size() << "\n";
            o << "  73\n"
              << entJson["ctrl_pts"].Size() << "\n";
            o << "  74\n"
              << entJson["fit_pts"].Size() << "\n";
            __OUTPUT_ENTITY_DOUBLE(knot_tol, 42)
            __OUTPUT_ENTITY_DOUBLE(ctrl_tol, 43)
            __OUTPUT_ENTITY_DOUBLE(fit_tol, 44)
            __OUTPUT_ENTITY_VECTOR3(beg_tan_vec, 12, 10)
            __OUTPUT_ENTITY_VECTOR3(end_tan_vec, 13, 10)

            for (int64_t i = 0; i < (int64_t)entJson["knots"].Size(); i++)
                o << "  40\n"
                  << entJson["knots"][i].GetDouble() << "\n";
            for (int64_t i = 0; i < (int64_t)entJson["ctrl_pts"].Size(); i++)
                for (int j = 0; j < 3; j++)
                    o << "  " << 10 * j + 10 << "\n"
                      << entJson["ctrl_pts"][i][j].GetDouble() << "\n";
            for (int64_t i = 0; i < (int64_t)entJson["fit_pts"].Size(); i++)
                for (int j = 0; j < 3; j++)
                    o << "  " << 10 * j + 11 << "\n"
                      << entJson["fit_pts"][i][j].GetDouble() << "\n";

            for (int64_t i = 0; i < (int64_t)entJson["ctrl_pts"].Size(); i++)
                o << "  " << 41 << "\n" // for weights
                  << entJson["ctrl_pts"][i][3].GetDouble() << "\n";
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else if (type == "INSERT"s)
        {
            __OUTPUT_SUBCLASS_NAME(AcDbBlockReference)
            __OUTPUT_ENTITY_STRING(blockName, 2)
            __OUTPUT_ENTITY_VECTOR3(ins_pt, 10, 10)
            __OUTPUT_ENTITY_VECTOR3(scale, 41, 1)
            __OUTPUT_ENTITY_DOUBLE(rotation, 50)
            __OUTPUT_ENTITY_INT(num_cols, 70)
            __OUTPUT_ENTITY_INT(num_rows, 71)
            __OUTPUT_ENTITY_DOUBLE(col_spacing, 44)
            __OUTPUT_ENTITY_DOUBLE(row_spacing, 45)
            if (__EXTRUSION_IS_FINITE())
                __OUTPUT_ENTITY_VECTOR3(extrusion, 210, 10)
        }
        else
            throw std::out_of_range("type not implemented for dxf out");
    }
}