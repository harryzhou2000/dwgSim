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
        if (type == DWG_TYPE_SPLINE)
        {
            auto ent = dwg_object_to_SPLINE(obj);
            auto entP = ent;
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
                point.PushBack(ent->ctrl_pts[i].w, alloc);
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
            __FILL_RAPIDJSON_FIELD_INT(num_cols)
            __FILL_RAPIDJSON_FIELD_INT(num_rows)
            __FILL_RAPIDJSON_FIELD_DOUBLE(col_spacing)
            __FILL_RAPIDJSON_FIELD_DOUBLE(row_spacing)
            __FILL_RAPIDJSON_FIELD_VECTOR3(extrusion)
        }
    }
}