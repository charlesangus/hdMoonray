// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

// Mesh is implemented using RdlMeshGeometry
// RDL attributes that don't have an equivalent in Mesh are set to
// an automatic default, but may be overridden by primvars of the form
// "moonray:xyz":
//  - mesh_resolution def: the natron:meshResolution render setting, 1 for a polygon mesh
//  - adaptive_error def: the natron:adaptiveError render setting; for a polygon mesh 0.0 or 1.0
//  - smooth_normal def: false
// Supports "uv" or "st" for texture coords, and 
// "normal" or "normals" for normals

// 
#include "Mesh.h"
#include "RenderDelegate.h"
#include "Material.h"
#include "HdmLog.h"
#include "MurmurHash3.h"
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec4d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/vt/array.h>
#include <vector>
#include <pxr/imaging/hd/meshUtil.h>
#include <pxr/imaging/pxOsd/tokens.h>
#include <scene_rdl2/scene/rdl2/Geometry.h>
#include <scene_rdl2/scene/rdl2/UserData.h>

#include <iostream>

using namespace pxr;

namespace {

// constants for RDL attrs and enums used herein
const std::string rdlClassMesh("RdlMeshGeometry");
const std::string rdlAttrFaceVertexCount("face_vertex_count");
const std::string rdlAttrVerticesByIndex("vertices_by_index");
const std::string rdlAttrOrientation("orientation");
constexpr int rdlOrientationRightHanded = 0;
constexpr int rdlOrientationLeftHanded = 1;
const std::string rdlAttrPartFaceCountList("part_face_count_list");
const std::string rdlAttrPartFaceIndices("part_face_indices");
const std::string rdlAttrPartList("part_list");
const std::string rdlAttrSmoothNormal("smooth_normal");
const std::string rdlAttrIsSubd("is_subd");
const std::string rdlAttrMeshResolution("mesh_resolution");
const std::string rdlAttrAdaptiveError("adaptive_error");
const std::string rdlAttrSubdScheme("subd_scheme");
constexpr int rdlSubdSchemeBilinear = 0;
constexpr int rdlSubdSchemeCatClark = 1;
const std::string rdlAttrSubdBoundary("subd_boundary");
constexpr int rdlSubdBoundaryNone = 0;
constexpr int rdlSubdBoundaryEdgeOnly = 1;
constexpr int rdlSubdBoundaryEdgeAndCorner = 2;
const std::string rdlAttrSubdFvarLinear("subd_fvar_linear");
constexpr int rdlSubdFvarLinearNone = 0;
constexpr int rdlSubdFvarLinearCornersOnly = 1;
constexpr int rdlSubdFvarLinearCornersPlus1 = 2;
constexpr int rdlSubdFvarLinearCornersPlus2 = 3;
constexpr int rdlSubdFvarLinearBoundaries = 4;
constexpr int rdlSubdFvarLinearAll = 5;
const std::string rdlAttrSubdCreaseIndices("subd_crease_indices");
const std::string rdlAttrSubdCreaseSharpnesses("subd_crease_sharpnesses");
const std::string rdlAttrSubdCornerIndices("subd_corner_indices");
const std::string rdlAttrSubdCornerSharpnesses("subd_corner_sharpnesses");
const std::string rdlAttrNormalList("normal_list");
const std::string rdlAttrUvList("uv_list");
}

namespace hdMoonray {

using scene_rdl2::logging::Logger;
using scene_rdl2::rdl2::IntVector;
using scene_rdl2::rdl2::FloatVector;
using scene_rdl2::rdl2::Vec2f;
using scene_rdl2::rdl2::Vec2fVector;
using scene_rdl2::rdl2::Vec3f;
using scene_rdl2::rdl2::Vec3fVector;

Mesh::Mesh(SdfPath const& id) :
    HdMesh(id), 
    GeometryMixin(this) 
{}

void
Mesh::Finalize(HdRenderParam* renderParam)
{
    // causes geometry to be hidden
    resetGeometryObject(RenderDelegate::get(renderParam));
}

HdDirtyBits
Mesh::GetInitialDirtyBitsMask() const
{
    int mask = HdChangeTracker::Clean
        | HdChangeTracker::InitRepr
        | HdChangeTracker::DirtyPrimID
        | HdChangeTracker::DirtyDisplayStyle
        | HdChangeTracker::DirtyPrimvar
        | HdChangeTracker::DirtyMaterialId
        | HdChangeTracker::DirtyTopology
        | HdChangeTracker::DirtyTransform
        | HdChangeTracker::DirtyVisibility
        | HdChangeTracker::DirtyDoubleSided
        | HdChangeTracker::DirtySubdivTags
        | HdChangeTracker::DirtyInstancer
        | HdChangeTracker::DirtyInstanceIndex
        | HdChangeTracker::DirtyCategories
        | HdChangeTracker::DirtyRepr;
    return (HdDirtyBits)mask;
}

HdDirtyBits
Mesh::_PropagateDirtyBits(HdDirtyBits bits) const
{
    return bits;
}

// This seems to be a GL thing to set up a shader. Sync is not called.
// This is only needed if Sync() looks at Repr. 
// Mesh does this to get flat vs smooth usdview render setting.
void
Mesh::_InitRepr(TfToken const &reprToken, HdDirtyBits *dirtyBits)
{
    // doing this seems to cause Sync() to be called:
    *dirtyBits |= HdChangeTracker::DirtyRepr;
}


scene_rdl2::rdl2::Geometry* 
Mesh::geometryForMeshLight(RenderDelegate& renderDelegate)
{
    // MeshLight(GeometryLight) may require the RDL2 geometry
    // object before the Mesh has synced. Since light syncs
    // are not run in parallel, this does not need to be
    // threadsafe
    if (mGeometry) return mGeometry;
    scene_rdl2::rdl2::SceneObject* object = renderDelegate.createSceneObject(rdlClassMesh, rprim.GetId());
    if (object) mGeometry = object->asA<scene_rdl2::rdl2::Geometry>();
    return mGeometry;
}

namespace {

template <typename T>
bool gatherArray(const VtValue& value, const std::vector<int>& indices, VtValue& out)
{
    if (!value.IsHolding<VtArray<T>>()) return false;
    const VtArray<T>& in = value.UncheckedGet<VtArray<T>>();
    VtArray<T> result(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        // a short array is left to the consumer's own size checks
        if (size_t(indices[i]) >= in.size()) return false;
        result[i] = in[indices[i]];
    }
    out = VtValue(std::move(result));
    return true;
}

}

bool
Mesh::updateHoles(const HdMeshTopology& topology)
{
    const VtIntArray& holes = topology.GetHoleIndices();
    const VtIntArray& counts = topology.GetFaceVertexCounts();
    std::vector<int> faceRemap;
    std::vector<int> keptFaces;
    std::vector<int> keptCorners;
    const bool hasHoles = !holes.empty();
    if (hasHoles) {
        std::vector<bool> isHole(counts.size(), false);
        for (int h : holes) {
            if (h >= 0 && size_t(h) < isHole.size()) isHole[h] = true;
        }
        faceRemap.assign(counts.size(), -1);
        int corner = 0;
        for (size_t f = 0; f < counts.size(); ++f) {
            if (!isHole[f]) {
                faceRemap[f] = int(keptFaces.size());
                keptFaces.push_back(int(f));
                for (int k = 0; k < counts[f]; ++k) keptCorners.push_back(corner + k);
            }
            corner += counts[f];
        }
    }
    const bool changed = hasHoles != mHasHoles || keptFaces != mKeptFaces;
    mHasHoles = hasHoles;
    mFaceRemap = std::move(faceRemap);
    mKeptFaces = std::move(keptFaces);
    mKeptCorners = std::move(keptCorners);
    return changed;
}

bool
Mesh::dropHoleEntries(const VtValue& value, const HdInterpolation& interp, VtValue& out) const
{
    if (!mHasHoles || value.IsEmpty()) return false;
    const std::vector<int>* indices = nullptr;
    if (interp == HdInterpolationUniform) indices = &mKeptFaces;
    else if (interp == HdInterpolationFaceVarying) indices = &mKeptCorners;
    else return false;

    return gatherArray<float>(value, *indices, out) ||
           gatherArray<double>(value, *indices, out) ||
           gatherArray<int>(value, *indices, out) ||
           gatherArray<unsigned int>(value, *indices, out) ||
           gatherArray<bool>(value, *indices, out) ||
           gatherArray<GfVec2f>(value, *indices, out) ||
           gatherArray<GfVec3f>(value, *indices, out) ||
           gatherArray<GfVec4f>(value, *indices, out) ||
           gatherArray<GfVec2d>(value, *indices, out) ||
           gatherArray<GfVec3d>(value, *indices, out) ||
           gatherArray<GfVec4d>(value, *indices, out) ||
           gatherArray<std::string>(value, *indices, out) ||
           gatherArray<TfToken>(value, *indices, out);
}

void
Mesh::syncTopology(const HdMeshTopology& topology)
{
    // copies face and subset (part) data to RDL
    // vertices (points) are copied by base GeometryMixin class

    const VtIntArray& faceVertexCounts = topology.GetFaceVertexCounts();
    const VtIntArray& faceVertexIndices = topology.GetFaceVertexIndices();
    if (mHasHoles) {
        IntVector counts;
        counts.reserve(mKeptFaces.size());
        for (int f : mKeptFaces) counts.push_back(faceVertexCounts[f]);
        IntVector indices;
        indices.reserve(mKeptCorners.size());
        for (int c : mKeptCorners) indices.push_back(faceVertexIndices[c]);
        geometry()->set(rdlAttrFaceVertexCount, std::move(counts));
        geometry()->set(rdlAttrVerticesByIndex, std::move(indices));
    } else {
        geometry()->set(rdlAttrFaceVertexCount,
                        IntVector(faceVertexCounts.begin(),
                                  faceVertexCounts.end()));
        geometry()->set(rdlAttrVerticesByIndex,
                        IntVector(faceVertexIndices.begin(),
                                  faceVertexIndices.end()));
    }

    TfToken orientation = topology.GetOrientation();
    int rdlOrientation = rdlOrientationRightHanded;
    if (orientation == PxOsdOpenSubdivTokens->leftHanded) {
        rdlOrientation = rdlOrientationLeftHanded;
    }
    geometry()->set(rdlAttrOrientation, rdlOrientation);

    // subsets (== rdl parts)
    const HdGeomSubsets& geomSubsets(topology.GetGeomSubsets());
    std::vector<int> partFaceCountList;
    std::vector<int> partFaceIndices;
    partFaceCountList.reserve(geomSubsets.size());
    partFaceIndices.reserve(geomSubsets.size()); // actually larger
    partList.clear();       partList.reserve(geomSubsets.size());
    partMaterials.clear();  partMaterials.reserve(geomSubsets.size());
    partPaths.clear();      partPaths.reserve(geomSubsets.size());
    for (const HdGeomSubset& geomSubset : geomSubsets) {
        const size_t first = partFaceIndices.size();
        for (int i : geomSubset.indices) {
            if (!mHasHoles) {
                partFaceIndices.push_back(i);
            } else if (i >= 0 && size_t(i) < mFaceRemap.size() && mFaceRemap[i] >= 0) {
                partFaceIndices.push_back(mFaceRemap[i]);
            }
        }
        partFaceCountList.push_back(int(partFaceIndices.size() - first));
        partList.push_back(geomSubset.id.GetName());
        partMaterials.push_back(geomSubset.materialId);
        partPaths.push_back(geomSubset.id);
    }
    geometry()->set(rdlAttrPartFaceCountList, partFaceCountList);
    geometry()->set(rdlAttrPartFaceIndices, partFaceIndices);
    geometry()->set(rdlAttrPartList, partList);
}

void
Mesh::syncSubdivScheme(const HdMeshTopology& topology,
                       HdSceneDelegate *sceneDelegate,
                       RenderDelegate& renderDelegate,
                       const TfToken& reprToken)
{
    // sets the subdivision scheme, and related options
    // resolution, adaptive_error and smooth normals

    TfToken subdScheme = topology.GetScheme();
    int rdlScheme = rdlSubdSchemeCatClark; // moonray default
    if (subdScheme == PxOsdOpenSubdivTokens->bilinear) rdlScheme = rdlSubdSchemeBilinear;
    geometry()->set(rdlAttrSubdScheme, rdlScheme);

    // subd is disabled if
    // - the repr has flat shading enabled
    // - the "forcePolgon" render setting is on, or
    // - the subd scheme is "none"
    bool disableSubd = _GetReprDesc(reprToken)[0].flatShadingEnabled ||
                       renderDelegate.getForcePolygon() ||
                       subdScheme == PxOsdOpenSubdivTokens->none;

    geometry()->set(rdlAttrIsSubd, !disableSubd);

    if (not disableSubd && subdScheme == PxOsdOpenSubdivTokens->loop) {
        Logger::warn(GetId(), ": Loop subdivision is rendered as Catmull-Clark");
    }

    // mesh resolution, adaptive error and smooth_normals can be overridden
    // by primvars, so check before overwriting
    static TfToken meshResolutionToken("moonray:mesh_resolution");
    static TfToken adaptiveErrorToken("moonray:adaptive_error");
    static TfToken smoothNormalToken("moonray:smooth_normal");

    // A polygon mesh keeps the values it has always had, so its tessellation for displacement
    // does not change with the subdivision settings.
    if (not isPrimvarUsed(meshResolutionToken)) {
        float rdlResolution = disableSubd ? 1.0f : renderDelegate.getMeshResolution();
        geometry()->set(rdlAttrMeshResolution, rdlResolution);
    }

    if (not isPrimvarUsed(adaptiveErrorToken)) {
        float adaptive_error = disableSubd ? (topology.IsEnabledAdaptive() ? 1.0f : 0.0f) :
                               renderDelegate.getAdaptiveError();
        geometry()->set(rdlAttrAdaptiveError, adaptive_error);
    }

    if (not isPrimvarUsed(smoothNormalToken)) {
        // polygons should not autogenerate smooth normals, per UsdGeomMesh doc
        if (disableSubd)
            geometry()->set(rdlAttrSmoothNormal, false);
    }
}

void
Mesh::syncSubdivTags(const PxOsdSubdivTags& tags)
{
    // sets RDL attrs from subdiv tags

    TfToken t = tags.GetVertexInterpolationRule();
    int rdlValue;
    if (t == PxOsdOpenSubdivTokens->none)          rdlValue = rdlSubdBoundaryNone;
    else if (t == PxOsdOpenSubdivTokens->edgeOnly) rdlValue = rdlSubdBoundaryEdgeOnly;
    else                                           rdlValue = rdlSubdBoundaryEdgeAndCorner;
    geometry()->set(rdlAttrSubdBoundary, rdlValue);

    t = tags.GetFaceVaryingInterpolationRule();
    if (t == PxOsdOpenSubdivTokens->none)              rdlValue = rdlSubdFvarLinearNone;
    else if (t == PxOsdOpenSubdivTokens->cornersOnly)  rdlValue = rdlSubdFvarLinearCornersOnly;
    else if (t == PxOsdOpenSubdivTokens->cornersPlus1) rdlValue = rdlSubdFvarLinearCornersPlus1;
    else if (t == PxOsdOpenSubdivTokens->cornersPlus2) rdlValue = rdlSubdFvarLinearCornersPlus2;
    else if (t == PxOsdOpenSubdivTokens->boundaries)   rdlValue =  rdlSubdFvarLinearBoundaries;
    else if (t == PxOsdOpenSubdivTokens->all)          rdlValue = rdlSubdFvarLinearAll;
    else                                               rdlValue = rdlSubdFvarLinearCornersOnly;
    geometry()->set(rdlAttrSubdFvarLinear, rdlValue);


    VtIntArray vi = tags.GetCreaseIndices();
    if (not vi.empty()) {
        // Hydra has connected strings of edges. Convert to RdlMesh individual edges
        VtIntArray lengths(tags.GetCreaseLengths());
        const size_t edges = vi.size() - lengths.size();
        VtFloatArray vf = tags.GetCreaseWeights();
        bool weightPerEdge = vf.size() >= edges;
        IntVector result(2 * edges);
        FloatVector weights(edges);
        size_t i = 0; // index into crease indices
        size_t edge = 0; // index into result
        for (size_t crease = 0; crease < lengths.size(); ++crease) {
            int n = lengths[crease];
            for (int k = 0; k + 1 < n; k++) {
                result[2*edge] = vi[i];
                result[2*edge + 1] = vi[i + 1];
                weights[edge] = weightPerEdge ? vf[edge] : vf[crease];
                ++i; ++edge;
            }
            ++i; // don't connect last point with first of next edge
        }
        geometry()->set(rdlAttrSubdCreaseIndices, result);
        geometry()->set(rdlAttrSubdCreaseSharpnesses, weights);
    } else {
        // doing this leaves it in the rdla data
        // geometry()->resetToDefault("subd_crease_indices");
        // geometry()->resetToDefault("subd_crease_sharpnesses");
    }

    vi = tags.GetCornerIndices();
    if (not vi.empty()) {
        geometry()->set(rdlAttrSubdCornerIndices, IntVector(&vi[0], &vi[0] + vi.size()));
        VtFloatArray vf = tags.GetCornerWeights();
        geometry()->set(rdlAttrSubdCornerSharpnesses, FloatVector(&vf[0], &vf[0] + vf.size()));
    } else {
        // geometry()->resetToDefault("subd_corner_indices");
        // geometry()->resetToDefault("subd_corner_sharpnesses");
    }
}

void
Mesh::primvarChanged(HdSceneDelegate *sceneDelegate, RenderDelegate& renderDelegate,
                     const TfToken& name,  const VtValue& rawValue,
                     const HdInterpolation& interp, const TfToken& role)
{
    // Overridden from GeometryMixin to sync primvars that drive RDL object
    // attributes. Will be called from syncAll().
    // RdlMeshGeometry supports st/uv, and normal/normals as
    // object attributes

    static const TfToken stToken("st");
    static const TfToken uvToken("uv");
    static const TfToken normalToken("normal");

    VtValue filtered;
    const VtValue& value = dropHoleEntries(rawValue, interp, filtered) ? filtered : rawValue;

    if (name == HdTokens->normals || name == normalToken) {
        if (value.IsEmpty()) {
            geometry()->resetToDefault(rdlAttrNormalList);
        } else if (value.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray& v = value.UncheckedGet<VtVec3fArray>();
            const Vec3f* p = reinterpret_cast<const Vec3f*>(&v[0]);
            Vec3fVector out(p, p + v.size());
            geometry()->set(rdlAttrNormalList, out);
        } 
    } else if (name == stToken || name == uvToken) {
        if (value.IsEmpty()) {
            geometry()->resetToDefault(rdlAttrUvList);
        } else {
            Vec2fVector out;
            if (value.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray& v = value.UncheckedGet<VtVec3fArray>();
                out.reserve(v.size());
                for (const auto& v3 : v) {
                    out.emplace_back(*reinterpret_cast<const Vec2f*>(&v3));
                }
            } else if (value.IsHolding<VtVec2fArray>()) {
                const VtVec2fArray& v = value.UncheckedGet<VtVec2fArray>();
                const Vec2f* p = reinterpret_cast<const Vec2f*>(&v[0]);
                out.assign(p, p + v.size());
            }
            geometry()->set(rdlAttrUvList, std::move(out));
        }
    } else {
        // allow GeometryMixin to handle all other cases
        GeometryMixin::primvarChanged(sceneDelegate, renderDelegate,
                                      name, value, interp, role);
    }
}

void
Mesh::syncCryptomatteUserData(RenderDelegate& renderDelegate)
{
    // adds additional UserData to support Cryptomatte render outputs
    if (!renderDelegate.getDisableRender()) return;
    std::string idName = renderDelegate.getDeepIdAttrName();
    if (idName.empty()) return;

    std::string prim_id = geometry()->getName() + ".primvars:" + idName;

    scene_rdl2::rdl2::UserData* cryptoId =
            renderDelegate.createSceneObject("UserData", prim_id)->asA<scene_rdl2::rdl2::UserData>();
    {
        UpdateGuard guard(cryptoId);
        if (!partList.empty()){
            scene_rdl2::rdl2::FloatVector data;
            data.reserve(partList.size());

            for (auto& part: partList){
                data.emplace_back(MurmurHash3_to_float(part.c_str()));
            }
            cryptoId->setFloatData(idName, data);

        } else {
            scene_rdl2::rdl2::FloatVector data(1);
            data[0]=MurmurHash3_to_float(geometry()->getName().c_str());
            cryptoId->setFloatData(idName, data);

        }

    }
    addUserData(pxr::TfToken("primvars:"+idName),cryptoId);
}

void
Mesh::syncAttributes(HdSceneDelegate* sceneDelegate, 
                            RenderDelegate& renderDelegate,
                            HdDirtyBits* dirtyBits,
                            const TfToken& reprToken) 
{
    // Overridden from GeometryMixin to sync additional data in the
    // Mesh schema to RDL attributes. Will be called from syncAll()
    
    const SdfPath& id = GetId();

    if (HdChangeTracker::IsDisplayStyleDirty(*dirtyBits, id) ||
        HdChangeTracker::IsReprDirty(*dirtyBits, id) ||
        HdChangeTracker::IsTopologyDirty(*dirtyBits, id) ||
        HdChangeTracker::IsSubdivTagsDirty(*dirtyBits, id)) {
            
        const HdMeshTopology& topology = GetMeshTopology(sceneDelegate);
        if (HdChangeTracker::IsTopologyDirty(*dirtyBits, id)) {
            // handles faces and GeomSubsets (verts handled by base class)
            syncTopology(topology);
            // if parts change, need to generate user data that drives cryptomattes
            syncCryptomatteUserData(renderDelegate);
        }
        // choose and set the subdiv scheme
        syncSubdivScheme(topology, sceneDelegate, renderDelegate, reprToken);        

        if (HdChangeTracker::IsSubdivTagsDirty(*dirtyBits, id)) {
            // tags specify interpolation, creases and corners
            syncSubdivTags(sceneDelegate->GetSubdivTags(id));
        }
    }

    // base class handles transform and double-sided
    GeometryMixin::syncAttributes(sceneDelegate, renderDelegate, dirtyBits, reprToken);
}

void
Mesh::Sync(HdSceneDelegate *sceneDelegate,
           HdRenderParam   *renderParam,
           HdDirtyBits     *dirtyBits,
           TfToken const   &reprToken)
{
    hdmLogSyncStart("Mesh", GetId(), dirtyBits);
    RenderDelegate& renderDelegate(RenderDelegate::get(renderParam));
    
    _UpdateVisibility(sceneDelegate, dirtyBits);
    _UpdateInstancer(sceneDelegate, dirtyBits);

    // Primvars sync before topology, so the dropped faces must be known first, and
    // primvars whose face lists changed with them must be re-read.
    if (HdChangeTracker::IsTopologyDirty(*dirtyBits, GetId()) &&
        updateHoles(GetMeshTopology(sceneDelegate))) {
        *dirtyBits |= HdChangeTracker::DirtyPrimvar;
    }

    syncAll(rdlClassMesh, sceneDelegate, renderDelegate, dirtyBits, reprToken);

    hdmLogSyncEnd(GetId());
}
 
}

