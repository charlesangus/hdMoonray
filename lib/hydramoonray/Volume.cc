// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

#include "Volume.h"
#include "RenderDelegate.h"
#include "HdmLog.h"
#include "pxr/usd/sdf/types.h" // for SdfAssetPath
#include <scene_rdl2/scene/rdl2/Geometry.h>
#include <algorithm>
#include <iostream>

using namespace pxr;

namespace {
    
const std::string rdlClassVdb("VdbGeometry");
const std::string rdlAttrModel("model");

}

namespace hdMoonray {

using scene_rdl2::logging::Logger;

Volume::Volume(SdfPath const& id) :
    HdVolume(id), 
    GeometryMixin(this) 
{}

void
Volume::Finalize(HdRenderParam* renderParam)
{
    // causes geometry to be hidden
    resetGeometryObject(RenderDelegate::get(renderParam));
}

HdDirtyBits
Volume::GetInitialDirtyBitsMask() const
{
    return HdChangeTracker::AllDirty;
}

void 
Volume::syncAttributes(HdSceneDelegate* sceneDelegate, 
                            RenderDelegate& renderDelegate,
                            HdDirtyBits* dirtyBits,
                            const TfToken& reprToken)
{
    static const TfToken densityToken("density");
    static const TfToken emissionToken("emission");
    static const TfToken velocityToken("velocity");
    
    auto fields(sceneDelegate->GetVolumeFieldDescriptors(GetId()));
    mFieldIds.clear();
    for (auto& desc: fields) {
        mFieldIds.push_back(desc.fieldId);
    }
    const std::string* filePath = nullptr;
    for (auto& desc: fields) {
        const OpenVdbAsset* const field =
            dynamic_cast<OpenVdbAsset*>(sceneDelegate->GetRenderIndex().GetBprim(
                                            desc.fieldPrimType, desc.fieldId));
        if (!field) continue;
           
        if (desc.fieldName == densityToken ||
            desc.fieldName == emissionToken ||
            desc.fieldName == velocityToken) {
                
            const std::string attributeName = desc.fieldName.GetString() + "_grid";
            std::string grid(field->name().GetString());
            if (field->index() > 0) {
                grid += "[" + std::to_string(field->index()) + "]";
            }
            geometry()->set(attributeName, grid);
                    
            const std::string& fp = field->filePath();
            if (not fp.empty()) {
                if (filePath && *filePath != fp) {
                    Logger::error(GetId(), " all fields must use same vdb file");
                    break;
                }
                filePath = &fp;
            }
            if (filePath) {
                geometry()->set(rdlAttrModel, *filePath);
            }
        }
    }

    // base class handles transform and double-sided
    GeometryMixin::syncAttributes(sceneDelegate, renderDelegate, dirtyBits, reprToken);
   
}

bool
Volume::usesField(const SdfPath& fieldId) const
{
    return std::find(mFieldIds.begin(), mFieldIds.end(), fieldId) != mFieldIds.end();
}

void
Volume::Sync(HdSceneDelegate* sceneDelegate,
             HdRenderParam*   renderParam,
             HdDirtyBits*     dirtyBits,
             const TfToken&   reprToken)
{
    hdmLogSyncStart("Volume", GetId(), dirtyBits);   
    RenderDelegate& renderDelegate(RenderDelegate::get(renderParam));
    
    _UpdateVisibility(sceneDelegate, dirtyBits);
    _UpdateInstancer(sceneDelegate, dirtyBits);
    
    syncAll(rdlClassVdb, sceneDelegate, renderDelegate, dirtyBits, reprToken);
    
    hdmLogSyncEnd(GetId());
}

void
OpenVdbAsset::Sync(HdSceneDelegate *sceneDelegate,
                   HdRenderParam   *renderParam,
                   HdDirtyBits     *dirtyBits)
{
    const TfToken fieldNameToken("fieldName");
    const TfToken fieldIndexToken("fieldIndex");

    const SdfPath& id = GetId();
    hdmLogSyncStart("OpenVdbAsset", id, dirtyBits);

    if (*dirtyBits & DirtyParams) {
        const std::string oldFilePath = mFilePath;
        const TfToken oldName = mName;
        const int oldIndex = mIndex;

        // An unauthored attribute still yields a typed (default) value, so an empty or
        // mistyped VtValue means the delegate has no data for the key; checked Gets on
        // it raise coding errors, so the previous value is kept instead. fieldIndex is
        // an attribute of the field like the others, not a light parameter.
        VtValue v;
        v = sceneDelegate->Get(id, HdFieldTokens->filePath);
        if (v.IsHolding<SdfAssetPath>()) {
            mFilePath = v.UncheckedGet<SdfAssetPath>().GetResolvedPath();
        }
        v = sceneDelegate->Get(id, fieldNameToken);
        if (v.IsHolding<TfToken>()) {
            mName = v.UncheckedGet<TfToken>();
        } else if (v.IsHolding<std::string>()) {
            mName = TfToken(v.UncheckedGet<std::string>());
        }
        v = sceneDelegate->Get(id, fieldIndexToken);
        mIndex = v.IsHolding<int>() ? v.UncheckedGet<int>() : -1;

        // Nothing marks a volume dirty when only its field changes (Storm adds that
        // dependency with a scene index of its own), and the volume's model is only
        // rewritten when the volume syncs, so a time-sampled filePath would never advance.
        if (mFilePath != oldFilePath || mName != oldName || mIndex != oldIndex) {
            HdChangeTracker& tracker = sceneDelegate->GetRenderIndex().GetChangeTracker();
            for (HdRprim* rprim : RenderDelegate::get(renderParam).volumes()) {
                if (static_cast<const Volume*>(rprim)->usesField(id)) {
                    tracker.MarkRprimDirty(rprim->GetId(), HdChangeTracker::DirtyVolumeField);
                }
            }
        }
    }
    *dirtyBits = Clean;
    hdmLogSyncEnd(id);
}

}
