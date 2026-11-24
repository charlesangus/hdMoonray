// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

#include "RenderPass.h"
#include "RenderBuffer.h"
#include "RenderDelegate.h"
#include "Camera.h"

#include <pxr/imaging/hd/renderPassState.h>
#include <pxr/base/gf/vec2f.h>
#include <scene_rdl2/scene/rdl2/Utils.h>

#include <iostream>
#include <chrono>
#include <optional>

//#define DEBUG_MSG


namespace hdMoonray
{

using scene_rdl2::logging::Logger;

RenderPass::~RenderPass()
{
}

bool
RenderPass::IsConverged() const
{
    // This oddness is to work around a Hydra bug that has been reported to pixar.
    // It does not call RenderBuffer::Resolve after IsConverged returns true, so
    // it never shows the last generated image. Fix this by requiring IsConverged()
    // to be called twice to return true and disable the _Execute call between them.
    if (mDeferIsConverged) {
        return true;
    } else {
        mDeferIsConverged = renderDelegate.renderer().isFrameComplete();
        return false;
    }
}

void
RenderPass::_MarkCollectionDirty()
{
}

void
RenderPass::_Sync()
{
}

void
RenderPass::_Execute(const pxr::HdRenderPassStateSharedPtr& renderPassState,
                     const pxr::TfTokenVector& renderTags)
{
    // Update for any changes in render settings, may create a new renderer
    renderDelegate.getRendererApplySettings();

    // Deal with changes to "purpose"
    renderDelegate.setRenderTags(GetRenderIndex(), renderTags);

    const scene_rdl2::rdl2::SceneContext& sc(renderDelegate.sceneContext());
    const scene_rdl2::rdl2::SceneVariables& sv(sc.getSceneVariables());

    int w, h;
    double aspectRatio;
    float pixelAspectRatio = 1.0f;
#if PXR_VERSION >= 2102
    if (renderPassState->GetFraming().IsValid()) {
        const pxr::CameraUtilFraming& framing(renderPassState->GetFraming());
        const pxr::GfRect2i& dw(framing.dataWindow);
        w = dw.GetWidth();
        h = dw.GetHeight();
        // the filmback maps to the display window, which an overscanned or
        // cropped data window does not match
        const pxr::GfVec2f displaySize = framing.displayWindow.GetSize();
        aspectRatio = displaySize[1] > 0.0f ? double(displaySize[0]) / displaySize[1] : double(w) / h;
        if (framing.pixelAspectRatio > 0.0f) pixelAspectRatio = framing.pixelAspectRatio;
    } else // older clients may use viewport instead of framing
#endif
    {   const pxr::GfVec4f& vp = renderPassState->GetViewport();
        w = vp[2];
        h = vp[3];
        aspectRatio = double(w) / h;
    }

    const Camera* camera(dynamic_cast<const Camera*>(renderPassState->GetCamera()));
    if (not camera) {
        Logger::error("RenderPassState without camera is unsupported");
        // It could the view+proj matricies below and update a fake Camera. But
        // usdview is not using this.
        return;
    }
    const_cast<Camera*>(camera)->setAsPrimaryCamera(renderDelegate, aspectRatio, pixelAspectRatio,
                                                    renderPassState->GetWindowPolicy());

    float frame = 0;
    scene_rdl2::rdl2::FloatVector motionSteps(2);
    auto usdDelegate = renderDelegate.usdImagingDelegate();
    if (usdDelegate) {
        pxr::UsdTimeCode currTime = usdDelegate->GetTime();
        if (currTime.IsNumeric()) {
            frame = (float)currTime.GetValue();
            pxr::GfInterval interval = usdDelegate->GetCurrentTimeSamplingInterval();
            motionSteps[0] = float(interval.GetMin()) - frame;
            motionSteps[1] = float(interval.GetMax()) - frame;
        }
    } else {
        motionSteps[0] = float(camera->GetShutterOpen());
        motionSteps[1] = float(camera->GetShutterClose());
        const pxr::GfVec2f window = motionSteps[0] < motionSteps[1] ?
            pxr::GfVec2f(motionSteps[0], motionSteps[1]) : pxr::GfVec2f(0.0f, 0.0f);
        if (renderDelegate.resampleMotionIfWindowChanged(camera->GetId(), window)) {
            // rprims sync again on the next execute, which must come before
            // the render reports convergence
            mDeferIsConverged = false;
        }
    }

    // Velocities are per second and MoonRay scales them by fps; an explicit fps
    // scene variable in the render settings wins over the stage's rate.
    float fps = sv.get(sv.sFpsKey);
    static const pxr::TfToken fpsSetting("moonray:sceneVariable:fps");
    static const pxr::TfToken fpsSettingAlt("sceneVariable_fps");
    if (renderDelegate.GetRenderSetting(fpsSetting).IsEmpty() &&
        renderDelegate.GetRenderSetting(fpsSettingAlt).IsEmpty()) {
        if (std::optional<double> tcps = renderDelegate.sceneTimeCodesPerSecond()) {
            fps = float(*tcps);
        }
    }

    const bool motionBlur = renderDelegate.getEnableMotionBlur() &&
                            motionSteps[0] < motionSteps[1];
    if (not motionBlur) { motionSteps[0] = -1; motionSteps[1] = 0; }

    if (frame != sv.get(sv.sFrameKey) ||
        fps != sv.get(sv.sFpsKey) ||
        motionSteps != sv.get(sv.sMotionSteps) ||
        motionBlur != sv.get(sv.sEnableMotionBlur) ||
        w != sv.get(sv.sImageWidth) ||
        h != sv.get(sv.sImageHeight))
    {
        scene_rdl2::rdl2::SceneVariables& wsv(renderDelegate.acquireSceneContext().getSceneVariables());
        UpdateGuard guard(wsv);
        wsv.set(sv.sFrameKey, frame);
        wsv.set(sv.sFpsKey, fps);
        wsv.set(sv.sMotionSteps, motionSteps);
        wsv.set(sv.sEnableMotionBlur, motionBlur);
        wsv.set(sv.sImageWidth, w);
        wsv.set(sv.sImageHeight, h);
    }

    const pxr::SdfPath& dicingPath = renderDelegate.getDicingCamera();
    scene_rdl2::rdl2::SceneObject* dicingCamera = nullptr;
    if (not dicingPath.IsEmpty()) {
        dicingCamera = Camera::createCamera(camera->getSceneDelegate(), renderDelegate, dicingPath);
        if (not dicingCamera && dicingPath != mWarnedDicingCamera) {
            Logger::warn(dicingPath, ": dicing camera not found, dicing from the render camera");
            mWarnedDicingCamera = dicingPath;
        }
    }
    if (dicingCamera != sv.get(sv.sDicingCamera)) {
        scene_rdl2::rdl2::SceneVariables& wsv(renderDelegate.acquireSceneContext().getSceneVariables());
        UpdateGuard guard(wsv);
        wsv.set(sv.sDicingCamera, dicingCamera);
    }

    // const pxr::GfMatrix4d& view = renderPassState->GetWorldToViewMatrix(); // same as camera->GetViewMatrix()
    // // This matrix contains info that is *not* in the Camera, to make the pixels square in the viewport:
    // const pxr::GfMatrix4d& proj = renderPassState->GetProjectionMatrix();

    // tell renderer about any new bindings
    const pxr::HdRenderPassAovBindingVector& aovBindings = renderPassState->GetAovBindings();
    for (const pxr::HdRenderPassAovBinding& aovBinding : aovBindings) {
        RenderBuffer* buffer = reinterpret_cast<RenderBuffer*>(aovBinding.renderBuffer);
        buffer->bind(aovBinding, camera);      
    }

    if (renderDelegate.renderer().isUpdateActive()) {      
        mDeferIsConverged = false;
    }
    renderDelegate.renderer().endUpdate();

    static std::string prevRdlaOutput;
    const std::string& rdlOutput(renderDelegate.rdlOutput());
    if (rdlOutput != prevRdlaOutput) {
        prevRdlaOutput = rdlOutput;
        if (not rdlOutput.empty()) {
            scene_rdl2::rdl2::writeSceneToFile(sc,
                                          rdlOutput,
                                          false, // deltas
                                          true); // skip defaults
        }
    }
}

}

