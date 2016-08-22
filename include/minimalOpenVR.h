#pragma once
/**
\file minimalOpenGL/minimalOpenVR.h
\author Morgan McGuire, http://graphics.cs.williams.edu
Distributed with the G3D Innovation Engine http://g3d.cs.williams.edu

Minimal headers for including Valve's OpenVR / SteamVR API,
which currently supports Vive and Oculus Virtual Reality
head mounted displays (HMDs). This does not depend on any
vector math library or specific OpenGL intialization library.

This requires the bin, lib, and headers directories from the
OpenVR SDK (which are also distributed with G3D):

https://github.com/ValveSoftware/openvr

The runtime for OpenVR is distributed with Steam. Ensure that
you've run Steam and let it update to the latest SteamVR before
running an OpenVR program.
*/

#include "openvr/openvr.h"
#include <string>

#include <GL/glew.h>
#include "glm/glm.hpp"

//#ifdef _WIN
//#   pragma comment(lib, "openvr_api")
//#endif

glm::mat4x3 convert(const vr::HmdMatrix34_t &m) {
	return glm::mat4x3(
		m.m[0][0], m.m[1][0], m.m[2][0],
		m.m[0][1], m.m[1][1], m.m[2][1],
		m.m[0][2], m.m[1][2], m.m[2][2],
		m.m[0][3], m.m[1][3], m.m[2][3]);
}

glm::mat4 convert(const vr::HmdMatrix44_t &m) {
	return glm::mat4(
		m.m[0][0], m.m[1][0], m.m[2][0], m.m[3][0],
		m.m[0][1], m.m[1][1], m.m[2][1], m.m[3][1],
		m.m[0][2], m.m[1][2], m.m[2][2], m.m[3][2],
		m.m[0][3], m.m[1][3], m.m[2][3], m.m[3][3]);
}

glm::mat4 getProjectionMatrix(vr::IVRSystem* hmd, vr::Hmd_Eye nEye, float nearZ, float farZ)
{
	const vr::HmdMatrix44_t mat(hmd->GetProjectionMatrix(nEye, nearZ, farZ,
		vr::API_OpenGL));
	return convert(mat);
}

glm::mat4 getHeadToEyeTransform(vr::IVRSystem* hmd, vr::Hmd_Eye nEye) {
	return glm::inverse(glm::mat4(convert(hmd->GetEyeToHeadTransform(nEye))));
}

/** Called by initOpenVR */
std::string getHMDString(vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError* peError = nullptr) {
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, nullptr, 0, peError);
	if (unRequiredBufferLen == 0) {
		return "";
	}

	char* pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;

	return sResult;
}


/** Call immediately before initializing OpenGL

\param hmdWidth, hmdHeight recommended render target resolution
*/
vr::IVRSystem* initOpenVR(uint32_t& hmdWidth, uint32_t& hmdHeight) {
	vr::EVRInitError eError = vr::VRInitError_None;
	vr::IVRSystem* hmd = vr::VR_Init(&eError, vr::VRApplication_Scene);

	if (eError != vr::VRInitError_None) {
		fprintf(stderr, "OpenVR Initialization Error: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		return nullptr;
	}

	const std::string& driver = getHMDString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
	const std::string& model = getHMDString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String);
	const std::string& serial = getHMDString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
	const float freq = hmd->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);

	//get the proper resolution of the hmd
	hmd->GetRecommendedRenderTargetSize(&hmdWidth, &hmdHeight);

	fprintf(stderr, "HMD: %s '%s' #%s (%d x %d @ %g Hz)\n", driver.c_str(), model.c_str(), serial.c_str(), hmdWidth, hmdHeight, freq);

	// Initialize the compositor
	vr::IVRCompositor* compositor = vr::VRCompositor();
	if (!compositor) {
		fprintf(stderr, "OpenVR Compositor initialization failed. See log file for details\n");
		vr::VR_Shutdown();
		assert("VR failed" && false);
	}

	return hmd;
}


/**
*/
void getEyeTransformations
(vr::IVRSystem* hmd, vr::TrackedDevicePose_t* trackedDevicePose,
	int				numEyes,
	float           nearPlaneZ,
	float           farPlaneZ,
	glm::mat4*      projection,
	glm::mat4*      headToEye,
	glm::mat4      &bodyToHead) {

	vr::VRCompositor()->WaitGetPoses(trackedDevicePose, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
	assert(trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid);
	glm::mat4x3 headToBody = convert(trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);

	bodyToHead = glm::inverse(glm::mat4(headToBody));

	headToEye[0] = getHeadToEyeTransform(hmd, vr::Eye_Left);
	headToEye[1] = getHeadToEyeTransform(hmd, vr::Eye_Right);

	projection[0] = getProjectionMatrix(hmd, vr::Eye_Left, -nearPlaneZ, -farPlaneZ);
	projection[1] = getProjectionMatrix(hmd, vr::Eye_Right, -nearPlaneZ, -farPlaneZ);
}


/** Call immediately before OpenGL swap buffers */
void submitToHMD(GLint ltEyeTexture, GLint rtEyeTexture, bool isGammaEncoded) {
	const vr::EColorSpace colorSpace = isGammaEncoded ? vr::ColorSpace_Gamma : vr::ColorSpace_Linear;

	const vr::Texture_t lt = { reinterpret_cast<void*>(intptr_t(ltEyeTexture)), vr::API_OpenGL, colorSpace };
	vr::VRCompositor()->Submit(vr::Eye_Left, &lt);

	const vr::Texture_t rt = { reinterpret_cast<void*>(intptr_t(rtEyeTexture)), vr::API_OpenGL, colorSpace };
	vr::VRCompositor()->Submit(vr::Eye_Right, &rt);

	// Tell the compositor to begin work immediately instead of waiting for the next WaitGetPoses() call
	vr::VRCompositor()->PostPresentHandoff();
}
