/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 * Version 0.6b
 *
 * Copyright (c) 2013-2014 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Main file author: Erik Sund�n
 *
 *********************************************************************************/

#include "openglcapabilities.h"
#include "glwrap/textureunit.h"
#include <inviwo/core/util/formatconversion.h>
#include <inviwo/core/util/logcentral.h>
#include <inviwo/core/util/stringconversion.h>

namespace inviwo {

#define OpenGLInfoNotFound(message) { LogInfo(message << " Info could not be retrieved"); }

OpenGLCapabilities::GLSLShaderVersion::GLSLShaderVersion() : number_(0), profile_("") {}

OpenGLCapabilities::GLSLShaderVersion::GLSLShaderVersion(int num) : number_(num), profile_("") {}

OpenGLCapabilities::GLSLShaderVersion::GLSLShaderVersion(int num, std::string pro) : number_(num), profile_(pro) {}

std::string OpenGLCapabilities::GLSLShaderVersion::getProfile() {
    return profile_;
}

int OpenGLCapabilities::GLSLShaderVersion::getVersion() {
    return number_;
}

std::string OpenGLCapabilities::GLSLShaderVersion::getVersionAsString() {
    return toString<int>(number_);
}

std::string OpenGLCapabilities::GLSLShaderVersion::getVersionAndProfileAsString() {
    return (hasProfile() ? getVersionAsString() + " " + profile_ : getVersionAsString());
}

bool OpenGLCapabilities::GLSLShaderVersion::hasProfile() {
    return (profile_ != "");
}

OpenGLCapabilities::OpenGLCapabilities() {
    supportedShaderVersions_.clear();
    currentGlobalGLSLHeader_ = "";
    currentGlobalGLSLFragmentDefines_ = "";
    preferredGLSLProfile_ = "compatibility";
}

OpenGLCapabilities::~OpenGLCapabilities() {
    TextureUnit::deinitialize();
}

void OpenGLCapabilities::printInfo() {
    //OpenGL General Info
    LogInfo("Vendor: " << glVendorStr_);
    LogInfo("Renderer: " << glRenderStr_);
    LogInfo("Version: " << glVersionStr_);

    //GLSL
    if (isShadersSupported()) {
        LogInfo("GLSL version: " << glslVersionStr_);
        LogInfo("Current set global GLSL version: " << getCurrentShaderVersion().getVersionAndProfileAsString());
        LogInfo("Shaders supported: YES");
    }
    else if (isShadersSupportedARB()) {
        LogInfo("GLSL version: " << glslVersionStr_);
        LogInfo("Current set global GLSL version: " << getCurrentShaderVersion().getVersionAndProfileAsString());
        LogInfo("Shaders supported: YES(ARB)");
    }
    else
        LogInfo("Shaders supported: NO");

    LogInfo("Framebuffer objects supported: " << (isFboSupported() ? "YES" : "NO "));
    // Texturing
    LogInfo("1D/2D textures supported: " << (isTexturesSupported() ? "YES" : "NO "));
    LogInfo("3D textures supported: " << (is3DTexturesSupported() ? "YES" : "NO "));
    LogInfo("Array textures supported: " << (isTextureArraysSupported() ? "YES" : "NO "));

    if (isTexturesSupported()) {
        LogInfo("Max 1D/2D texture size: " << getMaxTexSize());
    }

    if (is3DTexturesSupported()) {
        LogInfo("Max 3D texture size: " << getMax3DTexSize());
    }

    if (isTextureArraysSupported()) {
        LogInfo("Max array texture size: " << getMaxArrayTexSize());
    }

    if (isFboSupported()) {
        LogInfo("Max color attachments: " << getMaxColorAttachments());
    }

    if (isTexturesSupported()) {
        LogInfo("Max number of texture units: " << getNumTexUnits());
        glm::u64 totalMem = getTotalAvailableTextureMem();
        LogInfo("Total available texture memory: " << (totalMem>0 ? formatBytesToString(totalMem) : "UNKNOWN"));
        glm::u64 curMem = getCurrentAvailableTextureMem();
        LogInfo("Current available texture memory: " << (curMem>0 ? formatBytesToString(curMem) : "UNKNOWN"));
    }
}

bool OpenGLCapabilities::canAllocate(glm::u64 dataSize, glm::u8 percentageOfAvailableMemory) {
    return getCurrentAvailableTextureMem()*percentageOfAvailableMemory/100 >= dataSize;
}

uvec3 OpenGLCapabilities::calculateOptimalBrickSize(uvec3 dimensions, size_t formatSizeInBytes, glm::u8 percentageOfAvailableMemory) {
    uvec3 currentBrickDimensions = dimensions;

    while (!canAllocate(getMemorySizeInBytes(currentBrickDimensions, formatSizeInBytes), percentageOfAvailableMemory)) {
        int theMaxDim = (currentBrickDimensions.x > currentBrickDimensions.y ? (currentBrickDimensions.x > currentBrickDimensions.z ? 0 : 2) :
                         (currentBrickDimensions.y > currentBrickDimensions.z ? 1 : 2));

        if (currentBrickDimensions[theMaxDim] % 2 != 0)
            currentBrickDimensions[theMaxDim]++; //Make the dim we are dividing even

        currentBrickDimensions[theMaxDim] /= 2;
    }

    // adapt brick size according to maximum texture dimension
    unsigned int maxGPUTextureDim = static_cast<unsigned int>(getMaxTexSize());

    while (currentBrickDimensions.x>maxGPUTextureDim || currentBrickDimensions.y>maxGPUTextureDim
           || currentBrickDimensions.z>maxGPUTextureDim) {
        int theMaxDim = (currentBrickDimensions.x > currentBrickDimensions.y ? (currentBrickDimensions.x > currentBrickDimensions.z ? 0 : 2) :
                         (currentBrickDimensions.y > currentBrickDimensions.z ? 1 : 2));

        if (currentBrickDimensions[theMaxDim] % 2 != 0)
            currentBrickDimensions[theMaxDim]++; //Make the dim we are dividing even

        currentBrickDimensions[theMaxDim] /= 2;
    }

    return currentBrickDimensions;
}

bool OpenGLCapabilities::isExtensionSupported(const char* name) {
    return (glewIsExtensionSupported(name) != '0');
}

bool OpenGLCapabilities::isSupported(const char* name) {
    return (glewIsSupported(name) != '0');
}

bool OpenGLCapabilities::isTexturesSupported() {
    return texSupported_;
}

bool OpenGLCapabilities::isTextureArraysSupported() {
    return texArraySupported_;
}

bool OpenGLCapabilities::is3DTexturesSupported() {
    return tex3DSupported_;
}

bool OpenGLCapabilities::isFboSupported() {
    return fboSupported_;
}

bool OpenGLCapabilities::isShadersSupported() {
    return shadersAreSupported_;
}

bool OpenGLCapabilities::isShadersSupportedARB() {
    return shadersAreSupportedARB_;
}

OpenGLCapabilities::GLSLShaderVersion OpenGLCapabilities::getCurrentShaderVersion() {
    return supportedShaderVersions_[currentGlobalGLSLVersionIdx_];
}

std::string OpenGLCapabilities::getCurrentGlobalGLSLHeader() {
    if (currentGlobalGLSLHeader_ == "")
        rebuildGLSLHeader();

    return currentGlobalGLSLHeader_;
}

std::string OpenGLCapabilities::getCurrentGlobalGLSLFragmentDefines() {
    if (currentGlobalGLSLFragmentDefines_ == "")
        rebuildGLSLFragmentDefines();

    return currentGlobalGLSLFragmentDefines_;
}

glm::u64 OpenGLCapabilities::getCurrentAvailableTextureMem() throw (Exception) {
    glm::u64 currentAvailableTexMeminBytes = 0;

    try {
        GLint nCurAvailMemoryInKB[4] = {0};

        if (glVendor_ == NVIDIA) {
#ifdef GL_NVX_gpu_memory_info
            glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, nCurAvailMemoryInKB);
#endif
        }
        else if (glVendor_ == AMD) {
#ifdef GL_ATI_meminfo
            glGetIntegerv(GL_TEXTURE_FREE_MEMORY_ATI, nCurAvailMemoryInKB);
#endif
        }

        currentAvailableTexMeminBytes = KILOBYTES_TO_BYTES(static_cast<glm::u64>(nCurAvailMemoryInKB[0]));
    }
    catch (const Exception& e) {
        LogWarn("Failed to fetch current available texture memory: " << e.what());
    }

    return currentAvailableTexMeminBytes;
}

glm::u64 OpenGLCapabilities::getTotalAvailableTextureMem() throw (Exception) {
    glm::u64 totalAvailableTexMemInBytes = 0;

    try {
        if (glVendor_ == NVIDIA) {
#ifdef GL_NVX_gpu_memory_info
            GLint nTotalAvailMemoryInKB[4] = {0};
            glGetIntegerv(GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX, nTotalAvailMemoryInKB);
            totalAvailableTexMemInBytes = KILOBYTES_TO_BYTES(static_cast<glm::u64>(nTotalAvailMemoryInKB[0]));
#endif
        }
        else if (glVendor_ == AMD) {
#if defined(WGL_AMD_gpu_association)
            UINT n = wglGetGPUIDsAMD(0, 0);
            UINT *ids = new UINT[n];
            size_t total_mem_mb = 0;
            wglGetGPUIDsAMD(n, ids);
            wglGetGPUInfoAMD(ids[0], 
                WGL_GPU_RAM_AMD, 
                GL_UNSIGNED_INT, 
                sizeof(size_t),
                &total_mem_mb);
            totalAvailableTexMemInBytes = MEGABYTES_TO_BYTES(static_cast<glm::u64>(total_mem_mb));
#elif defined(GLX_AMD_gpu_association)
            UINT n = glXGetGPUIDsAMD(0, 0);
            UINT *ids = new UINT[n];
            size_t total_mem_mb = 0;
            glXGetGPUIDsAMD(n, ids);
            glXGetGPUInfoAMD(ids[0], 
                GLX_GPU_RAM_AMD, 
                GL_UNSIGNED_INT, 
                sizeof(size_t),
                &total_mem_mb);
            totalAvailableTexMemInBytes = MEGABYTES_TO_BYTES(static_cast<glm::u64>(total_mem_mb));
#endif
        }
    }
    catch (const Exception& e) {
        LogWarn("Failed to fetch total available texture memory: " << e.what());
    }

    return totalAvailableTexMemInBytes;
}

int OpenGLCapabilities::getMaxProgramLoopCount() {
    return maxProgramLoopCount_;
}

int OpenGLCapabilities::getNumTexUnits() {
    return numTexUnits_;
}

int OpenGLCapabilities::getMaxTexSize() {
    return maxTexSize_;
}

int OpenGLCapabilities::getMax3DTexSize() {
    return max3DTexSize_;
}

int OpenGLCapabilities::getMaxArrayTexSize() {
    return maxArrayTexSize_;
}

int OpenGLCapabilities::getMaxColorAttachments() {
    return maxColorAttachments_;
}

void OpenGLCapabilities::retrieveStaticInfo() {
    //GL
    const GLubyte* vendor = glGetString(GL_VENDOR);
    glVendorStr_ = std::string((vendor!=NULL ? reinterpret_cast<const char*>(vendor) : "INVALID"));

    if (glVendorStr_.find("NVIDIA") != std::string::npos)
        glVendor_ = NVIDIA;
    else if (glVendorStr_.find("AMD") != std::string::npos || glVendorStr_.find("ATI") != std::string::npos)
        glVendor_ = AMD;
    else if (glVendorStr_.find("INTEL") != std::string::npos || glVendorStr_.find("Intel") != std::string::npos)
        glVendor_ = INTEL;
    else
        glVendor_ = UNKNOWN;

    const GLubyte* glrender = glGetString(GL_RENDERER);
    glRenderStr_ = std::string((glrender!=NULL ? reinterpret_cast<const char*>(glrender) : "INVALID"));
    const GLubyte* glversion = glGetString(GL_VERSION);
    glVersionStr_ = std::string((glversion!=NULL ? reinterpret_cast<const char*>(glversion) : "INVALID"));
    int glVersion = parseAndRetrieveVersion(glVersionStr_);
    //GLSL
    shadersAreSupported_ = (glVersion >= 200);
    shadersAreSupportedARB_ = isExtensionSupported("GL_ARB_fragment_program");
    GLint numberOfSupportedVersions = 0;
    const GLubyte* glslStrByte = NULL;
#ifdef GL_VERSION_4_3
    if(glVersion >= 430){
        glslStrByte = glGetString(GL_SHADING_LANGUAGE_VERSION);
        glslVersionStr_ = std::string((glslStrByte!=NULL ? reinterpret_cast<const char*>(glslStrByte) : "000"));
        glGetIntegerv(GL_NUM_SHADING_LANGUAGE_VERSIONS, &numberOfSupportedVersions);

        for (int i=0; i<numberOfSupportedVersions; i++) {
            parseAndAddShaderVersion(toString<const GLubyte*>(glGetStringi(GL_SHADING_LANGUAGE_VERSION, i)));
        }
    }
#endif

    if (numberOfSupportedVersions == 0) {
        if (isShadersSupported())
            glslStrByte = glGetString(GL_SHADING_LANGUAGE_VERSION);
        else if (isShadersSupportedARB())
            glslStrByte = glGetString(GL_SHADING_LANGUAGE_VERSION_ARB);

        glslVersionStr_ = std::string((glslStrByte!=NULL ? reinterpret_cast<const char*>(glslStrByte) : "000"));
        int glslVersion = parseAndRetrieveVersion(glslVersionStr_);

        if (glslVersion != 0) {
#ifdef GL_VERSION_4_4
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(440, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(440, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_4_3
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(430, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(430, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_4_2
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(420, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(420, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_4_1
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(410, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(410, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_4_0
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(400, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(400, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_3_3
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(330, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(330, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_3_2
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(150, "core"), glslVersion);
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(150, "compatibility"), glslVersion);
#endif
#ifdef GL_VERSION_3_1
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(140), glslVersion);
#endif
#ifdef GL_VERSION_3_0
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(130), glslVersion);
#endif
#ifdef GL_VERSION_2_1
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(120), glslVersion);
#endif
#ifdef GL_VERSION_2_0
            addShaderVersionIfEqualOrLower(GLSLShaderVersion(110), glslVersion);
#endif
        }
    }

    //Set current used GLSL version to highest(i.e. 1st in vector) with preferred profile (or no profile)
    if (isShadersSupported() || isShadersSupportedARB()) {
        size_t i = 0;

        while (i<supportedShaderVersions_.size() && (supportedShaderVersions_[i].hasProfile()
                && supportedShaderVersions_[i].getProfile() != preferredGLSLProfile_))
            i++;

        currentGlobalGLSLVersionIdx_ = i;
    }

    maxProgramLoopCount_ = -1;

    if (GLEW_NV_fragment_program2) {
        GLint i = -1;
        glGetProgramivARB(GL_FRAGMENT_PROGRAM_ARB, GL_MAX_PROGRAM_LOOP_COUNT_NV, &i);

        if (i > 0) {
            //Restrict cycles to realistic samplingRate*maximumDimension, 20*(10 000) slices = 200 000
            maxProgramLoopCount_ = std::min<int>(static_cast<int>(i), 200000);
        }
    }

    //Texturing
#ifdef GL_VERSION_1_1
    texSupported_ = true;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, (GLint*)&maxTexSize_);
#else
    texSupported_ = isExtensionSupported("GL_EXT_texture");
    maxTexSize_ = 0;
#endif
#ifdef GL_VERSION_1_2
    tex3DSupported_ = true;
    glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, (GLint*)&max3DTexSize_);
#else
    tex3DSupported_ = isExtensionSupported("GL_EXT_texture3D");

    if (is3DTexturesSupported())
        glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE_EXT, (GLint*)&max3DTexSize_);
    else
        max3DTexSize_ = 0;

#endif
#ifdef GL_VERSION_3_0
    if(glVersion >= 300){
        texArraySupported_ = true;
        glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS, (GLint*)&maxArrayTexSize_);
    }
    else{
#endif
        texArraySupported_ = isExtensionSupported("GL_EXT_texture_array");

        if (isTextureArraysSupported())
            glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS_EXT, (GLint*)&maxArrayTexSize_);
        else
            maxArrayTexSize_ = 0;
#ifdef GL_VERSION_3_0
    }
#endif
    numTexUnits_ = -1;

    if (isShadersSupported())
        glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS_ARB, (GLint*)&numTexUnits_);

    if (getNumTexUnits() < 0)
        glGetIntegerv(GL_MAX_TEXTURE_UNITS, (GLint*)&numTexUnits_);

    TextureUnit::initialize(numTexUnits_);
    //FBO
    fboSupported_ = isExtensionSupported("GL_EXT_framebuffer_object");
    maxColorAttachments_ = 0;

    if (isFboSupported())
        glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS_EXT, &maxColorAttachments_);
}

void OpenGLCapabilities::retrieveDynamicInfo() {
}

void OpenGLCapabilities::rebuildGLSLHeader() {
    currentGlobalGLSLHeader_ = "#version " + supportedShaderVersions_[currentGlobalGLSLVersionIdx_].getVersionAndProfileAsString() + "\n";

    if (supportedShaderVersions_[currentGlobalGLSLVersionIdx_].getVersion() == 140 && preferredGLSLProfile_ == "compatibility")
        currentGlobalGLSLHeader_ += "#extension GL_ARB_compatibility : enable\n";

    if (supportedShaderVersions_[currentGlobalGLSLVersionIdx_].hasProfile()) {
        currentGlobalGLSLHeader_ += "#define GLSL_PROFILE_" + toUpper(supportedShaderVersions_[currentGlobalGLSLVersionIdx_].getProfile()) + "\n";
    }

    int lastVersion = -1;

    for (size_t i=currentGlobalGLSLVersionIdx_; i<supportedShaderVersions_.size(); i++) {
        if (lastVersion != supportedShaderVersions_[i].getVersion()) {
            currentGlobalGLSLHeader_ += "#define GLSL_VERSION_" + supportedShaderVersions_[i].getVersionAsString() + "\n";
            lastVersion = supportedShaderVersions_[i].getVersion();
        }
    }

    if (getMaxProgramLoopCount() > 0) {
        currentGlobalGLSLHeader_ += "#define MAX_PROGRAM_LOOP_COUNT " + toString(getMaxProgramLoopCount()) + "\n";
    }
}

void OpenGLCapabilities::rebuildGLSLFragmentDefines() {
    currentGlobalGLSLFragmentDefines_ = "";

    if (supportedShaderVersions_[currentGlobalGLSLVersionIdx_].getVersion() >= 130) {
        currentGlobalGLSLFragmentDefines_ += "layout(location = 0) out vec4 FragData0;\n";
        currentGlobalGLSLFragmentDefines_ += "layout(location = " + toString(getMaxColorAttachments()-1) + ") out vec4 PickingData;\n";
    } else {
        currentGlobalGLSLFragmentDefines_ += "#define FragData0 gl_FragColor\n";
        currentGlobalGLSLFragmentDefines_ += "#define PickingData gl_FragData[" + toString(getMaxColorAttachments()-1) + "]\n";
    }
}

void OpenGLCapabilities::addShaderVersion(GLSLShaderVersion version) {
    supportedShaderVersions_.push_back(version);
}

void OpenGLCapabilities::addShaderVersionIfEqualOrLower(GLSLShaderVersion version, int compVersion) {
    if (version.getVersion() <= compVersion)
        addShaderVersion(version);
}

void OpenGLCapabilities::parseAndAddShaderVersion(std::string versionStr) {
    //Assumes <version><space><profile> or <version>, example 420 core or 140
    if (!versionStr.empty()) {
        std::vector<std::string> versionSplit = splitString(versionStr);

        if (versionSplit.size() > 1)
            addShaderVersion(GLSLShaderVersion(stringTo<int>(versionSplit[0]), versionSplit[1]));
        else
            addShaderVersion(GLSLShaderVersion(stringTo<int>(versionSplit[0])));
    }
}

int OpenGLCapabilities::parseAndRetrieveVersion(std::string versionStr) {
    //Assumes <version><space><desc>
    if (!versionStr.empty()) {
        std::vector<std::string> versionSplit = splitString(versionStr);
        return stringTo<int>(removeFromString(versionSplit[0], '.'));
    }

    return 0;
}

} // namespace
