/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2022 Inviwo Foundation
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
 *********************************************************************************/

#include <modules/base/processors/imageinformation.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ImageInformation::processorInfo_{
    "org.inviwo.ImageInformation",     // Class identifier
    "Image Information",               // Display name
    "Information",                     // Category
    CodeState::Stable,                 // Code state
    "CPU, Image, Layer, Information",  // Tags
    R"(
    Shows available information provided by the input image including metadata.
    )"_unindentHelp};
const ProcessorInfo ImageInformation::getProcessorInfo() const { return processorInfo_; }

ImageInformation::ImageInformation()
    : Processor()
    , image_("image", "Input image"_help, OutportDeterminesSize::Yes)
    , imageInfo_("dataInformation", "Data Information")
    , metaDataProperty_(
          "metaData", "Meta Data",
          "Composite property listing all the metadata stored in the input Image"_help) {

    addPort(image_);
    addProperty(imageInfo_);
    addProperty(metaDataProperty_);

    imageInfo_.setSerializationMode(PropertySerializationMode::None);

    setAllPropertiesCurrentStateAsDefault();
}

void ImageInformation::process() {
    auto image = image_.getData();

    imageInfo_.updateForNewImage(*image);

    metaDataProps_.updateProperty(metaDataProperty_, image->getMetaDataMap());
}

}  // namespace inviwo
