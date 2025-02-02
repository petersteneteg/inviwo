/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2022 Inviwo Foundation
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

#include <modules/base/datavisualizer/meshinformationvisualizer.h>

#include <inviwo/core/common/inviwoapplication.h>
#include <modules/base/processors/meshsource.h>
#include <modules/base/processors/meshinformation.h>
#include <inviwo/core/processors/processorutils.h>
#include <inviwo/core/ports/meshport.h>

#include <inviwo/core/io/datareaderfactory.h>

namespace inviwo {

using GP = util::GridPos;

MeshInformationVisualizer::MeshInformationVisualizer(InviwoApplication* app)
    : DataVisualizer{}, app_(app) {}

std::string MeshInformationVisualizer::getName() const { return "Mesh Information"; }

Document MeshInformationVisualizer::getDescription() const {
    Document doc;
    auto b = doc.append("html").append("body");
    b.append("", "Construct a mesh information processor");
    return doc;
}

std::vector<FileExtension> MeshInformationVisualizer::getSupportedFileExtensions() const {
    auto rf = app_->getDataReaderFactory();
    auto exts = rf->getExtensionsForType<Mesh>();
    return exts;
}

bool MeshInformationVisualizer::isOutportSupported(const Outport* port) const {
    return dynamic_cast<const MeshOutport*>(port) != nullptr;
}

bool MeshInformationVisualizer::hasSourceProcessor() const { return true; }
bool MeshInformationVisualizer::hasVisualizerNetwork() const { return true; }

std::pair<Processor*, Outport*> MeshInformationVisualizer::addSourceProcessor(
    const std::string& filename, ProcessorNetwork* net) const {

    auto source = net->addProcessor(util::makeProcessor<MeshSource>(GP{0, 0}, app_, filename));
    auto outport = source->getOutports().front();
    return {source, outport};
}

std::vector<Processor*> MeshInformationVisualizer::addVisualizerNetwork(
    Outport* outport, ProcessorNetwork* net) const {

    auto info = net->addProcessor(util::makeProcessor<MeshInformation>(GP{0, 3}));

    net->addConnection(outport, info->getInports()[0]);

    return {info};
}

std::vector<Processor*> MeshInformationVisualizer::addSourceAndVisualizerNetwork(
    const std::string& filename, ProcessorNetwork* net) const {

    auto sourceAndOutport = addSourceProcessor(filename, net);
    auto processors = addVisualizerNetwork(sourceAndOutport.second, net);

    processors.push_back(sourceAndOutport.first);
    return processors;
}

}  // namespace inviwo
