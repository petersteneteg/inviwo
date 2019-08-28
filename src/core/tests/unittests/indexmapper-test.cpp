/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2014-2019 Inviwo Foundation
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

#include <warn/push>
#include <warn/ignore/all>
#include <gtest/gtest.h>
#include <warn/pop>

#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/indexmapper2.h>

namespace inviwo {

TEST(indexmapper, equivalence2D) {
    util::Vector_2<2, size_t> dimensions1(16, 16);
    Vector<2, size_t> dimensions2(16, 16);

    auto indexMapper1 = util::makeIndexMapper2(dimensions1);
    auto index1 = indexMapper1(9, 8);

    auto indexMapper2 = util::IndexMapper<2, size_t>(dimensions2);
    auto index2 = indexMapper2(9, 8);

    EXPECT_EQ(index1, index2);
}

TEST(indexmapper, equivalence3D) {
    util::Vector_2<3, size_t> dimensions1(16, 16, 16);
    Vector<3, size_t> dimensions2(16, 16, 16);

    auto indexMapper1 = util::makeIndexMapper2(dimensions1);
    auto index1 = indexMapper1(9, 8, 7);

    auto indexMapper2 = util::IndexMapper<3, size_t>(dimensions2);
    auto index2 = indexMapper2(9, 8, 7);

    EXPECT_EQ(index1, index2);
}

}  // namespace inviwo
