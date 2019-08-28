/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2019 Inviwo Foundation
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

#ifndef IVW_INDEXMAPPER2_H
#define IVW_INDEXMAPPER2_H

#include <array>
#include <iostream>
#include <type_traits>
#include <algorithm>
#include <numeric>

namespace inviwo {
namespace util {
template <size_t N, typename T>
struct Vector_2 {
    template <typename... Args>
    Vector_2(Args&&... args) {
        static_assert(sizeof...(args) == N, "Incorrect number of initializer arguments.");
        //static_assert(std::is_same<T, std::common_type_t<Args...>>::value,
        //              "Types of T and args are not the same.");

        data_ = std::array<T, N>{};
        size_t i{0};
        ((data_[i++] = static_cast<T>(args)), ...);
    }

    const auto& data() const { return data_; }
    const auto size() const { return data_.size(); }
    T operator[](const size_t index) const { return data_[index]; }

private:
    std::array<T, N> data_;
};

template <size_t N, typename IndexType>
struct IndexMapper2 {
    constexpr IndexMapper2(const Vector_2<N, IndexType>& dim) : dimensions_(dim) {
        // Generate coefficients
        for (size_t i{0}; i < N; ++i) {
            IndexType coeff{1};
            for (size_t j{0}; j < N - i - 1; ++j) {
                coeff *= dimensions_[j];
            }
            coeffArray_[i] = coeff;
        }

        // Reverse so we can simply compute the inner product of the coefficient vector and the
        // position vector.
        std::reverse(std::begin(coeffArray_), std::end(coeffArray_));
    };

    template <typename... IndexTypePack>
    constexpr auto operator()(IndexTypePack... coords) const noexcept {
        Vector_2<sizeof...(coords), IndexType> pos(coords...);
        return IndexMapper2::operator()(pos);
    }

    constexpr IndexType operator()(const Vector_2<N, IndexType>& pos) const noexcept {
        const auto& posArray = pos.data();
        return std::inner_product(std::cbegin(coeffArray_), std::cend(coeffArray_),
                                  std::cbegin(posArray), 0);
    }

private:
    Vector_2<N, IndexType> dimensions_;
    std::array<IndexType, N> coeffArray_;
};

using IndexMapper2_2D = IndexMapper2<2, size_t>;
using IndexMapper2_3D = IndexMapper2<3, size_t>;

template <size_t N, typename IndexType>
auto makeIndexMapper2(const Vector_2<N, IndexType>& dim) {
    return IndexMapper2<N, IndexType>(dim);
}

}  // namespace util
}  // namespace inviwo

#endif  // IVW_INDEXMAPPER2_H
