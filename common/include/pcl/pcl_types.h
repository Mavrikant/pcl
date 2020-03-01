/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, OpenPerception
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/**
 * \file pcl/pcl_types.h
 *
 * \brief Defines basic non-point types used by PCL
 * \ingroup common
 */

#include <pcl/pcl_macros.h>

// @TODO: replace by memory when std::shared_ptr is used for pcl::shared_ptr
#include <boost/smart_ptr/shared_ptr.hpp>

#include <cstdint>

namespace pcl
{
  /**
   * \brief Alias for boost::shared_ptr
   *
   * For ease of switching from boost::shared_ptr to std::shared_ptr
   *
   * \see pcl::make_shared
   * \tparam T Type of the object stored inside the shared_ptr
   */
  template <typename T>
  using shared_ptr = boost::shared_ptr<T>;

  using uint8_t PCL_DEPRECATED("use std::uint8_t instead of pcl::uint8_t") = std::uint8_t;
  using int8_t PCL_DEPRECATED("use std::int8_t instead of pcl::int8_t") = std::int8_t;
  using uint16_t PCL_DEPRECATED("use std::uint16_t instead of pcl::uint16_t") = std::uint16_t;
  using int16_t PCL_DEPRECATED("use std::uint16_t instead of pcl::int16_t") = std::int16_t;
  using uint32_t PCL_DEPRECATED("use std::uint32_t instead of pcl::uint32_t") = std::uint32_t;
  using int32_t PCL_DEPRECATED("use std::int32_t instead of pcl::int32_t") = std::int32_t;
  using uint64_t PCL_DEPRECATED("use std::uint64_t instead of pcl::uint64_t") = std::uint64_t;
  using int64_t PCL_DEPRECATED("use std::int64_t instead of pcl::int64_t") = std::int64_t;
  using int_fast16_t PCL_DEPRECATED("use std::int_fast16_t instead of pcl::int_fast16_t") = std::int_fast16_t;

  namespace detail {
    template <std::size_t Bits, bool Signed = true>
    struct int_type { using type = void; };
    template <std::size_t Bits, bool Signed = true>
    using int_type_t = typename int_type<Bits, Signed>::type;

    template <>
    struct int_type<8, true> { using type = std::int8_t; };
    template <>
    struct int_type<8, false> { using type = std::uint8_t; };
    template <>
    struct int_type<16, true> { using type = std::int16_t; };
    template <>
    struct int_type<16, false> { using type = std::uint16_t; };
    template <>
    struct int_type<32, true> { using type = std::int32_t; };
    template <>
    struct int_type<32, false> { using type = std::uint32_t; };
    template <>
    struct int_type<64, true> { using type = std::int64_t; };
    template <>
    struct int_type<64, false> { using type = std::uint64_t; };
  }  // namespace detail
#ifndef PCL_INDEX_SIZE
#define PCL_INDEX_SIZE 32
#endif
#ifndef PCL_INDEX_SIGNED
#define PCL_INDEX_SIGNED true
#endif
  /**
   * \brief Type used for indices in PCL
   *
   * Please use PCL_INDEX_SIZE and PCL_INDEX_SIGNED to choose a type best for you
   * Defaults:
   *   * PCL_INDEX_SIZE = 32
   *   * PCL_INDEX_SIGNED = true
   *   * index_t = std::int32_t;
   */
  using index_t = detail::int_type_t<PCL_INDEX_SIZE, PCL_INDEX_SIGNED>;
}  // namespace pcl
