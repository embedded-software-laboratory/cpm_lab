// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

/**
 * \file util.h
 *
 * \author maczijewski
 * \date Created: 20.09.2018 09:39:30
 * 
 * \brief This file provides some utility defines.
 * 
 * \ingroup low_level_controller
 */

#ifndef UTIL_H_
#define UTIL_H_

/**
 * \brief Given a byte p, this define sets the n-th bit.
 * \ingroup low_level_controller
 */
#define SET_BIT(p,n) ((p) |= (1 << (n)))

/**
 * \brief Given a byte p, this define resets the n-th bit.
 * \ingroup low_level_controller
 */
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))

/**
 * \brief Given a byte p, this define toggles the n-th bit.
 * \ingroup low_level_controller
 */
#define TOGGLE_BIT(p,n) ((p) ^= (1 << (n)))


#endif /* UTIL_H_ */