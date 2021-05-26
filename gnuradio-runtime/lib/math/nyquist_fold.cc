/* -*- c++ -*- */
/*
 * Copyright 2021 Analog Devices, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */


#include <gnuradio/math.h>

#include <cstdint>

namespace gr {

int64_t nyquist_fold(int64_t f, int64_t f_s)
{
    int64_t f_n = f_s / 2;

    bool neg = f < 0;
    if (neg)
        f = -f;

    int64_t n = (f / f_n + 1) / 2;

    int64_t r = f - n * f_s;
    if (neg)
        r = -r;

    return r;
}

} // namespace gr
