/* -*- c++ -*- */
/*
 * Copyright 2021 Analog Devices, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ad9081_sink_impl.h"
#include "device_source_impl.h"

#include <gnuradio/blocks/complex_to_float.h>
#include <gnuradio/blocks/float_to_short.h>
#include <gnuradio/io_signature.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace gr {
namespace iio {

ad9081_sink::ad9081_sink(const std::array<bool, MAX_CHANNEL_COUNT>& en,
                         sync_sptr src_block)
    : hier_block2("ad9081_sink",
                  gr::io_signature::make(
                      get_channel_count(en), get_channel_count(en), sizeof(gr_complex)),
                  gr::io_signature::make(0, 0, 0)),
      d_ad9081_block(src_block)
{
    basic_block_sptr hier = self();
    int n = get_channel_count(en);

    for (int i = 0; i < n; i++) {
        auto f2s1 = gr::blocks::float_to_short::make(1, 32768.0f);
        auto f2s2 = gr::blocks::float_to_short::make(1, 32768.0f);
        auto c2f = gr::blocks::complex_to_float::make(1);

        connect(hier, i, c2f, 0);
        connect(c2f, 0, f2s1, 0);
        connect(c2f, 1, f2s2, 0);
        connect(f2s1, 0, d_ad9081_block, 2 * i);
        connect(f2s2, 0, d_ad9081_block, 2 * i + 1);
    }
}

ad9081_sink::sptr ad9081_sink::make(const std::string& uri,
                                    std::array<bool, MAX_CHANNEL_COUNT> en,
                                    size_t buffer_size,
                                    bool cyclic)
{
    auto block = gnuradio::get_initial_sptr(new ad9081_sink_impl(
        device_source_impl::get_context(uri), en, buffer_size, cyclic));

    return gnuradio::get_initial_sptr(new ad9081_sink(en, block));
}

void ad9081_sink::set_main_nco_freq(int nco, int64_t freq)
{
    std::dynamic_pointer_cast<ad9081_sink_impl>(d_ad9081_block)
        ->set_main_nco_freq(nco, freq);
}

void ad9081_sink::set_main_nco_phase(int nco, float phase)
{
    std::dynamic_pointer_cast<ad9081_sink_impl>(d_ad9081_block)
        ->set_main_nco_phase(nco, phase);
}

void ad9081_sink::set_channel_nco_freq(int nco, int64_t freq)
{
    std::dynamic_pointer_cast<ad9081_sink_impl>(d_ad9081_block)
        ->set_channel_nco_freq(nco, freq);
}

void ad9081_sink::set_channel_nco_phase(int nco, float phase)
{
    std::dynamic_pointer_cast<ad9081_sink_impl>(d_ad9081_block)
        ->set_channel_nco_phase(nco, phase);
}

ad9081_sink_impl::ad9081_sink_impl(struct iio_context* ctx,
                                   std::array<bool, MAX_CHANNEL_COUNT> en,
                                   size_t buffer_size,
                                   bool cyclic)
    : gr::sync_block("ad9081_sink",
                     gr::io_signature::make(1, -1, sizeof(int16_t)),
                     gr::io_signature::make(0, 0, 0)),
      device_sink_impl(ctx,
                       destroy_ctx,
                       "axi-ad9081-tx-hpc",
                       get_channel_vector(en),
                       "axi-ad9081-rx-hpc",
                       std::vector<std::string>(),
                       buffer_size,
                       0,
                       cyclic)
{
    for (int i = 0; i < MAX_CHANNEL_COUNT; i++) {
        auto& st = d_channel_state[i];
        st.enabled = en[i];

        if (!st.enabled)
            continue;

        d_last_active_channel = i;

        std::string ch_i_name = "voltage" + std::to_string(i) + "_i";
        std::string ch_q_name = "voltage" + std::to_string(i) + "_q";

        st.ch_i = iio_device_find_channel(phy, ch_i_name.c_str(), true);
        st.ch_q = iio_device_find_channel(phy, ch_q_name.c_str(), true);

        if (!st.ch_i || !st.ch_q)
            throw std::runtime_error("ad9081: Failed to acquire channel voltage" +
                                     std::to_string(i) + "_[iq]!");

        long long converter_rate;
        auto ret =
            iio_channel_attr_read_longlong(st.ch_i, "dac_frequency", &converter_rate);
        if (ret)
            throw std::runtime_error(
                "ad9081: Failed to read dac_frequency from iio channel!");

        st.converter_rate = static_cast<int64_t>(converter_rate);

        parse_datapath(st);

        d_channel_ncos[st.channel_nco] = static_cast<int8_t>(i);
        d_main_ncos[st.main_nco] = static_cast<int8_t>(i);
    }

    if (d_last_active_channel == -1)
        throw std::runtime_error("ad9081: No channels active!");
}

} /* namespace iio */
} /* namespace gr */
