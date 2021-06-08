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

#include <gnuradio/io_signature.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace gr {
namespace iio {

ad9081_sink::sptr ad9081_sink::make(const std::string& uri,
                                    const std::array<bool, MAX_CHANNEL_COUNT>& en,
                                    size_t buffer_size,
                                    bool cyclic)
{
    return gnuradio::make_block_sptr<ad9081_sink_impl>(
        // TODO: move get_context() from device_source_impl to a common location
        device_source_impl::get_context(uri),
        en,
        buffer_size,
        cyclic);
}

ad9081_sink_impl::ad9081_sink_impl(struct iio_context* ctx,
                                   const std::array<bool, MAX_CHANNEL_COUNT>& en,
                                   size_t buffer_size,
                                   bool cyclic)
    : gr::sync_block("ad9081_sink",
                     gr::io_signature::make(1, -1, sizeof(gr_complex)),
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
