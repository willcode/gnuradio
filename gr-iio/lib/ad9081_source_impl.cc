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

#include "ad9081_source_impl.h"

#include <gnuradio/io_signature.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace gr {
namespace iio {

ad9081_source::sptr ad9081_source::make(const std::string& uri,
                                        const std::array<bool, MAX_CHANNEL_COUNT>& en,
                                        size_t buffer_size)
{
    return gnuradio::make_block_sptr<ad9081_source_impl>(
        // TODO: move get_context() from device_source_impl to a common location
        device_source_impl::get_context(uri),
        en,
        buffer_size);
}

ad9081_source_impl::ad9081_source_impl(struct iio_context* ctx,
                                       const std::array<bool, MAX_CHANNEL_COUNT>& en,
                                       size_t buffer_size)
    : gr::sync_block("ad9081_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, -1, sizeof(gr_complex))),
      device_source_impl(ctx,
                         destroy_ctx,
                         "axi-ad9081-rx-hpc",
                         get_channel_vector(en),
                         "axi-ad9081-rx-hpc",
                         std::vector<std::string>(),
                         buffer_size,
                         0)
{
    for (int i = 0; i < MAX_CHANNEL_COUNT; i++) {
        auto& st = d_channel_state[i];
        st.enabled = en[i];

        if (!st.enabled)
            continue;

        d_last_active_channel = i;

        std::string ch_i_name = "voltage" + std::to_string(i) + "_i";
        std::string ch_q_name = "voltage" + std::to_string(i) + "_q";

        st.ch_i = iio_device_find_channel(dev, ch_i_name.c_str(), false);
        st.ch_q = iio_device_find_channel(dev, ch_q_name.c_str(), false);

        if (!st.ch_i || !st.ch_q)
            throw std::runtime_error("ad9081: Failed to acquire channel voltage" +
                                     std::to_string(i) + "_[iq]!");

        long long int converter_rate;
        auto ret =
            iio_channel_attr_read_longlong(st.ch_i, "adc_frequency", &converter_rate);
        if (ret)
            throw std::runtime_error(
                "ad9081: Failed to read adc_frequency from iio channel!");

        st.converter_rate = static_cast<int64_t>(converter_rate);

        parse_datapath(st);

        d_channel_ncos[st.channel_nco] = (int8_t)i;
        d_main_ncos[st.main_nco] = (int8_t)i;
    }

    if (d_last_active_channel == -1)
        throw std::runtime_error("ad9081: No channels active!");
}

void ad9081_source_impl::set_nyquist_zone(bool odd)
{
    auto& st = d_channel_state[d_last_active_channel];
    int ret = iio_channel_attr_write(st.ch_i, "nyquist_zone", odd ? "odd" : "even");
    if (ret < 0)
        throw std::runtime_error(
            "ad9081: set_nyquist_zone: Failed to write NCO attribute");
}

void ad9081_source_impl::set_filter_source_file(const std::string& filter)
{
    if (filter.empty() || !iio_device_find_attr(phy, "filter_fir_config"))
        return;

    std::ifstream ifs(filter.c_str(), std::ifstream::binary);
    if (!ifs)
        throw std::runtime_error("ad9081: Failed to open filter file!");

    ifs.seekg(0, ifs.end);
    size_t length = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    char* buffer = new char[length];

    ifs.read(buffer, length);
    ifs.close();

    int ret = iio_device_attr_write_raw(phy, "filter_fir_config", buffer, length);

    delete[] buffer;
    if (ret < 0)
        throw std::runtime_error("ad9081: Failed to write filter config to device");
}

} /* namespace iio */
} /* namespace gr */
