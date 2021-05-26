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

#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/short_to_float.h>
#include <gnuradio/io_signature.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace gr {
namespace iio {

ad9081_source::ad9081_source(const std::array<bool, MAX_CHANNEL_COUNT>& en,
                             sync_sptr src_block)
    : hier_block2("ad9081_source",
                  gr::io_signature::make(0, 0, 0),
                  gr::io_signature::make(
                      get_channel_count(en), get_channel_count(en), sizeof(gr_complex))),
      d_ad9081_block(src_block)
{
    auto hier = self();
    int n = get_channel_count(en);

    for (int i = 0; i < n; i++) {
        auto s2f1 = gr::blocks::short_to_float::make(1, 8192.0f);
        auto s2f2 = gr::blocks::short_to_float::make(1, 8192.0f);
        auto f2c = gr::blocks::float_to_complex::make(1);

        connect(d_ad9081_block, i * 2, s2f1, 0);
        connect(d_ad9081_block, i * 2 + 1, s2f2, 0);
        connect(s2f1, 0, f2c, 0);
        connect(s2f2, 0, f2c, 1);
        connect(f2c, 0, hier, i);
    }
}

void ad9081_source::set_main_nco_freq(int nco, int64_t freq)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)
        ->set_main_nco_freq(nco, freq);
}

void ad9081_source::set_main_nco_phase(int nco, float phase)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)
        ->set_main_nco_phase(nco, phase);
}

void ad9081_source::set_channel_nco_freq(int nco, int64_t freq)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)
        ->set_channel_nco_freq(nco, freq);
}

void ad9081_source::set_channel_nco_phase(int nco, float phase)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)
        ->set_channel_nco_phase(nco, phase);
}

void ad9081_source::set_nyquist_zone(bool odd)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)->set_nyquist_zone(odd);
}

void ad9081_source::set_filter_source_file(const std::string& src_file)
{
    std::dynamic_pointer_cast<ad9081_source_impl>(d_ad9081_block)
        ->set_filter_source_file(src_file);
}

ad9081_source::sptr ad9081_source::make(const std::string& uri,
                                        std::array<bool, MAX_CHANNEL_COUNT> en,
                                        size_t buffer_size)
{
    auto block = gnuradio::get_initial_sptr(
        new ad9081_source_impl(device_source_impl::get_context(uri), en, buffer_size));

    return gnuradio::get_initial_sptr(new ad9081_source(en, block));
}

ad9081_source_impl::ad9081_source_impl(struct iio_context* ctx,
                                       std::array<bool, MAX_CHANNEL_COUNT> en,
                                       size_t buffer_size)
    : gr::sync_block("ad9081_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, -1, sizeof(int16_t))),
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
