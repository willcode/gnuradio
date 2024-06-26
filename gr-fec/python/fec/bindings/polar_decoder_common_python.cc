/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(polar_decoder_common.h)                                    */
/* BINDTOOL_HEADER_FILE_HASH(218d7a62038392c8e14b6411861fdd41)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/fec/polar_decoder_common.h>
// pydoc.h is automatically generated in the build directory
#include <polar_decoder_common_pydoc.h>

void bind_polar_decoder_common(py::module& m)
{


    py::module m_code = m.def_submodule("code");

    using polar_decoder_common = ::gr::fec::code::polar_decoder_common;


    py::class_<polar_decoder_common,
               gr::fec::generic_decoder,
               std::shared_ptr<polar_decoder_common>>(
        m_code, "polar_decoder_common", D(code, polar_decoder_common))

        // .def(py::init<int,int,std::vector<int, std::allocator<int> >,std::vector<char,
        // std::allocator<char> >>(),           py::arg("block_size"),
        //    py::arg("num_info_bits"),
        //    py::arg("frozen_bit_positions"),
        //    py::arg("frozen_bit_values"),
        //    D(code,polar_decoder_common,polar_decoder_common,0)
        // )
        // .def(py::init<gr::fec::code::polar_decoder_common const &>(), py::arg("arg0"),
        //    D(code,polar_decoder_common,polar_decoder_common,1)
        // )


        .def("rate", &polar_decoder_common::rate, D(code, polar_decoder_common, rate))


        .def("get_input_size",
             &polar_decoder_common::get_input_size,
             D(code, polar_decoder_common, get_input_size))


        .def("get_output_size",
             &polar_decoder_common::get_output_size,
             D(code, polar_decoder_common, get_output_size))


        .def("set_frame_size",
             &polar_decoder_common::set_frame_size,
             py::arg("frame_size"),
             D(code, polar_decoder_common, set_frame_size))

        ;
}
