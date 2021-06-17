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
/* BINDTOOL_HEADER_FILE(fmcomms2_sink.h)                                           */
/* BINDTOOL_HEADER_FILE_HASH(1ca54fa74c12834cae56b0832dd5c3f3)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/iio/fmcomms2_sink.h>
// pydoc.h is automatically generated in the build directory
#include <fmcomms2_sink_pydoc.h>

void bind_fmcomms2_sink(py::module& m)
{

    using fmcomms2_sink = gr::iio::fmcomms2_sink;


    py::class_<fmcomms2_sink,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<fmcomms2_sink>>(m, "fmcomms2_sink", D(fmcomms2_sink))

        .def(py::init(&fmcomms2_sink::make),
             py::arg("uri"),
             py::arg("longfrequency"),
             py::arg("samplerate"),
             py::arg("bandwidth"),
             py::arg("ch1_en"),
             py::arg("ch2_en"),
             py::arg("ch3_en"),
             py::arg("ch4_en"),
             py::arg("buffer_size"),
             py::arg("cyclic"),
             py::arg("rf_port_select"),
             py::arg("attenuation1"),
             py::arg("attenuation2"),
             py::arg("filter_source") = "",
             py::arg("filter_filename") = "",
             py::arg("Fpass") = 0.0,
             py::arg("Fstop") = 0.0,
             D(fmcomms2_sink, make))

        .def("set_params",
             &fmcomms2_sink::set_params,
             py::arg("longfrequency"),
             py::arg("samplerate"),
             py::arg("bandwidth"),
             py::arg("rf_port_select"),
             py::arg("attenuation1"),
             py::arg("attenuation2"),
             py::arg("filter_source") = "",
             py::arg("filter_filename") = "",
             py::arg("Fpass") = 0.0,
             py::arg("Fstop") = 0.0,
             D(fmcomms2_sink, set_params))

        ;
}