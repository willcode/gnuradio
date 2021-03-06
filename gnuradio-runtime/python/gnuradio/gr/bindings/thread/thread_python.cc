/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/* This file is automatically generated using bindtool */

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/thread.h>

void bind_thread(py::module& m)
{


    m.def("get_current_thread_id", &gr::get_current_thread_id);
    m.def("thread_bind_to_processor",
          (void (*)(std::vector<int, std::allocator<int>> const&)) &
              gr::thread_bind_to_processor,
          py::arg("mask"));
    m.def("thread_bind_to_processor",
          (void (*)(int)) & gr::thread_bind_to_processor,
          py::arg("n"));
    m.def("thread_bind_to_processor",
          (void (*)(gr::thread::gr_thread_t,
                    std::vector<int, std::allocator<int>> const&)) &
              gr::thread_bind_to_processor,
          py::arg("thread"),
          py::arg("mask"));
    m.def("thread_bind_to_processor",
          (void (*)(gr::thread::gr_thread_t, unsigned int)) &
              gr::thread_bind_to_processor,
          py::arg("thread"),
          py::arg("n"));
    m.def("thread_unbind", (void (*)()) & gr::thread_unbind);
    m.def("thread_unbind",
          (void (*)(gr::thread::gr_thread_t)) & gr::thread_unbind,
          py::arg("thread"));
    m.def("thread_priority", &gr::thread_priority, py::arg("thread"));
    m.def("set_thread_priority",
          &gr::set_thread_priority,
          py::arg("thread"),
          py::arg("priority"));
    m.def("set_thread_name", &gr::set_thread_name, py::arg("thread"), py::arg("name"));
}
