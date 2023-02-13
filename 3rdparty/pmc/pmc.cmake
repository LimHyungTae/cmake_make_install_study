# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
include(ExternalProject)

ExternalProject_Add(
  external_pmc
  PREFIX pmc
  URL https://github.com/LimHyungTae/pmc/releases/tag/v1.0.0/libpmc.tar.gz
  # URL_HASH SHA256=b0cbe137e31bb62577f672b89f0bca3ea870285108e3d42203903cb65720abe0
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Get_Property(external_pmc SOURCE_DIR)
add_library(PMCHelper INTERFACE)
add_dependencies(PMCHelper external_pmc)
target_include_directories(PMCHelper SYSTEM INTERFACE $<BUILD_INTERFACE:${SOURCE_DIR}>)
set_property(TARGET PMCHelper PROPERTY EXPORT_NAME pmc)
add_library(pmc ALIAS PMCHelper)
