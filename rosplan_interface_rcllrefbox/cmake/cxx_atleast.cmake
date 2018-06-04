# - Use highest available CXX standard
#
# Copyright (c) 2017  Tim Niemueller [www.niemueller.de]
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

macro(cxx_atleast min_version)
  if (CMAKE_VERSION VERSION_LESS "3.1")
		foreach (V IN ITEMS 17 14 11 0x)
			#message(STATUS "Checking C++ standard C++${V}")
			if (("${min_version}" STREQUAL "${V}") OR ("${min_version}" STRLESS "${V}"))
				CHECK_CXX_COMPILER_FLAG("-std=c++${V}" COMPILER_SUPPORTS_MINVER)

				if (COMPILER_SUPPORTS_MINVER)
					if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
						message(STATUS "Found matching C++ standard C++${V} (GNU)")
						set (CMAKE_CXX_FLAGS "--std=gnu++${V} ${CMAKE_CXX_FLAGS}")
						break()
					else()
						message(STATUS "Found matching C++ standard C++${V}")
						set (CMAKE_CXX_FLAGS "--std=c++${V} ${CMAKE_CXX_FLAGS}")
					endif()
				endif ()
			endif()
		endforeach(V)
	else ()
		message(STATUS "Set minimum standard to C++${V} (cmake 3.1+)")
    set (CMAKE_CXX_STANDARD ${min_version})
    set (CMAKE_CXX_STANDARD_REQUIRED ON)
  endif ()
endmacro(cxx_atleast)
