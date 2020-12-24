cmake_minimum_required (VERSION 3.10)
project (liblsl
	VERSION 1.14.0
	LANGUAGES C CXX
        DESCRIPTION "Labstreaminglayer C/C++ library"
	)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_POSITION_INDEPENDENT_CODE On)
if(NOT MSVC_VERSION VERSION_LESS 1700)
	set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(LSL_WINVER "0x0601" CACHE STRING
	"Windows version (_WIN32_WINNT) to target (defaults to 0x0601 for Windows 7)")

# Add an object library so all files are only compiled once
add_library(lslobj STATIC
	src/api_config.cpp
	src/api_config.h
	src/api_types.hpp
	src/cancellable_streambuf.h
	src/cancellation.h
	src/cancellation.cpp
	src/cast.h
	src/cast.cpp
	src/common.cpp
	src/common.h
	src/consumer_queue.cpp
	src/consumer_queue.h
	src/data_receiver.cpp
	src/data_receiver.h
	src/forward.h
	src/info_receiver.cpp
	src/info_receiver.h
	src/inireader.h
	src/inireader.cpp
	src/inlet_connection.cpp
	src/inlet_connection.h
	src/lsl_resolver_c.cpp
	src/lsl_inlet_c.cpp
	src/lsl_outlet_c.cpp
	src/lsl_streaminfo_c.cpp
	src/lsl_xml_element_c.cpp
	src/netinterfaces.h
	src/netinterfaces.cpp
	src/portable_archive/portable_archive_exception.hpp
	src/portable_archive/portable_archive_includes.hpp
	src/portable_archive/portable_iarchive.hpp
	src/portable_archive/portable_oarchive.hpp
	src/resolver_impl.cpp
	src/resolver_impl.h
	src/resolve_attempt_udp.cpp
	src/resolve_attempt_udp.h
	src/sample.cpp
	src/sample.h
	src/send_buffer.cpp
	src/send_buffer.h
	src/socket_utils.cpp
	src/socket_utils.h
	src/stream_info_impl.cpp
	src/stream_info_impl.h
	src/stream_inlet_impl.h
	src/stream_outlet_impl.cpp
	src/stream_outlet_impl.h
	src/tcp_server.cpp
	src/tcp_server.h
	src/time_postprocessor.cpp
	src/time_postprocessor.h
	src/time_receiver.cpp
	src/time_receiver.h
	src/udp_server.cpp
	src/udp_server.h
	src/buildinfo.cpp
        thirdparty/loguru/loguru.cpp
	# headers
	include/lsl_c.h
	include/lsl_cpp.h
	include/lsl/common.h
	include/lsl/inlet.h
	include/lsl/outlet.h
	include/lsl/resolver.h
	include/lsl/streaminfo.h
	include/lsl/types.h
	include/lsl/xml.h
)


message(STATUS "Using bundled pugixml")
target_sources(lslobj PRIVATE thirdparty/pugixml/pugixml.cpp)
target_include_directories(lslobj
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml)


# try to find out which revision is currently checked out
find_package(Git)
if(lslgitrevision AND lslgitbranch)
	message(STATUS "Got git information ${lslgitrevision}/${lslgitbranch} from the command line")
elseif(GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} describe --tags HEAD
		WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
		OUTPUT_VARIABLE lslgitrevision
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --symbolic-full-name --abbrev-ref @
		WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
		OUTPUT_VARIABLE lslgitbranch
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	message(STATUS "Git version information: ${lslgitbranch}/${lslgitrevision}")
else()
	set(lslgitrevision "unknown")
	set(lslgitbranch "unknown")
endif()

set(LSL_VERSION_INFO "git:${lslgitrevision}/branch:${lslgitbranch}/build:${CMAKE_BUILD_TYPE}/compiler:${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}")

## create the lslboost target
add_library(lslboost STATIC
	lslboost/asio_objects.cpp
	lslboost/serialization_objects.cpp
)

find_package(Threads REQUIRED)

if(UNIX)
	include(CheckSymbolExists)
	# look for clock_gettime, if not we need to link against librt
	check_symbol_exists(clock_gettime time.h HAS_GETTIME)
	if(NOT HAS_GETTIME)
		set(NEEDS_LIBRT ON CACHE INTERNAL "Link to librt")
        endif()

        target_link_libraries(lslboost PUBLIC Threads::Threads )

        target_compile_features(lslboost PUBLIC cxx_std_11 cxx_lambda_init_captures)

        target_compile_definitions(lslboost PUBLIC
            BOOST_ALL_NO_LIB
            BOOST_ASIO_STANDALONE
            BOOST_ASIO_SEPARATE_COMPILATION
            BOOST_THREAD_DONT_PROVIDE_INTERRUPTIONS )

        target_include_directories(lslboost PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/lslboost)
        target_link_libraries(lslobj
                PRIVATE lslboost
                PUBLIC dl rt )
endif()

target_include_directories(lslobj PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/loguru )

target_compile_definitions(lslobj PRIVATE
     LOGURU_DEBUG_LOGGING=0
     INTERFACE LSLNOAUTOLINK # don't use #pragma(lib) in CMake builds
)

set(LSL_EXPORT_TARGETS lsl lslobj lslboost)

add_library(lsl-static STATIC src/buildinfo.cpp)
target_compile_definitions(lsl-static PRIVATE LSL_LIBRARY_INFO_STR="${LSL_VERSION_INFO}/link:static")
target_link_libraries(lsl-static PUBLIC lslobj PRIVATE lslboost)
list(APPEND LSL_EXPORT_TARGETS lsl-static)
# for LSL_CPP_API export header
target_compile_definitions(lsl-static PUBLIC LIBLSL_STATIC)



