

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(FindPackageHandleStandardArgs)

find_path(LIBBFD_INCLUDE_DIR bfd.h
  /opt/local/include
  /usr/local/include
  /usr/include
)

set(LIBBFD_NAMES bfd)

find_library(LIBBFD_LIBRARY
  NAMES ${LIBBFD_NAMES}
  PATHS /usr/lib /usr/local/lib /opt/local/lib
)

if (LIBBFD_INCLUDE_DIR AND LIBBFD_LIBRARY)
   set(LIBBFD_FOUND TRUE)
   set( LIBBFD_LIBRARIES ${LIBBFD_LIBRARY} )
else (LIBBFD_INCLUDE_DIR AND LIBBFD_LIBRARY)
   set(LIBBFD_FOUND FALSE)
   set(LIBBFD_LIBRARIES)
endif (LIBBFD_INCLUDE_DIR AND LIBBFD_LIBRARY)

if (LIBBFD_FOUND)
      message(STATUS "Found BFD: ${LIBBFD_LIBRARY}")
      find_file(BACKWARD_SRC backward.cpp PATHS ${CMAKE_CURRENT_LIST_DIR})
      find_path(BACKWARD_INCLUDE_DIR backward.hpp PATHS ${CMAKE_CURRENT_LIST_DIR})
      set(BACKWARD_LIBS ${CMAKE_DL_LIBS} bfd PARENT_SCOPE )

else (LIBBFD_FOUND)
      message(STATUS "Could NOT find Bfd library named ${LIBBFD_NAMES}.")
endif (LIBBFD_FOUND)

mark_as_advanced(
  LIBBFD_LIBRARY
  LIBBFD_INCLUDE_DIR
)



