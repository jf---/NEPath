cmake_minimum_required(VERSION 3.15)
project(nepath_bindings LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ENABLE_PRECOMPILED_HEADERS "Enable precompiled headers for the build" ON)

# External dependencies
include(ExternalProject)

# Setup Eigen (header-only library)
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(EIGEN_INCLUDE_DIR "${EXTERNAL_DIR}/eigen")

# Create external downloads target
add_custom_target(external_downloads ALL)

# Download Eigen if not already present
if(NOT EXISTS "${EIGEN_INCLUDE_DIR}")
  message(STATUS "Downloading Eigen...")
  ExternalProject_Add(
      eigen_download
      URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
      SOURCE_DIR "${EIGEN_INCLUDE_DIR}"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      LOG_DOWNLOAD ON
      UPDATE_COMMAND ""
      PATCH_COMMAND ""
      TLS_VERIFY ON
  )
  add_dependencies(external_downloads eigen_download)
endif()

# Find Python and nanobind
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)

find_package(nanobind CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Download CGAL (header-only, specific version for compatibility)
set(CGAL_DIR "${EXTERNAL_DIR}/cgal")
set(CGAL_INCLUDE_DIR "${CGAL_DIR}/include")

if(NOT EXISTS "${CGAL_DIR}")
  message(STATUS "Downloading CGAL 6.0.1...")
  ExternalProject_Add(
    cgal_download
    URL https://github.com/CGAL/cgal/releases/download/v6.0.1/CGAL-6.0.1.zip
    SOURCE_DIR "${CGAL_DIR}"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    TLS_VERIFY ON
  )
  add_dependencies(external_downloads cgal_download)
  set(HAVE_CGAL TRUE)
else()
  set(HAVE_CGAL TRUE)
endif()

# Download Boost headers (specific version for CGAL compatibility)
# Use SourceForge release which has proper header structure in boost/
set(BOOST_DIR "${EXTERNAL_DIR}/boost")
set(BOOST_INCLUDE_DIR "${BOOST_DIR}")

if(NOT EXISTS "${BOOST_DIR}")
  message(STATUS "Downloading Boost 1.82.0 headers...")
  ExternalProject_Add(
    boost_download
    URL https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz
    SOURCE_DIR "${BOOST_DIR}"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    TLS_VERIFY ON
  )
  add_dependencies(external_downloads boost_download)
endif()

# Add include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master
  ${EIGEN_INCLUDE_DIR}
)

# Find IPOPT
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Collect NEPath source files
set(NEPATH_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/Basic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/Connector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/ContourParallel.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/Curve.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/DirectionParallel.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/FileAgent.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/NEPathPlanner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/PlanningOptions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/clipper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master/NonEquidistant.cpp  # Now included with IPOPT support
)

# Define a function to add a nanobind module with common settings
function(add_nanobind_extension name source)
  nanobind_add_module(
    ${name}
    STABLE_ABI
    NB_STATIC
    ${source}
    ${ARGN}  # Additional source files
  )

  # Apply precompiled headers
  target_precompile_headers(${name} PRIVATE src/nepath.h)

  # Include directories
  target_include_directories(${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../NEPath-master
    ${EIGEN_INCLUDE_DIR}
    ${nanobind_INCLUDE_DIRS}
    ${IPOPT_INCLUDE_DIRS}
  )

  # Compiler definitions
  target_compile_definitions(${name} PRIVATE
    IncludeIpopt=1   # Enable IPOPT support
    IncludeGurobi=0  # Disable Gurobi
    ENABLE_DIRECTION_PARALLEL  # Enable Raster/Zigzag support
  )

  # Compiler flags
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${name} PRIVATE
      -Wno-register  # Allow deprecated 'register' keyword
      -Wno-deprecated-declarations
    )
  elseif(MSVC)
    target_compile_options(${name} PRIVATE
      /wd4996  # Disable deprecation warnings
    )
  endif()

  # Link IPOPT libraries
  target_link_libraries(${name} PRIVATE
    ${IPOPT_LIBRARIES}
    ${CMAKE_DL_LIBS}
  )

  # Add IPOPT link directories
  target_link_directories(${name} PRIVATE
    ${IPOPT_LIBRARY_DIRS}
  )

  # Add dependencies
  add_dependencies(${name} external_downloads)

  # Install the module
  install(TARGETS ${name} LIBRARY DESTINATION nepath_bindings)
endfunction()

# Create individual extension modules for each C++ file
# NEPath wrapper module with all NEPath source files
add_nanobind_extension(_nepath src/nepath_wrapper.cpp ${NEPATH_SOURCES})

# ============================================================================
# CGAL geometry module (optional)
# ============================================================================
if(HAVE_CGAL)
  # CGAL module - separate from main NEPath module
  nanobind_add_module(
    _cgal
    STABLE_ABI
    NB_STATIC
    src/cgal_bindings.cpp
    src/cgal_geometry.cpp
  )

  # CGAL-specific configuration (use downloaded headers)
  target_include_directories(_cgal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN_INCLUDE_DIR}
    ${CGAL_INCLUDE_DIR}
    ${BOOST_INCLUDE_DIR}
  )

  # CGAL compile definitions (header-only mode, use Boost.Multiprecision)
  target_compile_definitions(_cgal PRIVATE
    CGAL_HEADER_ONLY=1
    CGAL_USE_BOOST_MP=1
    CGAL_DISABLE_GMP=1
    CGAL_DO_NOT_USE_MPZF=1
  )

  # Compiler flags
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(_cgal PRIVATE
      -Wno-deprecated-declarations
    )
  endif()

  # Add dependencies (ensure downloads complete before building)
  add_dependencies(_cgal external_downloads)

  # Install
  install(TARGETS _cgal LIBRARY DESTINATION nepath_bindings)

  message(STATUS "CGAL geometry module: ENABLED")
  message(STATUS "CGAL Include Dir: ${CGAL_INCLUDE_DIR}")
  message(STATUS "Boost Include Dir: ${BOOST_INCLUDE_DIR}")
else()
  message(STATUS "CGAL geometry module: DISABLED")
endif()

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen Include Dir: ${EIGEN_INCLUDE_DIR}")
message(STATUS "IPOPT Include Dirs: ${IPOPT_INCLUDE_DIRS}")
message(STATUS "IPOPT Library Dirs: ${IPOPT_LIBRARY_DIRS}")
message(STATUS "IPOPT Libraries: ${IPOPT_LIBRARIES}")
message(STATUS "=======================================")
