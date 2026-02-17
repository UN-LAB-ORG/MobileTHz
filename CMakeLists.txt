cmake_minimum_required(VERSION 3.16)
project(MobileTHz VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(WIN32)
    # Use target_compile_definitions for modern CMake practice
    # This will apply to all targets created after this point
    add_compile_definitions(
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )

    # Check if the compiler is MSVC before adding MSVC-specific flags
    if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")

        # The subsystem flag is also specific to the MSVC linker
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
    endif()
endif()


if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
endif()

# --- Build Type Specific Flags ---
# Debug
if(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi") # No optimization, debug info
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG")
else()
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og") # Optimize for debug experience
endif()

# RelWithDebInfo (Release with Debug Info)
if(MSVC)
    # Find and replace the default inlining flag instead of appending a new one
    string(REPLACE "/Ob1" "/Ob2" CMAKE_CXX_FLAGS_RELWITHDEBINFO_REPLACED "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO_REPLACED} /O2 /Ob2 /Oi /Gy /Gw /GL /arch:AVX2 /fp:fast" CACHE STRING "Flags for Release with Debug Info" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /DEBUG /LTCG /OPT:REF /OPT:ICF")
else()
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g -flto")
endif()

# Release
if(MSVC)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2 /Oi /Gy /Gw /GL /arch:AVX2 /fp:fast")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG /OPT:REF /OPT:ICF")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")
else()
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -flto -fno-strict-aliasing -march=native")
endif()

# Optimize for repeated runs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set ENABLE_DSO based on the operating system
if(WIN32)
    set(ENABLE_DSO OFF)
else()
    set(ENABLE_DSO OFF)
endif()

# Find Qt package
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Charts Svg Network Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Charts Svg Network Concurrent)

if(APPLE OR (UNIX AND NOT ANDROID))
    find_package(PkgConfig REQUIRED)
    # The Makefile shows we need both libftdi1 and libusb-1.0
    pkg_check_modules(FTDI_USB REQUIRED libftdi1 libusb-1.0)
endif()

# Set MOC and UIC search paths
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/Codebase)

# Set custom build directory
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Executables/${CMAKE_SYSTEM_NAME}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Set the output directory for executables
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")

# Define main Qt-related source files
set(MAIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Codebase/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Assets/qtResourceFile.qrc
)

# Force CMake to run MOC on our files
set_property(SOURCE ${MAIN_SOURCES} PROPERTY SKIP_AUTOGEN OFF)

# Define other source files more efficiently
file(GLOB_RECURSE OTHER_SOURCES
    "Codebase/Experimental/*.cpp"
    "Codebase/Experimental/*.h"
    "Codebase/Software/*.cpp"
    "Codebase/Software/*.h"
    "Codebase/UserInterface/*.cpp"
    "Codebase/UserInterface/*.h"
    "Codebase/Simulation/*.cpp"
    "Codebase/Simulation/*.h"
    "Codebase/Algorithm/HierarchicalSearch/*.*"
    "Codebase/Algorithm/IMUAssist/*.*"
    "Codebase/Algorithm/JustInTime_AP/*.*"
    "Codebase/Algorithm/JustInTime_UE/*.*"
    "Codebase/Algorithm/PerfectAlignment/*.*"
    "Codebase/Algorithm/Static/*.*"
    "Codebase/Algorithm/AlgorithmInterface.h"
)

# SimpleBLE setup
set(SIMPLEBLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/imuExp/SimpleBLE-main/simpleble/")
set(SIMPLEBLE_BUILD_DIR "${SIMPLEBLE_DIR}/build_${CMAKE_SYSTEM_NAME}_${CMAKE_BUILD_TYPE}")
set(SIMPLEBLE_INSTALL_DIR "${SIMPLEBLE_DIR}/install_simpleble_${CMAKE_SYSTEM_NAME}_${CMAKE_BUILD_TYPE}")

# Function to build and install SimpleBLE
function(build_simpleble)
    if(NOT EXISTS ${SIMPLEBLE_INSTALL_DIR})
        message(STATUS "Building and installing SimpleBLE...")
        file(MAKE_DIRECTORY ${SIMPLEBLE_BUILD_DIR})

        # Define SimpleBLE CXX Flags conditionally
        set(SIMPLEBLE_CXX_FLAGS "")
        if(WIN32)
            set(SIMPLEBLE_CXX_FLAGS "/D_SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING")
        endif()

        execute_process(
            COMMAND ${CMAKE_COMMAND}
                -S "${SIMPLEBLE_DIR}"
                -B "${SIMPLEBLE_BUILD_DIR}"
                -G "${CMAKE_GENERATOR}"
                "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
                "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
                -DCMAKE_INSTALL_PREFIX=${SIMPLEBLE_INSTALL_DIR}
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DCMAKE_CXX_STANDARD=17
                -DCMAKE_CXX_STANDARD_REQUIRED=ON
                -DCMAKE_CXX_EXTENSIONS=OFF
                "-DCMAKE_CXX_FLAGS=${SIMPLEBLE_CXX_FLAGS}" # Use the conditional flags
            RESULT_VARIABLE result
        )
        if(result EQUAL 0)
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build "${SIMPLEBLE_BUILD_DIR}" --config ${CMAKE_BUILD_TYPE} -j4
                RESULT_VARIABLE result
            )
            if(result EQUAL 0)
                execute_process(
                    COMMAND ${CMAKE_COMMAND} --install "${SIMPLEBLE_BUILD_DIR}" --config ${CMAKE_BUILD_TYPE}
                    RESULT_VARIABLE result
                )
                if(result EQUAL 0)
                    message(STATUS "SimpleBLE built and installed successfully.")
                    return()
                endif()
            endif()
        endif()
        message(FATAL_ERROR "Failed to build and install SimpleBLE")
    endif()
endfunction()

# Call build_simpleble to ensure it's built
build_simpleble()

include_directories(${SIMPLEBLE_INSTALL_DIR}/include)
link_directories(${SIMPLEBLE_INSTALL_DIR}/lib)

# Set up SimpleBLE
set(SimpleBLE_DIR ${SIMPLEBLE_INSTALL_DIR}/lib/cmake/simpleble)
find_package(SimpleBLE REQUIRED)

# Combine all sources
set(PROJECT_SOURCES ${MAIN_SOURCES} ${OTHER_SOURCES})

# Remove specific files from compilation
list(REMOVE_ITEM PROJECT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/rotaryExp/rotarylib/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/rotaryExp/rotarylib/Matlab/rotarylib_wrapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/rotaryExp/rotarylib/Matlab/rotarylib_wrapper.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/rotaryExp/rotarylib/Python/_rotarylib.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental/imuExp/main.cpp"
)

# Ignore the build files in IMU directory
list(FILTER PROJECT_SOURCES EXCLUDE REGEX "Codebase/Experimental/imuExp/SimpleBLE-main/*")

# Create executable
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCES}
    )
endif()

# Set up include directories
target_include_directories(MobileTHz PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Codebase
    ${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental
    ${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Software
    ${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Simulation
    ${SIMPLEBLE_INSTALL_DIR}/include
)

if(FTDI_USB_FOUND)
    target_include_directories(MobileTHz PRIVATE ${FTDI_USB_INCLUDE_DIRS})
endif()

# Set properties
set_target_properties(MobileTHz PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.MobileTHz
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
    LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
)

if(FTDI_USB_FOUND)
    target_link_directories(MobileTHz PRIVATE ${FTDI_USB_LIBRARY_DIRS})
endif()

# Link libraries
target_link_libraries(MobileTHz PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Charts
    Qt${QT_VERSION_MAJOR}::Svg
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Concurrent
    $<$<CONFIG:Debug>:simpleble-debug>
    $<$<CONFIG:Release>:simpleble>
    $<$<CONFIG:RelWithDebInfo>:simpleble-relwithdebinfo>
    $<$<CONFIG:MinSizeRel>:simpleble>
    $<$<PLATFORM_ID:Windows>:ws2_32>
    $<$<PLATFORM_ID:Linux,Darwin>:m>
)

if(FTDI_USB_FOUND)
    target_link_libraries(MobileTHz PRIVATE ${FTDI_USB_LIBRARIES})
endif()

# Link CoreBluetooth framework on macOS
if(APPLE)
    find_library(COREBLUETOOTH_FRAMEWORK CoreBluetooth)
    target_link_libraries(MobileTHz PRIVATE ${COREBLUETOOTH_FRAMEWORK})
endif()

# Add compiler definitions
target_compile_definitions(MobileTHz PRIVATE
    ENABLE_DSO=$<BOOL:${ENABLE_DSO}>
    Experimental_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Experimental"
    SOFTWARE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Software"
    SIMULATION_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Codebase/Simulation"
    CONFIG_ANTENNA_DIR="${EXECUTABLE_OUTPUT_PATH}/ConfigFiles/Antenna"
    CONFIG_PARAMETER_DIR="${EXECUTABLE_OUTPUT_PATH}/ConfigFiles/Parameter"
    CONFIG_POSITION_DIR="${EXECUTABLE_OUTPUT_PATH}/ConfigFiles/Position"
    RESULTS_DIR="${EXECUTABLE_OUTPUT_PATH}/Results"
    "$<$<NOT:$<PLATFORM_ID:Windows>>:_UNIX>"
)

# Windows-specific settings
if(WIN32 AND ENABLE_DSO)
    target_include_directories(MobileTHz PRIVATE "C:/Program Files/IVI Foundation/VISA/Win64/Include")
    target_link_directories(MobileTHz PRIVATE "C:/Program Files/IVI Foundation/VISA/Win64/Lib_x64/msc")
    target_link_libraries(MobileTHz PRIVATE visa64)
endif()

# Icon configuration
set(ICON_PACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/IconPack")

if(WIN32)
    set(APP_ICON_RESOURCE_WINDOWS "${ICON_PACK_DIR}/web/favicon.ico")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/icon.rc.in" "${CMAKE_CURRENT_BINARY_DIR}/icon.rc")
    target_sources(MobileTHz PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/icon.rc")
elseif(APPLE)
    set(MACOSX_BUNDLE_ICON_FILE AppIcon.icns)
    set(APP_ICON_MACOSX "${ICON_PACK_DIR}/macos/AppIcon.icns")
    set_source_files_properties(${APP_ICON_MACOSX} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
    target_sources(MobileTHz PRIVATE ${APP_ICON_MACOSX})
    set_target_properties(MobileTHz PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "${PROJECT_NAME}"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.yourcompany.${PROJECT_NAME}"
        MACOSX_BUNDLE_ICON_FILE "${MACOSX_BUNDLE_ICON_FILE}"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/MacOSXBundleInfo.plist.in"
    )
elseif(UNIX)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.desktop.in" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop" @ONLY)
    install(FILES "${ICON_PACK_DIR}/web/icon-512.png"
            DESTINATION "share/icons/hicolor/512x512/apps"
            RENAME "${PROJECT_NAME}.png")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop"
            DESTINATION "share/applications")
endif()

# Deployment
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}")

if(WIN32)
    set(CPACK_GENERATOR "ZIP")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
else()
    set(CPACK_GENERATOR "TGZ")
endif()

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION .
)

# Install Qt plugins and libraries
set(APP_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${APP_OUTPUT_DIRECTORY}")

if(WIN32)
    get_target_property(_qmake_executable Qt${QT_VERSION_MAJOR}::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E env PATH="${_qt_bin_dir}" "${WINDEPLOYQT_EXECUTABLE}"
            --verbose 0
            --no-compiler-runtime
            --no-opengl-sw
            --dir "${APP_OUTPUT_DIRECTORY}"
            "$<TARGET_FILE:${PROJECT_NAME}>"
    )
elseif(APPLE)
    get_target_property(_qmake_executable Qt${QT_VERSION_MAJOR}::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
    find_program(MACDEPLOYQT_EXECUTABLE macdeployqt HINTS "${_qt_bin_dir}")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND "${MACDEPLOYQT_EXECUTABLE}"
            "$<TARGET_BUNDLE_DIR:${PROJECT_NAME}>"
            -always-overwrite
            -verbose=1
        COMMAND ${CMAKE_COMMAND} -E copy
            "${APP_ICON_MACOSX}"
            "$<TARGET_BUNDLE_CONTENT_DIR:${PROJECT_NAME}>/Resources"
    )
else()
    find_program(LINUXDEPLOY_EXECUTABLE linuxdeploy)
    find_program(LINUXDEPLOY_QT_EXECUTABLE linuxdeploy-plugin-qt)
    if(LINUXDEPLOY_EXECUTABLE AND LINUXDEPLOY_QT_EXECUTABLE)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND "${LINUXDEPLOY_EXECUTABLE}"
                --plugin qt
                --output appimage
                --executable "$<TARGET_FILE:${PROJECT_NAME}>"
                --desktop-file "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.desktop"
                --icon-file "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.png"
        )
    else()
        message(WARNING "linuxdeploy and/or linuxdeploy-plugin-qt not found. Falling back to manual deployment.")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_qt_bin_dir}/../lib"
                "${APP_OUTPUT_DIRECTORY}/lib"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_qt_bin_dir}/../plugins"
                "${APP_OUTPUT_DIRECTORY}/plugins"
        )
    endif()
endif()

include(CPack)

# --- Define Paths Used by the Custom Target ---
# Define these *outside* the target so they are evaluated correctly
set(DEST_CONFIG_DIR "${EXECUTABLE_OUTPUT_PATH}/ConfigFiles")
set(DEST_RESULTS_DIR "${EXECUTABLE_OUTPUT_PATH}/Results")
set(SOURCE_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ConfigFiles")
set(PYTHON_SCRIPT_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/ConfigFiles/plot_results.py")
set(PYTHON_SCRIPT_DEST "${EXECUTABLE_OUTPUT_PATH}/ConfigFiles/plot_results.py")

# --- Define a Custom Target for Managing Output Files ---
add_custom_target(ManageOutputFiles ALL # Add 'ALL' to make it run by default
    COMMAND ${CMAKE_COMMAND} -E echo "--- Managing Output Files ---" # Debug message

    # 1. Clean existing ConfigFiles directory
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${DEST_CONFIG_DIR}"

    # 2. Recreate ConfigFiles structure
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_CONFIG_DIR}/Parameter"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_CONFIG_DIR}/Position"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_CONFIG_DIR}/Antenna"

    # 3. Ensure Results directory exists
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_RESULTS_DIR}"

    # 4. Copy config directories
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SOURCE_CONFIG_DIR}/Parameter"
            "${DEST_CONFIG_DIR}/Parameter"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SOURCE_CONFIG_DIR}/Position"
            "${DEST_CONFIG_DIR}/Position"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SOURCE_CONFIG_DIR}/Antenna"
            "${DEST_CONFIG_DIR}/Antenna"

    # 5. Copy Python script
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PYTHON_SCRIPT_SOURCE}"
            "${PYTHON_SCRIPT_DEST}"

    COMMENT "Cleaning and repopulating output directories (ConfigFiles, Results) and copying script"
    VERBATIM # Ensure commands are executed literally
)

add_dependencies(${PROJECT_NAME} ManageOutputFiles)

# Create package
set(PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}")
add_custom_target(create_package
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/package"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>" "${CMAKE_BINARY_DIR}/package/${PROJECT_NAME}"
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_BINARY_DIR}/package"
        ${CMAKE_COMMAND} -E tar "czf"
        "${CMAKE_BINARY_DIR}/${PACKAGE_FILE_NAME}$<$<PLATFORM_ID:Windows>:.zip>$<$<NOT:$<PLATFORM_ID:Windows>>:.tar.gz>"
        $<$<PLATFORM_ID:Windows>:--format=zip>
        "${PROJECT_NAME}"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Creating package..."
)

# Use multiple cores for building
include(ProcessorCount)
ProcessorCount(N)
if(NOT N EQUAL 0)
    set(CMAKE_BUILD_PARALLEL_LEVEL ${N})
endif()

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(${PROJECT_NAME})
endif()
