cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(solver-visualisation)

set(CMAKE_CXX_STANDARD 20)

set(SOURCE_FILES Vector.cpp
        Node.cpp
        Graph.cpp
        SpaceGrid.cpp
        Interaction.cpp
        cnf3d2.cpp
        Display.cpp
        API.cpp
        CryptoWalkSAT.cpp
        WalkSAT.c)
set(HEADERS Vector.h
        Node.h
        Graph.h
        SpaceGrid.h
        Interaction.h
        Display.h
        API.h
        CryptoWalkSAT.h
        WalkSAT.h)
set(CC_FLAGS "-O3 -Wall -DNDEBUG -DUSE_SPACE_GRID")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_FLAGS}")

set(VTK_DIR "/home/cormac/projects/VTK-build")

find_package(VTK COMPONENTS
        CommonColor
        CommonCore
        CommonDataModel
        FiltersGeneral
        FiltersSources
        InfovisLayout
        InteractionStyle
        RenderingCore
        RenderingFreeType
        RenderingOpenGL2
        ViewsCore
        ViewsInfovis)

find_package(cryptominisat5 REQUIRED)

include_directories(${ZeroMQ_INCLUDE_DIR} ${cppzmq_INCLUDE_DIR})

message(STATUS "Cryptominisat version: ${CRYPTOMINISAT5_VERSION_MAJOR}.${CRYPTOMINISAT5_VERSION_MINOR}")

add_executable(s_vis MACOSX_BUNDLE ${SOURCE_FILES} ${HEADERS})

if (VTK_FOUND)
    message(STATUS "VTK_VERSION: ${VTK_VERSION}")
    if (VTK_VERSION VERSION_LESS "8.90.0")
        # old system
        include(${VTK_USE_FILE})
    else ()
        # vtk_module_autoinit is needed
        vtk_module_autoinit(
                TARGETS s_vis
                MODULES ${VTK_LIBRARIES}
        )
    endif ()
    target_link_libraries(s_vis PRIVATE ${VTK_LIBRARIES})
else ()
    message("Skipping VTK ${VTK_NOT_FOUND_MESSAGE}")
endif ()

target_link_libraries(s_vis PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${CRYPTOMINISAT5_LIBRARIES}
        zmq)
