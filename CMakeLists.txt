cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(3Dvis)

set(SOURCE_FILES Vector.cpp
        Node.cpp
        Graph.cpp
        SpaceGrid.cpp
        SceneParameters.cpp
        Interaction.cpp
        cnf3d2.cpp
        InteractionCallback.cpp
        Display.cpp)
set(HEADERS Vector.h
        Node.h
        Graph.h
        SpaceGrid.h
        SceneParameters.h
        Interaction.h
        InteractionCallback.h
        Display.h)
set(CC_FLAGS "-O3 -Wall -DNDEBUG -DUSE_SPACE_GRID")
# set(LN_FLAGS "-lXi -lX11 -lXext -lGL -lGLU -lglut -lX11")

set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL)
find_package(GLUT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_FLAGS}")

set(VTK_DIR "/home/cormac/VTK-build")

find_package(VTK COMPONENTS
        CommonColor
        CommonCore
        CommonDataModel
        FiltersGeneral
        RenderingCore
        RenderingFreeType
        RenderingOpenGL2
        InteractionStyle)

add_executable(cnf3d2 MACOSX_BUNDLE ${SOURCE_FILES} ${HEADERS})

if (VTK_FOUND)
    message(STATUS "VTK_VERSION: ${VTK_VERSION}")
    if (VTK_VERSION VERSION_LESS "8.90.0")
        # old system
        include(${VTK_USE_FILE})
    else ()
        # vtk_module_autoinit is needed
        vtk_module_autoinit(
                TARGETS cnf3d2
                MODULES ${VTK_LIBRARIES}
        )
    endif ()
    target_link_libraries(cnf3d2 PRIVATE ${VTK_LIBRARIES})
else ()
    message("Skipping VTK ${VTK_NOT_FOUND_MESSAGE}")
endif ()

target_link_libraries(cnf3d2 PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES})
