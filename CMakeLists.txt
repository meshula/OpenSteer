cmake_minimum_required(VERSION 3.10)

project(OpenSteer)

set(OPENSTEER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

# Turn on folder usage
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# glfw
set(GLFW_DIR "third-party/glfw")
set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")
add_subdirectory("${GLFW_DIR}")

set(OpenSteer_Headers 
   include/OpenSteer/AbstractVehicle.h
   include/OpenSteer/Annotation.h
   include/OpenSteer/Camera.h
   include/OpenSteer/Clock.h
   include/OpenSteer/Color.h
   include/OpenSteer/Draw.h
   include/OpenSteer/LocalSpace.h
   include/OpenSteer/lq.h
   include/OpenSteer/Obstacle.h
   include/OpenSteer/OldPathway.h
   include/OpenSteer/OpenSteerDemo.h
   include/OpenSteer/Path.h
   include/OpenSteer/Pathway.h
   include/OpenSteer/PlugIn.h
   include/OpenSteer/PolylineSegmentedPath.h
   include/OpenSteer/PolylineSegmentedPathwaySegmentRadii.h
   include/OpenSteer/PolylineSegmentedPathwaySingleRadius.h
   include/OpenSteer/Proximity.h
   include/OpenSteer/QueryPathAlike.h
   include/OpenSteer/QueryPathAlikeBaseDataExtractionPolicies.h
   include/OpenSteer/QueryPathAlikeMappings.h
   include/OpenSteer/QueryPathAlikeUtilities.h
   include/OpenSteer/SegmentedPath.h
   include/OpenSteer/SegmentedPathAlikeUtilities.h
   include/OpenSteer/SegmentedPathway.h
   include/OpenSteer/SharedPointer.h
   include/OpenSteer/SimpleVehicle.h
   include/OpenSteer/StandardTypes.h
   include/OpenSteer/SteerLibrary.h
   include/OpenSteer/UnusedParameter.h
   include/OpenSteer/Utilities.h
   include/OpenSteer/Vec3.h
   include/OpenSteer/Vec3Utilities.h)

set(OpenSteer_Sources
   src/Camera.cpp
   src/Clock.cpp
   src/lq.c
   src/Obstacle.cpp
   src/OldPathway.cpp
   src/Path.cpp
   src/Pathway.cpp
   src/PlugIn.cpp
   src/PolylineSegmentedPath.cpp
   src/PolylineSegmentedPathwaySegmentRadii.cpp
   src/PolylineSegmentedPathwaySingleRadius.cpp
   src/SegmentedPath.cpp
   src/SegmentedPathway.cpp
   src/SimpleVehicle.cpp
   src/TerrainRayTest.cpp
   src/Vec3.cpp
   src/Vec3Utilities.cpp)

set(OpenSteer_Misc
   third-party/glfw/deps/glad.c
   src/OpenSteerDemo.cpp
   src/Draw.cpp
   src/Color.cpp
   src/main.cpp
   src/TerrainRayTest.h
   test/PolylineSegmentedPathTest.h
   test/PolylineSegmentedPathwaySingleRadiusTest.h
   test/SharedPointerTest.h
   win32/resource.h
   plugins/Boids.cpp
   plugins/CaptureTheFlag.cpp
   plugins/LowSpeedTurn.cpp
   plugins/MapDrive.cpp
   plugins/MultiplePursuit.cpp
   plugins/OneTurning.cpp
   plugins/Pedestrian.cpp
   plugins/PedestriansWalkingAnEight.cpp
   plugins/Soccer.cpp)

set(OpenSteer_Tests
   test/PolylineSegmentedPathTest.cpp
   test/PolylineSegmentedPathwaySingleRadiusTest.cpp
   test/SharedPointerTest.cpp
   test/TestMain.cpp)

add_library(libopensteer STATIC ${OpenSteer_Headers} ${OpenSteer_Sources})
target_include_directories(libopensteer PUBLIC include)
source_group(OpenSteer\\include FILES ${OpenSteer_Headers})
source_group(OpenSteer\\src FILES ${OpenSteer_Sources})
install(FILES ${OpenSteer_Headers} DESTINATION include/OpenSteer)
add_library(OpenSteer::Lib ALIAS libopensteer)

add_executable(OpenSteerDemo ${OpenSteer_Misc})
target_link_libraries(OpenSteerDemo OpenSteer::Lib)
target_link_libraries(OpenSteerDemo "glfw" ${GLFW_LIBRARIES})
target_include_directories(OpenSteerDemo PRIVATE ${GLFW_DIR}/include)
target_include_directories(OpenSteerDemo PRIVATE ${GLFW_DIR}/deps)
target_compile_definitions(OpenSteerDemo PRIVATE "GLFW_INCLUDE_NONE")
