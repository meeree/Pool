cmake_minimum_required (VERSION 2.6)
project (Pool)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG  "-g")
set(CMAKE_CXX_FLAGS_RELEASE "")

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(Armadillo REQUIRED)

# add flags. See https://stackoverflow.com/questions/24532853/how-to-add-linker-flag-for-libraries-with-cmake/24534867
SET(CMAKE_SHARED_LINKER_FLAGS, "-DARMA_DONT_USE_WRAPPER,-DARMA_USE_SUPERLU")

include_directories( "${PROJECT_BINARY_DIR}" "src/*.h" "SGV3D/src/*.h" ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

file( GLOB GLIB_SOURCE "src/*.cpp" "SGV3D/src/*.cpp" ) 
 
# add the  executable
add_executable(pool ${GLIB_SOURCE})
target_link_libraries(pool ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${GLM_LIBRARIES} glfw ${ARMADILLO_LIBRARIES})
