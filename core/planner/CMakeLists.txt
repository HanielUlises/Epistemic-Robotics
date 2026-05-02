cmake_minimum_required(VERSION 3.16)
project(epistemic_gbfs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

set(SOURCES
    src/main.cpp
    src/formula.cpp
    src/state.cpp
    src/action.cpp
    src/parser.cpp
    src/product_update.cpp
    src/bisimulation.cpp
    src/heuristic.cpp
    src/search.cpp
)

add_executable(epistemic_gbfs ${SOURCES})

target_include_directories(epistemic_gbfs PRIVATE include)
target_link_libraries(epistemic_gbfs PRIVATE nlohmann_json::nlohmann_json)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_custom_command(TARGET epistemic_gbfs POST_BUILD
    COMMAND ${CMAKE_STRIP} $<TARGET_FILE:epistemic_gbfs>
    COMMENT "Stripping binary"
  )
endif()