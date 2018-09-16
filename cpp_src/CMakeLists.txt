cmake_minimum_required(VERSION 3.12)

function(Flag flag init_flag flag_message)
    if(NOT DEFINED ${flag})
        set(${flag} ${init_flag} CACHE BOOL "${flag_message} : true or false" FORCE)
    endif()
endfunction(Flag)

Flag(USE_Boost true "Find and link Boost library")
Flag(USE_CV true "Find and link OpenCV library")

#project : プログラム全体の構成単位
#基本的にプログラム全体を格納しているディレクトリの名前をプロジェクト名とする
if(NOT PROJECT_NAME)
    get_filename_component(DEFAULT_PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)
    set(PROJECT_NAME ${DEFAULT_PROJECT_NAME} CACHE STRING "Project name" FORCE)

endif(NOT PROJECT_NAME)
project(${PROJECT_NAME} C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CXX_SOURCE_FILE_EXTS cpp;cxx)
set(CXX_INCLUDE_FILE_EXTS hpp;h)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

#ビルドの対象と実行ファイル名
add_executable(calcAR
                    calcAR.h
                    calcAR.cpp)

#Boostライブラリを探す
if(USE_Boost)
    #必要なライブラリの場合はREQUIREDを付ける
    find_package(Boost REQUIRED)

    if(Boost_FOUND)
        message("---- Found Boost ${Boost_INCLUDE_DIRS}")
        set(INCLUDE_DIRECTORIES ${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIR})

    else(Boost_FOUND)
        message(FATAL_ERROR " ---- Not Found Boost")

    endif(Boost_FOUND)

    set(ALL_LINK_LIBRARIES ${ALL_LINK_LIBRARIES} ${Boost_LIBRARIES} Boost::thread)

endif(USE_Boost)

if(USE_CV)
    find_package(OpenCV REQUIRED)

    if(OpenCV_FOUND)
        message("---- Found OpenCV ${OpenCV_INCLUDE_DIRS}")
        set(INCLUDE_DIRECTORIES ${INCLUDE_DIRECTORIES} ${OpenCV_INCLUDE_DIRS})

    else(OpenCV_FOUND)
        message(FATAL_ERROR "---- Not Found OpenCV")

    endif(OpenCV_FOUND)

    set(ALL_LINK_LIBRARIES ${ALL_LINK_LIBRARIES} ${OpenCV_LIBRARIES})

endif(USE_CV)