# ホスト (Mac / PC) 上でコアロジックの単体テストをビルドするための CMake プロジェクト。
# ソースは Arduino スケッチと共通の ../src を直接参照する（二重管理を避けるため）。
#
#   cd core
#   cmake -B build
#   cmake --build build
#   ./build/test_vtxmon_core

cmake_minimum_required(VERSION 3.16)
project(vtxmon_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(VTXMON_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../src")

# プラットフォーム非依存のアルゴリズムのみを対象とする。
# app.cpp / usb_host_sdr.cpp / ui_renderer.cpp は Arduino 依存なので含めない。
add_library(vtxmon_core STATIC
    "${VTXMON_SRC_DIR}/esp_sdr_protocol.cpp"
    "${VTXMON_SRC_DIR}/dsp_stitcher.cpp"
    "${VTXMON_SRC_DIR}/channel_analysis.cpp"
)

target_include_directories(vtxmon_core PUBLIC "${VTXMON_SRC_DIR}")

enable_testing()

add_executable(test_vtxmon_core
    test/test_all.cpp
)

target_link_libraries(test_vtxmon_core PRIVATE vtxmon_core)

add_test(NAME core_tests COMMAND test_vtxmon_core)
