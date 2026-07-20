add_library(ngc_pendant STATIC
        VistaCncP2sDisplay.cpp
        VistaCncP2sDriver.cpp
        VistaCncP2sManager.cpp
        VistaCncP2sProfile.cpp
        VistaCncP2sProtocol.cpp)

if(WIN32)
    target_sources(ngc_pendant PRIVATE platform/WindowsHidTransport.cpp)
    target_link_libraries(ngc_pendant PRIVATE hid setupapi)
else()
    target_sources(ngc_pendant PRIVATE platform/UnsupportedHidTransport.cpp)
endif()

target_include_directories(ngc_pendant PUBLIC
        ${PROJECT_SOURCE_DIR}/src/include)

ngc_target_defaults(ngc_pendant)

add_executable(ngc_pendant_tests test.cpp)
target_link_libraries(ngc_pendant_tests PRIVATE ngc_pendant)
ngc_target_defaults(ngc_pendant_tests)
add_test(NAME ngc_pendant_tests COMMAND ngc_pendant_tests)
