cmake_minimum_required(VERSION 3.22)

project(SpatialAudioEngine VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "" FORCE)

# Paths.
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(RUST_TARGET_DIR "${REPO_ROOT}/target/release")
set(RUST_STATIC_LIB "${RUST_TARGET_DIR}/libengine.a")
set(RUST_INCLUDE_DIR "${REPO_ROOT}/crates/engine/include")
set(HRTF_BIN              "${REPO_ROOT}/data/hrtf_decoder_native.bin")
set(HRTF_POST_A_BIN       "${REPO_ROOT}/data/hrtf_post_filter_a.bin")
set(HRTF_POST_B_BIN       "${REPO_ROOT}/data/hrtf_post_filter_b.bin")

# --- Rust engine staticlib ------------------------------------------------

# Build the engine staticlib via cargo. Always invoke cargo — it
# is fast on incremental no-op builds.
add_custom_target(BuildEngineRust ALL
    COMMAND cargo build --release -p engine --features c-api
    WORKING_DIRECTORY "${REPO_ROOT}"
    BYPRODUCTS "${RUST_STATIC_LIB}"
    COMMENT "Building engine staticlib (cargo --release --features c-api)"
    VERBATIM
)

add_library(engine_rust STATIC IMPORTED GLOBAL)
add_dependencies(engine_rust BuildEngineRust)
set_target_properties(engine_rust PROPERTIES
    IMPORTED_LOCATION "${RUST_STATIC_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${RUST_INCLUDE_DIR}"
)

# On macOS, Rust staticlibs need these system libs.
if(APPLE)
    target_link_libraries(engine_rust INTERFACE
        "-framework CoreFoundation"
        "-framework Security"
    )
endif()

# --- JUCE -----------------------------------------------------------------

include(FetchContent)
FetchContent_Declare(
    JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG        8.0.6
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(JUCE)

# --- Plugin target --------------------------------------------------------

juce_add_plugin(SpatialAudioEngine
    COMPANY_NAME           "Spatial Audio"
    BUNDLE_ID              com.spatialaudio.engine
    PLUGIN_MANUFACTURER_CODE Sptl
    PLUGIN_CODE            SaEn
    FORMATS                AU Standalone
    PRODUCT_NAME           "Spatial Audio Engine"
    IS_SYNTH               FALSE
    NEEDS_MIDI_INPUT       FALSE
    NEEDS_MIDI_OUTPUT      FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE
)

# Bundled HRTF blobs as JUCE BinaryData (main HRTF + §13 W-binauralizer pair).
juce_add_binary_data(SpatialAudioEngineResources
    HEADER_NAME BinaryData.h
    NAMESPACE   SpatialAudioBinary
    SOURCES
        "${HRTF_BIN}"
        "${HRTF_POST_A_BIN}"
        "${HRTF_POST_B_BIN}"
)

target_sources(SpatialAudioEngine PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginProcessor.h
    Source/PluginEditor.cpp
    Source/PluginEditor.h
)

target_compile_definitions(SpatialAudioEngine PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(SpatialAudioEngine PRIVATE
    SpatialAudioEngineResources
    engine_rust
    juce::juce_audio_utils
    juce::juce_dsp
    juce::juce_audio_plugin_client
    juce::juce_recommended_config_flags
    juce::juce_recommended_lto_flags
    juce::juce_recommended_warning_flags
)
