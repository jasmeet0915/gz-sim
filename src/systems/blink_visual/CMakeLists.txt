gz_add_system(blink-visual
  SOURCES
    BlinkVisual.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sim${PROJECT_VERSION_MAJOR}-rendering
)
