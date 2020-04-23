ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-manipulation >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-manipulation-urdf >= 4")

ADD_EXECUTABLE(benchmarks
  benchmark.cc
  pr2-in-iai-kitchen.cc
  hrp2-on-the-ground.cc
  romeo-placard.cc
  )

PKG_CONFIG_USE_DEPENDENCY(benchmarks hpp-core)
PKG_CONFIG_USE_DEPENDENCY(benchmarks hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY(benchmarks hpp-manipulation-urdf)
