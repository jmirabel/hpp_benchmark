ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")

ADD_EXECUTABLE(benchmarks
  benchmark.cc
  pr2-in-iai-kitchen.cc
  hrp2-on-the-ground.cc
  )

PKG_CONFIG_USE_DEPENDENCY(benchmarks hpp-core)
