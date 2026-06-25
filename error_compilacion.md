/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:82:1: error: invalid suffix "on" on floating constant
   82 | .0on (Serial/USB)
      | ^~~~
/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:82:1: error: expected identifier or ‘(’ before numeric constant
In file included from /mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:29:
/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c: In function ‘main’:
/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:210:85: error: ‘PORT_NAME’ undeclared (first use in this function)
  210 | ("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
      |                                                             ^~~~~~~~~

/mnt/hgfs/mip_sdk_shared/src/c/microstrain/logging.h:147:72: note: in definition of macro ‘MICROSTRAIN_LOG_LOG’
  147 | ROSTRAIN_LOG_LOG(level, ...) microstrain_logging_log(level, __VA_ARGS__)
      |                                                             ^~~~~~~~~~~

/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:210:5: note: in expansion of macro ‘MICROSTRAIN_LOG_INFO’
  210 |     MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
      |     ^~~~~~~~~~~~~~~~~~~~
/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:210:85: note: each undeclared identifier is reported only once for each function it appears in
  210 | ("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
      |                                                             ^~~~~~~~~

/mnt/hgfs/mip_sdk_shared/src/c/microstrain/logging.h:147:72: note: in definition of macro ‘MICROSTRAIN_LOG_LOG’
  147 | ROSTRAIN_LOG_LOG(level, ...) microstrain_logging_log(level, __VA_ARGS__)
      |                                                             ^~~~~~~~~~~

/mnt/hgfs/mip_sdk_shared/examples/c/7_series/threading/7_series_threading_example.c:210:5: note: in expansion of macro ‘MICROSTRAIN_LOG_INFO’
  210 |     MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);
      |     ^~~~~~~~~~~~~~~~~~~~
make[2]: *** [examples/c/7_series/threading/CMakeFiles/7_series_threading_example_c.dir/build.make:76: examples/c/7_series/threading/CMakeFiles/7_series_threading_example_c.dir/7_series_threading_example.c.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:857: examples/c/7_series/threading/CMakeFiles/7_series_threading_example_c.dir/all] Error 2
make: *** [Makefile:136: all] Error 2
