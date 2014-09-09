Topology
========

./bin/storm jar ~/projects/iotrobots/performance/processor/target/performance-processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.perf.proc.PerformanceTopology -name perf_rabbit -ds_mode 0 -trp r 
 
Sensor
======

./bin/iotcloud jar repository/sensors/performance-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.perf.sensor.PerformanceSensor -f ~/projects/test.yaml -n 2 -s "iot1 iot2" -t k
