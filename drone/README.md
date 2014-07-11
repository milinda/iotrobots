Deploy on Storm
===============

1. Build the processor module i.e. 
   mvn clean install 
2. Go to the storm directory
   cd ~/storm_home
3. Run the storm command

./bin/storm jar path_to_jar_file cgl.iotrobots.st.storm.SphereTrackingTopology st
 
 Here is an example
 
./bin/storm jar ~/dev/projects/iotrobots/drone/processor/target/drone-processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.st.storm.SphereTrackingTopology -url amqp://broker_ip:5672 -name drone_processor
 
Deploy the sensor
=================

1. Build the sensor module and copy the jar with dependencies to repository/sensors directory of IOTCloud
2. Go to IOTCloud master and Run the command

./bin/iotcloud jar repository/sensors/drone-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.st.STSensor -url amqp://local_broker_ip:5672

-url argument gives the URL of the local RabbitMQ server, where the Drone sends video messages
