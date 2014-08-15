Deploy on Storm
===============

1. Build the processor module 
2. Run the storm command 

./bin/storm jar ~/projects/iotrobots/turtlebot/processor/target/turtle-processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.storm.FollowerTopology -name turtle_processor -ds_mode 0

Deploy the sensor
=================

1. Build the sensor module and copy the jar with dependencies to repository/sensors directory of IOTCloud
2. Go to IOTCloud master and Run the command

./bin/iotcloud jar repository/sensors/turtle-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.TSensor -local_ip 156.56.93.58 -ros_master http://156.56.95.214:11311 -url amqp://localhost:5672 -mode nt 

-url argument gives the URL of the RabbitMQ server, where the TurtleBot receives kinect messages
