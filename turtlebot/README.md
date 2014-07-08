Deploy on Storm
===============

1. Build the processor module 
2. Run the storm command 

./bin/storm jar ~/projects/iotrobots/turtbot/processor/target/processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.storm.FollowerTopology image_proc

Deploy the sensor
=================

1. Build the sensor module and copy the jar with dependencies to repository/sensors directory of IOTCloud
2. Go to IOTCloud master and Run the command

./bin/iotcloud jar repository/sensors/iotrobots-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.TSensor -url amqp://localhost:5672

-url argument gives the URL of the RabbitMQ server, where the TurtleBot receives kinect messages
