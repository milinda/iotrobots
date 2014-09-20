Build the Kinect Driver
=======================

Get the libfreenect source version 4.0.3

https://github.com/OpenKinect/libfreenect/releases

Extract the tar and go inside the tar. Execute the following commands

mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib64/

This will build the libfreenect in your system

./runNTest.sh amqp://149.165.159.39:5672


Deploy on Storm
===============

1. Build the processor module 
2. Run the storm command 

./bin/storm jar ~/projects/iotrobots/turtlebot/processor/target/turtle-processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.storm.FollowerTopology -name turtle_processor -ds_mode 0

Deploy the sensor
=================

1. Build the sensor module and copy the jar with dependencies to repository/sensors directory of IOTCloud
2. Go to IOTCloud master and Run the command

./bin/iotcloud jar repository/sensors/turtle-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.turtlebot.TSensor -local_ip 156.56.93.58 -ros_master http://156.56.95.214:11311 -url amqp://localhost:5672 -mode nt -s "iot1" -n 1

-url argument gives the URL of the RabbitMQ server, where the TurtleBot receives kinect messages
