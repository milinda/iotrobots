Build Iotcloud
=================

For developing, to provide necessary dependencies, need to build and install the whole project including iotcloud2, 

sensorstream, and storm connector.

run mvn clean install in each project

Then go to the iotcloud2 distribution directory and run

mvn clean install

This will build the zip distribution in the target directory. Unzip the zip file and you are ready to run IOTCloud.

How to Run
==========

Before running iotcloud, first need to start the zooKeeper server.

There are two services we need to run in-order to bring up IOTCloud

1. Master

2. Site

To start the site go to the iotcloud2 distribution directory and type

./bin/iotcloud master

Then open a new terminal, go to iotcloud2 distribution directory and type the following to start a site.

./bin/iotcloud site

Configurations are in the conf directory of iotcloud2 directory.

Deploy the sensor
=================

1. Run mvn clean install to build the sensor module and copy the jar with dependencies in target dir to 

repository/sensors directory of iotcloud2

2. In iotcloud2 dir run

./bin/iotcloud jar repository/sensors/sensors-collavoid-1.0-SNAPSHOT-jar-with-dependencies.jar \\

cgl.iotrobots.collavoid.sensors.CollAvoidSensor -n <number of robots,default is 2>

Here n should be the same as the number of robots running in the simulator

Remember sensor uses ROS to relay velocity command, so before deploying sensor, ros server should be launched.