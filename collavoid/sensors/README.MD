Build Iotcloud
=================
To provide necessary dependencies, need to build and install the whole project including iotcloud, sensorstream, and storm connector.

mvn clean install

Then go to the distribution directory and type

mvn clean install

This will build the zip distribution in the target directory. Unzip the zip file and you are ready to run IOTCloud.

How to Run
==========
Before running iotcloud, first need to start the zooKeeper server.

There are two services we need to run in-order to bring IOTCloud

1. Master

2. Site

To start the site go to the iotcloud distribution directory and type

./bin/iotcloud master

Then open a new terminal, go to iotcloud distribution directory and type the following to start a site.

./bin/iotcloud site

Configurations are in the conf directory of iotcloud directory.


Deploy the sensor
=================
In developing!

1. Build the sensor module and copy the jar with dependencies to repository/sensors directory of IOTCloud

2. Go to IOTCloud master and Run the command

./bin/iotcloud jar repository/sensors/sensors-collavoid-1.0-SNAPSHOT-jar-with-dependencies.jar AgentSensor