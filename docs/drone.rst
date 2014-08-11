Drone
=====

The drone we are using is Parrot Drone Version 2.0.

Objective
---------

The drone has a Camera facing downwards. We would like the drone to track a moving target
on the ground and follow that target. The information captured by the drone will be sent to
a Cloud environment and will be processed there. The commands to control the drone is calculated
in the Cloud and will be sent to the drone.

Overall Architecture
====================

The information flow happens as in the following.

Drone --1--> Gateway --2--> Apache Storm

The 1 and 2 communication happens through publish-subscribe messaging. We use RabbitMQ as the publish-subscribe broker.

The driver to control the drone and get information from the drone is written in python. The drone drive code will receive the video camera data and navigation data and push it to the RabbitMQ message broker.

The Gateway relays this information to Apache Storm and the processing happens in Apache Storm. Apache Storm is a distributed stream processing engine.


BUILDING THE SOURCE CODE
========================

Before building the drone source code you need to build iotcloud project and storm-broker-connectors project.

You need Apache Maven and Java installed in the system.

iotcloud
--------

The source code can be found in https://github.com/iotcloud/iotcloud2.git

Get this source code to local machine using git clone.

Then go inside the iotcloud2 directory and type

mvn clean install

This will build iotcloud.

storm-broker-connectors
-----------------------

The source code can be found in

https://github.com/iotcloud/storm-broker-connectors.git

Get this source code to local machine using git clone.

Then go inside the storm-broker-connectors directory and type

mvn clean install

This will build the storm spouts and bolts that are needed to communicate with storm.

iotrobots
---------

After building the above two projects you are ready to build the iotrobots project.

The source code can be found in

https://github.com/iotcloud/iotrobots.git

Get this source code to local machine using git clone.

Then go inside the iotrobots directory and type

mvn clean install

This will build the drone code. If you only need to compile the drone you can do this by going to drone directory and typing the above command.

Deployment
==========

There are two components of the project that needs to be deployed to get the data from the drone and run the processing in the cloud.