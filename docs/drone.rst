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

Lets look at the source tree first. The code contains two main components. The processor and the sensor

iotrobots --> drone -->
                      => processor  [contains code of the stom topology]
                      => sensor     [the gateway code to relay the messages coming from drone to cloud processing layer]

The source code can be found in

https://github.com/iotcloud/iotrobots.git

Get this source code to local machine using git clone.

Then go inside the iotrobots directory and type

mvn clean install

This will build the drone code. If you only need to compile the drone you can do this by going to drone directory and typing the above command.

Deployment
==========

There are two components of the project that needs to be deployed to get the data from the drone and run the processing in the cloud. They are the sensor and the processor.

Cloud Deployment
----------------

We use FutureGrid as our cloud provider to deploy the components. Please read the FutureGrid OpenStack manual_ before proceeding.

Cloud deployment is done using three frameworks.

1. Apache Storm
2. IOTCloud
3. RabbitMQ broker

Apache Storm is running on 4 nodes in FugureGrid. These are the nodes running Apache Storm

skamburu-storm-nimus-zookeeper
skamburu-supervisor-01
skamburu-supervisor-02
skamburu-supervisor-03

The broker is running on the node: skamburu-broker-01

The iotcloud is running on the node: skamburu-iot-01

You can find the IPs of these nodes using the nova commands after loggin in to Futuregrid.

The skamburu-storm-nimus-zookeeper node run Apache Storm Nimbus and ZooKeeper. You can deploy the code by logging in to this node.

First log in to Futuregrid. Then use the command

ssh -l ubuntu -i private_key ip

to log in to the machine.

After you log in go to the ~/projects/iotrobots folder.

Then do a git pull to get your latest changes.

After that do a

mvn clean install

to build the project with the changes.

Now go to ~/deploy/storm

You need to kill the existing topology to deploy the new topology with the changes

You can use

./bin/storm kill drone_processor

Where drone_processor is the name of the running topology. Storm will take about 30 seconds to kill the running topology.

After that you can deploy the new topology with the command

./bin/storm jar ~/projects/iotrobots/drone/processor/target/drone-processor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.st.storm.DroneProcessorTopology -url amqp://broker_ip:5672 -name drone_processor -ds_mode 2

Make sure we se


.. _manual: http://www.python.org/
