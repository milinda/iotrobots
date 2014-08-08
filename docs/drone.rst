Drone
=====

The drone we are using is Parrot Drone Version 2.0.

Objective
---------

The drone has a Camera facing downwards. We would like the drone to track a moving target on the ground and follow that target. The information captured by the drone will be sent to a Cloud environment and will be processed there. The commands to control the drone is calculated in the Cloud and will be sent to the drone.

Overall Architecture
--------------------

The information flow happens as in the following.

Drone --1--> Gateway --2--> Apache Storm

The 1 and 2 communication happens through publish-subscribe messaging. We use RabbitMQ as the publish-subscribe broker.

The driver to control the drone and get information from the drone is written in python. The drone code will capture this information and push it to RabbitMQ message broker.

The Gateway relays this information to Apache Storm and the processing happens in Apache Storm. Apache Storm is a distributed stream processing engine.


BUILDING THE SOURCE CODE
------------------------

The source includes two main components.

1. Applicatio


