Cloud based robot collision avoidance
=========

Before running the program, iotcloud and related components shoud be build and installed. ROS hydro, Rabbitmq 3.4.2 and Zookeeper 3.4.6 should be installed.

1. Build the whole project. If you are going to deploy the topology on remote cluster, refer to the doc in planner module before building.

2. Launch Ros and Zookeeper.

3. Run simulator according to the doc in simulator module.

4. Run iotcloud master and site, deploy sesnors according to the doc in sensors module.

5. Launch topology according to the doc in planners module.

6. Click run button on simulator panel to start robot collision avoidance simulation.

PS: If too many robots are added to the scene, computation delay can be seconds and the calculated velocities are invalid, so robots may collide with each other or wanders around. Normally 4-6 robots on machine with quad-core cpu if all system running on one machine.
