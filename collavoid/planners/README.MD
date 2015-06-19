Deploy on Storm
===============

1. Build the planner module

If you are going to run in local mode, make sure scope of storm-core dependency in iotrobots-collavoid-parent.pom is NOT 

provided(comment the entry) when building the module otherwise set it to provided.

2. If you are going to deploy on cluster, install storm in client machine

3. Deploy

Local mode:

In target dir, run java -jar planners-collavoid-1.0-SNAPSHOT-jar-with-dendencies.jar -l

Remote mode:

To deploy on cluster, first zk.servers in resource file topology.yaml should be configured. Then run

./bin/storm jar /pathToTheJar/planners-collavoid-1.0-SNAPSHOT-jar-with-dependencies.jar

Several arguments can be set when running the command.

Note: Run zookeeper first.