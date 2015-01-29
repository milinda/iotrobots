Deploy on Storm
===============
In developing!

1. Build the planner module 
2. Run the storm command 

./bin/storm jar /dirToJar/JarName.jar MainClass TopologyNameAsParameter

example

./bin/storm jar ~/Dropbox/git/iotrobots/collavoid/planners/target/planners-collavoid-1.0-SNAPSHOT-jar-with-dependencies.jar \
cgl.iotrobots.collavoid.iotTopology.runIotTopology collavoid