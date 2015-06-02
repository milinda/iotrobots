Storm options
-------------
./bin/jstorm jar ~/dev/projects/iotrobots/collectives/target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.BroadCastTopology  -name bcast_processor -ds_mode 0 -p 8
./bin/jstorm kill bcast_processor

DataGenerator options
---------------------
"amqp://149.165.158.215:5672" 2000