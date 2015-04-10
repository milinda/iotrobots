import paramiko
import time

sshNZ = paramiko.SSHClient()
sshNZ.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshNZ.connect('10.39.1.18', username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshBR = paramiko.SSHClient()
sshBR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshBR.connect('10.39.1.28', username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshI = paramiko.SSHClient()
sshI.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshI.connect('10.39.1.26', username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshIR = paramiko.SSHClient()
sshIR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshIR.connect('10.39.1.26', username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

def copy_scan_matcher(ssh, topologyFile):
    print "compiling"
    cmd = 'cp ' + str(topologyFile) + ' serial/src/main/java/cgl/iotrobots/slam/core/scanmatcher/ScanMatcher.java'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/slam
    ''' + cmd + '''
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()

def compile_program(ssh, topologyFile):
    print "compiling"
    cmd = 'cp ' + str(topologyFile) + ' streaming/src/main/resources/topology.yaml'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/slam
    ''' + cmd + '''
    mvn clean install -Dmaven.test.skip=true
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()

def restart_zk(ssh):
    print "compiling"
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    sudo service supervisor stop
    sleep 5
    rm -rf /tmp/zookeeper/
    sudo service supervisor start
    sleep 60
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()

def exec_storm(ssh, particles, parallel):
    print "executing storm commands"
    cmd = './bin/storm jar ~/projects/iotrobots/slam/streaming/target/iotrobots-slam-streaming-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.streaming.SLAMTopology -name slam_processor -ds_mode 0 -p ' + str(parallel) + ' -pt ' + str(particles) + ' -i'
    # cmd = './bin/storm jar ~/projects/iotrobots/slam/streaming/target/iotrobots-slam-streaming-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.streaming.SLAMTopology -name slam_processor -ds_mode 0 -p ' + str(parallel) + ' -pt ' + str(particles)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd deploy/storm
    ./bin/storm kill slam_processor -w 1
    sleep 30
    ''' + cmd + '''
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(30)

def exec_rabbit(ssh):
    print "starting rabbitmq...."
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    pid=`ps ax | grep "rabbitmq" | awk '{print $1}'`
    sudo kill $pid
    sleep 5
    cd /home/ubuntu/deploy/rabbitmq_server-3.3.2/sbin
    sudo ./rabbitmq-server > /dev/null 2>&1 &
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(5)

def exec_iotcloud(ssh):
    print "starting iotcloud...."
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    pid=`jps | grep "LocalCluster" | awk '{print $1}'`
    kill $pid
    sleep 5
    cd deploy/iotcloud2-bin-1.0-SNAPSHOT
    ./bin/iotcloud local > /dev/null 2>&1 &
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(5)

def exec_sensor(ssh):
    print "starting sensor...."
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd deploy/iotcloud2-bin-1.0-SNAPSHOT
    ./bin/iotcloud jar repository/sensors/iotrobots-slam-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.sensor.SlamSensor -s local -sim -url "amqp://10.39.1.28:5672"  > /dev/null 2>&1 &
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(5)

def run_test(ssh, test, parallel, particles, input, simbad):
    print "running test...."
    cmd = 'java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://10.39.1.28:5672" ' + str(input) + ' results_dir4/' +str(test) + '/' + str(particles) + '_' + str(parallel) + ' ' +str(simbad) + ' false 1000'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/simulator
    ''' + cmd + '''
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()

def run_simbard_test():
    tasks = [4, 8, 12, 16, 20]
    particles = [20, 60, 100]

    compile_program(sshNZ, "simbard.yaml")
    for par in particles:
        for t in tasks:
            exec_rabbit(sshBR)
            exec_iotcloud(sshI)
            exec_sensor(sshI)
            exec_storm(sshNZ, par, t)
            run_test(sshIR, 'sim', t, par, 'data/simbard_1.txt', 'true')

def run_simbard_cost_test():
    tasks = [4, 8, 12, 16, 20]
    particles = [20, 60, 100]

    copy_scan_matcher(sshNZ, "ScanMatcher.java")
    compile_program(sshNZ, "simbard.yaml")
    for par in particles:
        for t in tasks:
            exec_rabbit(sshBR)
            exec_iotcloud(sshI)
            exec_sensor(sshI)
            exec_storm(sshNZ, par, t)
            run_test(sshIR, 'const', t, par, 'data/simbard_1.txt', 'true')

def run_aces_test():
    tasks = [4, 8, 12, 16, 20]
    particles = [20, 60, 100]
    compile_program(sshNZ, "aces.yaml")
    for par in particles:
        for t in tasks:
            exec_rabbit(sshBR)
            exec_iotcloud(sshI)
            exec_sensor(sshI)
            exec_storm(sshNZ, par, t)
            run_test(sshIR, 'aces', t, par, 'data/aces_300.txt', 'false')

def run_rs_test():
    tasks = [4, 8, 12, 16, 20]
    particles = [20, 60, 100]
    compile_program(sshNZ, "simbard_rs.yaml")
    for par in particles:
        for t in tasks:
            exec_rabbit(sshBR)
            exec_iotcloud(sshI)
            exec_sensor(sshI)
            exec_storm(sshNZ, par, t)
            run_test(sshIR, 'rs', t, par, 'data/simbard_1.txt', 'true')

def start_cluster(par, t):
    exec_rabbit(sshBR)
    exec_iotcloud(sshI)
    exec_sensor(sshI)
    exec_storm(sshNZ, par, t)

def run_serial(ssh, par, file, simbard):
    print "running test...."
    cmd = "java -Xmx6G -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedSimulator true data/" + str(file) + " " + str(par) + " " + str(simbard) + " 1 false > " + str(file) + str(par)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/simulator
    ''' + cmd + '''
    exit
    ''')
    #print stdout.read()
    stdout.close()
    stdin.close()

def main():
    restart_zk(sshNZ)
    run_aces_test()
    restart_zk(sshNZ)
    run_simbard_test()
    restart_zk(sshNZ)
    run_rs_test()
    restart_zk(sshNZ)
    run_simbard_cost_test()

    # run_serial(sshI, 20, "simbard_1.txt", "true")
    # run_serial(sshI, 60, "simbard_1.txt", "true")
    # run_serial(sshI, 100, "simbard_1.txt", "true")
    # run_serial(sshI, 20, "aces.txt", "false")
    # run_serial(sshI, 60, "aces.txt", "false")
    # run_serial(sshI, 100, "aces.txt", "false")

if __name__ == "__main__":
    main()
