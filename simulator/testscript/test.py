import paramiko
import time

ipNz = '10.39.1.18'
ipB = '10.39.1.28'
ipI = '10.39.1.26'

sshNZ = paramiko.SSHClient()
sshNZ.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshNZ.connect(ipNz, username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshBR = paramiko.SSHClient()
sshBR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshBR.connect(ipB, username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshI = paramiko.SSHClient()
sshI.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshI.connect(ipI, username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

sshIR = paramiko.SSHClient()
sshIR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshIR.connect(ipI, username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

def copy_scan_matcher(ssh, topologyFile):
    print "compiling"
    cmd = 'cp ../simulator/testscript/' + str(topologyFile) + ' serial/src/main/java/cgl/iotrobots/slam/core/scanmatcher/ScanMatcher.java'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/slam
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def compile_program(ssh, topologyFile):
    print "compiling"
    cmd = 'cp ../simulator/testscript/' + str(topologyFile) + ' streaming/src/main/resources/topology.yaml'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/slam
    ''' + cmd + '''
    mvn clean install -Dmaven.test.skip=true
    exit
    ''')
    print stdout.read()
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
    print stdout.read()
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
    print stdout.read()
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
    print stdout.read()
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
    print stdout.read()
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
    print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(5)

aces_times = {20: [300, 250, 200, 200, 200], 60: {400, 350, 300, 300, 300}, 100: {600, 500, 400, 400, 400}}
simbard_times = {20: [300, 300, 300, 300, 300], 60: {400, 400, 400, 400, 400}, 100: {500, 500, 500, 500, 500}}

def run_test(ssh, test, parallel, particles, input, simbad):
    print "running test...."
    cmd = 'java -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedDistributedSimulator "amqp://10.39.1.28:5672" ' + str(input) + ' results_dir20/' +str(test) + '/' + str(particles) + '_' + str(parallel) + ' ' +str(simbad) + ' false 1000'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/simulator
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
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

def run_serial(ssh, par, file, simbard, name):
    print "running test...."
    cmd = "java -Xmx4G -cp target/simulator-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.sim.FileBasedSimulator false data/" + str(file) + " " + str(par) + " " + str(simbard) + " 1 false > " + str(name) + str(par)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/simulator
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def copy_file(ssh, src, dest):
    print "copy file"
    cmd = 'cp ' + str(src) + ' ' + str(dest)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/slam
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def compile(ssh, dir):
    print "compiling"
    cmd = 'cd ' + str(dir)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    ''' + cmd + '''
    mvn clean install -Dmaven.test.skip=true
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def main():
    #copy_file(sshI, "testscript/ScanMatcher.java", "serial/src/main/java/cgl/iotrobots/slam/core/scanmatcher/ScanMatcher.java")
    #restart_zk(sshNZ)
    #restart_zk(sshNZ)
    run_simbard_test()
    run_aces_test()
    #restart_zk(sshNZ)
    run_rs_test()
    #restart_zk(sshNZ)
    #run_simbard_cost_test()
    copy_file(sshI, "../simulator/testscript/Simbard.java", "serial/src/main/java/cgl/iotrobots/slam/core/app/GFSAlgorithm.java")
    compile(sshI, "/home/ubuntu/projects/iotrobots/slam")
    compile(sshI, "/home/ubuntu/projects/iotrobots/simulator")
    run_serial(sshI, 20, "simbard_1.txt", "true", "simbard_serial__")
    run_serial(sshI, 60, "simbard_1.txt", "true", "simbard_serial__")
    run_serial(sshI, 100, "simbard_1.txt", "true", "simbard_serial__")
    copy_file(sshI, "../simulator/testscript/Aces.java", "serial/src/main/java/cgl/iotrobots/slam/core/app/GFSAlgorithm.java")
    compile(sshI, "/home/ubuntu/projects/iotrobots/slam")
    compile(sshI, "/home/ubuntu/projects/iotrobots/simulator")
    run_serial(sshI, 20, "aces_300.txt", "false", "aces_serial__")
    run_serial(sshI, 60, "aces_300.txt", "false", "aces_serial__")
    run_serial(sshI, 100, "aces_300.txt", "false", "aces_serial__")
    #copy_file(sshI, "../testscript/Simbard.java", "serial/src/main/java/cgl/iotrobots/slam/core/app/GFSAlgorithm.java")
    #copy_file(sshI, "../testscript/ScanMatcher_const.java", "serial/src/main/java/cgl/iotrobots/slam/core/scanmatcher/ScanMatcher.java")
    #compile(sshI, "/home/ubuntu/projects/iotrobots/slam")
    #compile(sshI, "/home/ubuntu/projects/iotrobots/simulator")
    #run_serial(sshI, 20, "simbard_1.txt", "true", "simbard_c_serial__")
    #run_serial(sshI, 60, "simbard_1.txt", "true", "simbard_c_serial__")
    #run_serial(sshI, 100, "simbard_1.txt", "true", "simbard_c_serial__")

if __name__ == "__main__":
    main()
