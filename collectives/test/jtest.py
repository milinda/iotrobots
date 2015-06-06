import paramiko
import time

ipNz = '10.23.0.22'
ipB = '10.23.3.110'
ipI = '10.23.0.22'
resultsdir = ""

sshNZ = paramiko.SSHClient()
sshNZ.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshNZ.connect(ipNz, username='ubuntu', key_filename='/home/ubuntu/.ssh/id_rsa')

sshBR = paramiko.SSHClient()
sshBR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshBR.connect(ipB, username='ubuntu', key_filename='/home/ubuntu/.ssh/id_rsa')

sshI = paramiko.SSHClient()
sshI.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshI.connect(ipI, username='ubuntu', key_filename='/home/ubuntu/.ssh/id_rsa')

sshIR = paramiko.SSHClient()
sshIR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshIR.connect(ipI, username='ubuntu', key_filename='/home/ubuntu/.ssh/id_rsa')

def restart_zk(ssh):
    print "Restart ZK"
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

def exec_storm(ssh, parallel):
    print "executing storm commands"
    cmd = './bin/jstorm jar ~/projects/iotrobots/collectives/target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.BroadCastTopology  -name bcast_processor -ds_mode 0 -p ' + str(parallel)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/deploy/jstorm-0.9.6.3
    ./bin/jstorm kill bcast_processor 1
    sleep 45
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(45)

def exec_rabbit(ssh):
    print "starting rabbitmq...."
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    pid=`ps ax | grep "rabbitmq" | awk '{print $1}'`
    sudo kill $pid
    sleep 5
    cd /home/ubuntu/deploy/rabbitmq_server-3.3.5/sbin
    sudo ./rabbitmq-server > /dev/null 2>&1 &
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(5)

def run_test(ssh, test, parallel, t, size, msgs):
    print "running test...."
    cmd = 'java -cp target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.DataGenerator "amqp://' + str(ipB) +':5672"' + ' ' + str(test) + '/' + str(size) + '_' + str(parallel) + ' ' + str(t) + ' ' + str(size) + ' ' + str(msgs)
    print cmd
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/collectives
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def run_bcast_test():
    tasks = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
    data = [100, 1000, 10000, 100000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000]

    copy_file(sshNZ, "test/test.yaml", "src/main/resources/topology.yaml")
    compile(sshNZ, "/home/ubuntu/projects/iotrobots/collectives")
    for t in tasks:
        exec_rabbit(sshBR)
        exec_storm(sshNZ, t)
        for d in data:
            run_test(sshIR, 'jstorm_bcast_first', t, 100, d, 500)

def start_cluster(par, t):
    exec_rabbit(sshBR)
    exec_storm(sshNZ, par, t)

def copy_file(ssh, src, dest):
    print "copy file"
    cmd = 'cp ' + str(src) + ' ' + str(dest)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd /home/ubuntu/projects/iotrobots/collectives
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
    # restart_zk(sshNZ)
    run_bcast_test()


if __name__ == "__main__":
    main()

