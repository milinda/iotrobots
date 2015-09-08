import paramiko
import time

ipNz = 'cn01'
ipB = 'cn03'
ipI = 'cn10'
resultsdir = ""

storm_dir="/N/u/skamburu/jstorm_cluster/jstorm-original"
project_dir="/N/u/skamburu/projects/iotrobots/collectives"
key_file='/N/u/skamburu/.ssh/id_rsa'

username = 'skamburu'

sshNZ = paramiko.SSHClient()
sshNZ.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshNZ.connect(ipNz, username=('%s' % username), key_filename=

sshBR = paramiko.SSHClient()
sshBR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshBR.connect(ipB, username=('%s' % username), key_filename='~/.ssh/id_rsa')

sshI = paramiko.SSHClient()
sshI.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshI.connect(ipI, username=('%s' % username), key_filename='~/.ssh/id_rsa')

sshIR = paramiko.SSHClient()
sshIR.set_missing_host_key_policy(paramiko.AutoAddPolicy())

sshIR.connect(ipI, username=('%s' % username), key_filename='~/.ssh/id_rsa')

def exec_storm(ssh, parallel):
    print "executing storm commands"
    cmd = './bin/jstorm jar ~/projects/iotrobots/collectives/target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.BroadCastTopology  -name bcast_processor -ds_mode 0 -p ' + str(parallel)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd ''' + storm_dir + '''
    ./bin/jstorm kill bcast_processor 1
    sleep 45
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(45)

def run_test(ssh, test, parallel, t, size, msgs):
    print "running test...."
    cmd = 'java -cp target/collectives-1.0-SNAPSHOT-jar-with-dependencies.jar edu.iu.cs.storm.collectives.app.DataGenerator "amqp://' + str(ipB) +':5672"' + ' ' + str(test) + '/' + str(size) + '_' + str(parallel) + ' ' + str(t) + ' ' + str(size) + ' ' + str(msgs)
    print cmd
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd ''' + cmd + '''
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def run_bcast_test():
    # tasks = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
    tasks = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    data = [100, 1000, 10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000]

    copy_file(sshNZ, "test/test.yaml", "src/main/resources/topology.yaml")
    compile(sshNZ, project_dir)
    for t in tasks:
        #exec_rabbit(sshBR)
        exec_storm(sshNZ, t)
        for d in data:
            run_test(sshIR, 'jstorm_bcast_third', t, 100, d, 500)


def copy_file(ssh, src, dest):
    print "copy file"
    cmd = 'cp ' + str(src) + ' ' + str(dest)
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd ''' + project_dir + '''
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

