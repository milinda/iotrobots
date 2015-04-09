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

def exec_storm(ssh, particles, parallel):
    cmd = './bin/storm jar ~/projects/iotrobots/slam/streaming/target/iotrobots-slam-streaming-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.streaming.SLAMTopology -name slam_processor -ds_mode 0 -p ' + str(parallel) + ' -pt ' + str(particles) + ' -i'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    cd deploy/storm
    ./bin/storm kill slam_processor -w 1
    sleep 10
    ''' + cmd + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()
    time.sleep(30)

def exec_rabbit(ssh):
    stdin, stdout, stderr = ssh.exec_command('cd /home/ubuntu/deploy/rabbitmq_server-3.3.2/sbin; sudo ./rabbitmq-server')

def exec_iotcloud(ssh):
    stdin, stdout, stderr = ssh.exec_command('cd deploy/iotcloud2-bin-1.0-SNAPSHOT; ./bin/iotcloud local')

def exec_sensor(ssh):
    stdin, stdout, stderr = ssh.exec_command('cd deploy/iotcloud2-bin-1.0-SNAPSHOT; ./bin/iotcloud jar repository/sensors/iotrobots-slam-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar cgl.iotrobots.slam.sensor.SlamSensor -s local -sim -url "amqp://10.39.1.28:5672"')

def main():
    exec_storm(sshNZ, 20, 4)

if __name__ == "__main__":
    main()