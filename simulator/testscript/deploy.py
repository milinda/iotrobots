import paramiko
import time
from scp import SCPClient

supervisors = ['10.23.0.87', '10.23.0.88', '10.23.0.89', '10.23.0.90', '10.23.0.91']
nimbus = '10.23.0.92'
iot = '10.23.0.94'
broker = '10.23.0.93'

key_file = '/home/ubuntu/skamburu-key'

sshNZ = paramiko.SSHClient()
sshNZ.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshNZ.connect(nimbus, username='ubuntu', key_filename=key_file)

sshBR = paramiko.SSHClient()
sshBR.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshBR.connect(broker, username='ubuntu', key_filename=key_file)

sshI = paramiko.SSHClient()
sshI.set_missing_host_key_policy(paramiko.AutoAddPolicy())
sshI.connect(iot, username='ubuntu', key_filename='/home/ubuntu/skamburu-key')

supervisor_connections = {}

supervisorStart = 'sudo service supervisor start'
supervisorStop = 'sudo service supervisor stop'
hostsCP = 'sudo cp /home/ubuntu/hosts /etc/hosts'

def init_connections():
    for s in supervisors:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(s, username='ubuntu', key_filename=key_file)
        supervisor_connections[s] = ssh

def kill_process_cmd(s):
    s = '''pid=`ps ax | grep "''' + s + '''" | awk "{print $1}"`
    sudo kill $pid'''
    return s

def scp_file(ssh, src, dst):
    scp = SCPClient(ssh.get_transport())
    scp.put(src, dst)

def config_nimbus():
    print "compiling"
    ssh = sshNZ

    scp_file(ssh, "hosts", 'hosts')
    scp_file(ssh, "storm.yaml", "storm.yaml")
    kill = kill_process_cmd('supervisor')
    stormCP = 'cp storm.yaml'  + ' /home/ubuntu/deploy/storm/conf/storm.yaml'
    stormRm = 'rm -rf /home/ubuntu/deploy/storm/storm-local'
    zooRm = 'rm -rf /tmp/zookeeper'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    ''' + stormCP + '''
    ''' + stormRm + '''
    ''' + zooRm + '''
    ''' + kill + '''
    ''' + supervisorStop + '''
    sleep 5
    ''' + supervisorStart + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def config_supervisors():
    for s in supervisors:
        print "compiling"
        ssh = supervisor_connections[s]

        scp_file(ssh, "hosts", 'hosts')
        scp_file(ssh, "storm.yaml", "storm.yaml")

        kill = kill_process_cmd('supervisor')
        stormCP = 'cp storm.yaml'  + ' /home/ubuntu/deploy/storm/conf/storm.yaml'
        stormRm = 'rm -rf deploy/storm/storm-local'
        channel = ssh.invoke_shell()
        stdin = channel.makefile('wb')
        stdout = channel.makefile('rb')
        stdin.write('''
        ''' + hostsCP + '''
        ''' + stormCP + '''
        ''' + stormRm + '''
        ''' + kill + '''
        ''' + supervisorStop + '''
        sleep 5
        ''' + supervisorStart + '''
        exit
        ''')
        print stdout.read()
        stdout.close()
        stdin.close()

def config_iot():
    print "compiling"
    ssh = sshI

    scp_file(ssh, "hosts", 'hosts')
    scp_file(ssh, "iot.yaml", "iot.yaml")

    iotCP = 'cp iot.yaml'  + ' /home/ubuntu/iotcloud2-bin-1.0-SNAPSHOT/conf/iot.yaml'
    channel = ssh.invoke_shell()
    stdin = channel.makefile('wb')
    stdout = channel.makefile('rb')
    stdin.write('''
    ''' + iotCP + '''
    exit
    ''')
    print stdout.read()
    stdout.close()
    stdin.close()

def main():
    init_connections()
    config_nimbus()
    config_supervisors()
    config_iot()

if __name__ == "__main__":
    main()
