import pika
import json
import threading
from threading  import Thread
import time
import Queue
import sys

def send_frames():
    # parameters = pika.URLParameters('amqp://149.165.159.3:5672')
    # connection = pika.BlockingConnection(pika.ConnectionParameters(host='149.165.159.3', port=5672))
    # connection = pika.BlockingConnection(parameters)
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    channel = connection.channel()
    channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)
    # try:
    send_count = 0
    for x in range(0, 1596):
        data = file_read(x)
        channel.basic_publish(exchange='turtle_kinect',
                              routing_key='turtle_frames',
                              body =data,
                              properties=pika.BasicProperties(delivery_mode = 2, headers={"time": str(int(round(time.time() * 1000)))}))
        time.sleep(.03)
        send_count += 1
        print "send_count: " + str(send_count)

def send_nav_data():
    # parameters = pika.URLParameters('amqp://149.165.159.3:5672')
    # connection = pika.BlockingConnection(pika.ConnectionParameters(host='149.165.159.3', port=5672))
    # connection = pika.BlockingConnection(parameters)
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    channel = connection.channel()
    channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)
    for x in range(0, 1596):
        channel.basic_publish(exchange='drone',
                              routing_key='drone_nav_data',
                              body = 'hello',
                              properties=pika.BasicProperties(delivery_mode = 2, headers={"time": str(int(round(time.time() * 1000)))}))
        time.sleep(.03)
        print "send_count: " + str(send_count)
        # except:
        #     e = sys.exc_info()[0]
        #     raise e

def recv_commands():
    # parameters = pika.URLParameters('amqp://149.165.159.3:5672/')
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    # connection = pika.BlockingConnection(parameters)
    # connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    channel = connection.channel()
    channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)

    channel.basic_consume(on_message, queue='control')
    channel.start_consuming()

average = 0.0
count = 0
run = 0
recv_count = 0
def on_message(channel, method_frame, header_frame, body):
    global average
    global count
    global run
    global recv_count

    channel.basic_ack(delivery_tag=method_frame.delivery_tag)
    latency = int(round(time.time() * 1000)) - long(header_frame.headers["time"])
    if latency < 150:
        count += 1
        average = average + (latency - average) / count
    #print header_frame.headers["time"]
    recv_count += 1
    print "recv_count: " + str(recv_count) + " latency: ", str(latency), " average: ", str(average)
    d = json.loads(body)
    print body


def file_read(i):
    f = open('/home/supun/dev/projects/dist/data/kinect/kinect' + str(i + 1), 'r')
    data = f.read()
    f.close()
    return data

def main():
    t = Thread(target=send_frames)
    t.daemon = True
    t.start()

    # t = Thread(target=send_nav_data)
    # t.daemon = True
    # t.start()

    # t = Thread(target=recv_commands)
    # t.daemon = True
    # t.start()

    t.join()

def consume():
    global queue
    while True:
        queue.put("Hello")

def produce():
    global queue
    while True:
        print queue.get()

def test():
    global queue

    queue = Queue.Queue()
    t = Thread(target=consume)
    t.daemon = True
    t.start()

    t = Thread(target=produce)
    t.daemon = True
    t.start()

    t.join()

if __name__ == "__main__":
    main()
    # try:
    #     test()
    # except:
    #     print "hello"
