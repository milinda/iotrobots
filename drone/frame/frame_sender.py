import pika
import json
import threading
from threading  import Thread
import time
import Queue

def send_frames():
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    channel = connection.channel()
    channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)

    for x in range(0, 1596):
        data = file_read(x)
        channel.basic_publish(exchange='drone',
                                   routing_key='drone_frame',
                                   body =data,
                                   properties=pika.BasicProperties(delivery_mode = 2, headers={"time": str(int(round(time.time() * 1000)))}))
        time.sleep(.03)


def recv_commands():
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
    channel = connection.channel()
    channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)

    average = 0.0
    count = 0
    run = 0
    while run == 0:
        method_frame, header_frame, body = channel.basic_get('control')
        if method_frame:

            channel.basic_ack(method_frame.delivery_tag)

            latency = int(round(time.time() * 1000)) - long(header_frame.headers["time"])
            if latency < 150:
                count += 1
                average = average + (latency - average) / count
            #print header_frame.headers["time"]
            print "latency ", str(latency), " average: ", str(average)
            d = json.loads(body)
            print body

def file_read(i):
    f = open('frames/output/' + str(i), 'r')
    data = f.read()
    f.close()
    return data

def main():
    t = Thread(target=send_frames)
    t.daemon = True # thread dies with the program
    #self.f = open('test.out', 'w')
    t.start()

    t = Thread(target=recv_commands)
    t.daemon = True # thread dies with the program
    #self.f = open('test.out', 'w')
    t.start()

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
