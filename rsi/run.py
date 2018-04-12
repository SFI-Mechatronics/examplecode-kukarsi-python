import socket
import numpy
import command
import time_keeper

tiden = time_keeper.time_keeper()

def simple_joint_correction_command(from_kuka):
    A1 = 5.0 * numpy.sin(tiden.get() / 10.0)
    joint_desired = [A1, 0.0, 0.0, 0.0, 0.0, 0.0]
    return command.joint_correction_command(from_kuka, joint_desired)

def run():
    BUFFER_SIZE = 1024
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.1.198", 49001))
    counter = 0
    while True:
        received_data, socket_of_krc = sock.recvfrom(BUFFER_SIZE)  # buffer size is 1024 bytes
        print("counter: ", counter, " received_data:")
        print(received_data)
        counter += 1
#        reply = bytes(get_reply(received_data.decode("utf-8")), 'utf-8')
        kuka_command = simple_joint_correction_command(received_data.decode("utf-8"))
#        print("kuka_command: ")
#        print(kuka_command)
        reply = bytes(kuka_command)#, 'utf-8')
        sock.sendto(reply, socket_of_krc)
