import socket
import trajectory
from kukarsi import time_keeper
from kukarsi import command
import multiprocessing
import time

def trajectory_joint_correction_command(from_kuka):
    now = time.time()
    joint_desired = trajectory.get_joint_command(now)
    return command.joint_correction_command(from_kuka, joint_desired)

def run_kuka_communication(signals):
    BUFFER_SIZE = 1024
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.1.198", 49001))
    counter = 0
    done = False
    def isset(signal_name):
        res = signals[signal_name].is_set()
        signals[signal_name].clear()
        return res
    
    while not done:
        if isset('stop'):
            trajectory.stop()
        elif isset('up'):
            trajectory.move_up()
        elif isset('down'):
            trajectory.move_down()
        elif isset('quit'):
            done = True
        received_data, socket_of_krc = sock.recvfrom(BUFFER_SIZE)
        counter += 1
        cmd  = trajectory_joint_correction_command(received_data.decode("utf-8"))
        reply = bytes(cmd)
        sock.sendto(reply, socket_of_krc)

if __name__ == '__main__':
    signals = {'up' : multiprocessing.Event(),
               'down' : multiprocessing.Event(),
               'stop' : multiprocessing.Event(),
               'quit' : multiprocessing.Event()}
    kuka_process = multiprocessing.Process(name='kuka',
                                           target=run_kuka_communication,
                                           args=(signals,))
    kuka_process.start()

    while not signals['quit'].is_set():
        print("U: Up")
        print("D: Down")
        print("Q: Quit")
        key = raw_input('Press Any other key and/or Enter to stop:\n')
        if key == 'u' or key == 'U':
            signals['up'].set()
        elif key == 'd' or key == 'D':
            signals['down'].set()
        elif key == 'q' or key == 'Q':
            signals['quit'].set()
        else:
            print("Sending stop signal")
            signals['stop'].set()
        
    kuka_process.join()
