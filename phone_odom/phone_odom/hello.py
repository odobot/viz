import socket
from struct import *
import time

# # Prepare the UDP connection
# UDP_IP = "192.168.42.171"
# print("Receiver IP: ", UDP_IP)
# UDP_PORT = 52000
# print("Port: ", UDP_PORT)
# sock = socket.socket(socket.AF_INET, # Internet
#                      socket.SOCK_DGRAM) # UDP
# sock.bind((UDP_IP, UDP_PORT))

# # Initialize displacement and velocity variables
# x, y, z = 0.0, 0.0, 0.0
# vx, vy, vz = 0.0, 0.0, 0.0
# last_time = time.time()

def main():
    # Prepare the UDP connection
    UDP_IP = "192.168.42.171"
    print("Receiver IP: ", UDP_IP)
    UDP_PORT = 52000
    print("Port: ", UDP_PORT)
    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    # Initialize displacement and velocity variables
    x, y, z = 0.0, 0.0, 0.0
    vx, vy, vz = 0.0, 0.0, 0.0    

    last_time = time.time()

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes

        # Unpack the received data
        ax = unpack_from('!f', data, 0)[0]
        ay = unpack_from('!f', data, 4)[0]
        az = unpack_from('!f', data, 8)[0]
        roll = unpack_from('!f', data, 44)[0]
        pitch = unpack_from('!f', data, 40)[0]
        yaw = unpack_from('!f', data, 36)[0]

        # Calculate the time difference
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Update velocities
        vx += ax * dt
        vy += ay * dt
        vz += az * dt

        # Update displacements
        x += vx * dt
        y += vy * dt
        z += vz * dt

        # Print received message and calculated displacements
        print("received message: ",
            "Acceleration X: %1.4f" % ax,
            "Acceleration Y: %1.4f" % ay,
            "Acceleration Z: %1.4f" % az,
            "Roll: %1.4f" % roll,
            "Pitch: %1.4f" % pitch,
            "Yaw: %1.4f" % yaw,
            "Displacement X: %1.4f" % x,
            "Displacement Y: %1.4f" % y,
            "Displacement Z: %1.4f" % z)


if __name__ == '__main__':
    main()


    