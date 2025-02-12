import socket
import threading
import time
import random
import sys, os
import signal

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))

# DDBoat 17 (Server)
def handle_client(conn, addr, gps):
    print("Connection established with {}".format(addr))

    rmc_ok = False
    while not rmc_ok:
        rmc_ok, rmc_data = gps.read_rmc_non_blocking()
        if rmc_ok:
            gps_position = "%f;%s;%f;%s\0" % (rmc_data[0], rmc_data[1], rmc_data[2], rmc_data[3])
            break
        time.sleep(0.1)
    
    try:
        while True:
            rmc_ok = False
            while not rmc_ok:
                rmc_ok, rmc_data = gps.read_rmc_non_blocking()
                print ("RMC Data",rmc_data,rmc_ok)
                if rmc_ok:
                    gps_position = "%f;%s;%f;%s\0" % (rmc_data[0], rmc_data[1], rmc_data[2], rmc_data[3])
                    break
                time.sleep(0.1)            
            try:
                conn.sendall(gps_position.encode())
                print("Sent GPS Position to {}: {}".format(addr, gps_position))
            except (socket.error, BrokenPipeError):
                print("Client {} disconnected unexpectedly.".format(addr))
                break  # Exit loop when client disconnects
            time.sleep(0.01)  # save computations
    except Exception as e:
        print("Server error with {}: {}".format(addr, e))
    finally:
        conn.close()

def robot1_server():
    import gps_driver_v2 as gps_drv
    gps = gps_drv.GpsIO(tty_dev=1)
    gps.set_filter_speed("0")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 5000))  # Listen on all available interfaces
    server_socket.listen(15)  # Allow up to 15 clients
    print("DDBoat 5 (Server) is waiting for connections...")
    
    while True:
        conn, addr = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(conn, addr, gps))
        client_thread.start()



if __name__ == "__main__":
    robot1_server()
