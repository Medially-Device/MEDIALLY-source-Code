import socket
import paramiko
import os
import keyboard  # For detecting key presses
import time


# Server configuration
HOST = "0.0.0.0"  # Listen on all available interfaces
PORT = 12345  # Choose a port for the server

# BeagleBone details
beaglebone_ip = "192.168.10.2"  # Replace with your BeagleBone IP address
# ethernet ip 192.168.10.2
#set ethernet host ip to static 192.168.10.1
username = "debian"  # Your username
password = "MEDIALLY"  # Your password (if required)

def execute_ssh_commands():
    """Connects to the BeagleBone, runs the script, and downloads the video file."""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(beaglebone_ip, username=username, password=password)
        
        # Chain the commands so both run in the same session and run the script in the background
        startup_command = "source CompVision/bin/activate && python3 SocketServer-Client_tf_lite.py &"
        
        # Execute the command
        stdin, stdout, stderr = ssh.exec_command(startup_command)
        
        # Wait for the command to complete and get output
        exit_status = stdout.channel.recv_exit_status()  # Get the exit status
        
        # Check if the command executed successfully
        if exit_status == 0:
            print("SSH command executed successfully!")
        else:
            error_message = stderr.read().decode('utf-8')
            print(f"SSH command failed with exit status {exit_status}. Error: {error_message}")
            return None
        
        ssh.close()
        return True

    except Exception as e:
        print(f"SSH Error: {e}")
        return None

def start_server():
    """Starts the server and waits for client connections."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Server listening on {HOST}:{PORT}")
        #execute_ssh_commands()

        conn, addr = server_socket.accept()
        with conn:
            print(f"Connected by {addr}")
            
            while True:
                # Wait for Space, Shift, or another key press
                print("Press SPACE to start face ID or C to start pill classifier, or any other key to exit...")
                start_time = time.time()  # Record the time before waiting for key press
                
                event = keyboard.read_event()
                
                if event.event_type == keyboard.KEY_DOWN:
                    if event.name == 'space':
                        signal = b"START_FACEID"
                        print("Sending START_M signal to client...")
                    elif event.name == 'c':
                        signal = b"START_MAIN"
                        print("Sending START_S signal to client...")
                    else:
                        print("Exiting script...")
                        break
                    
                    conn.sendall(signal)
                    
                    # Measure the time after the start signal is sent and wait for response
                    response = conn.recv(1024).decode()
                    
                    # Measure the time after receiving the response
                    elapsed_time = time.time() - start_time  # Time difference in seconds
                    
                    print(f"Received from client: {response}")
                    print(f"Time between key press and receiving response: {elapsed_time:.4f} seconds")
                    
                    

if __name__ == "__main__":
    start_server()
