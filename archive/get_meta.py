import socket

def get_meta_data(host, port):
    """
    Function to listen for a single connection from the Meta headset,
    receive data, process it, and return the relevant information.

    Args:
    - host (str): The host IP address or hostname to listen on.
    - port (int): The port number to listen on.

    Returns:
    - tuple: A tuple containing the Meta headset data (x, y, z).
    """
    meta_data = None

    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Listening for Meta headset on {host}:{port}")

    try:
        # Accept connection
        client_socket, client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")

        # Receive and process data
        data = client_socket.recv(1024).decode('utf-8')
        start = data.find('(')
        end = data.find(')')
        if start != -1 and end != -1:
            coordinates_str = data[start + 1:end]
            coordinates = [float(coord) for coord in coordinates_str.split(',')]
            # Process the coordinates as needed
            meta_data = tuple(coordinates)
            print(f"Received Meta headset data: X={meta_data[0]}, Y={meta_data[1]}, Z={meta_data[2]}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close sockets
        client_socket.close()
        server_socket.close()

    return meta_data

if __name__ == '__main__':
    # Define host and port
    host = '0.0.0.0'  # Listen on all available interfaces
    port = 8888

    # Call function to get Meta headset data
    meta_data = get_meta_data(host, port)
    print("Meta headset data:", meta_data)
