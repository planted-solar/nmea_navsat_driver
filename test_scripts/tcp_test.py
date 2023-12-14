import socket
import time

def main():
    server_ip = '127.0.0.2'  # Server IP address
    server_port = 9001  # Server port number

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(5)
    print(f"Server is listening on {server_ip}:{server_port}")

    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Connection established with {client_address}")

        nmea_sentences = [
        "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F\n",
        "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F\n",
        ]

        try:
            while True:
                for sentence in nmea_sentences:
                    client_socket.send(sentence.encode())
                    time.sleep(1)  # Sending sentences with a delay of 1 second

        except socket.error as exc:
            print(f"Socket error: {exc}")
            client_socket.close()

    server_socket.close()

if __name__ == "__main__":
    main()
