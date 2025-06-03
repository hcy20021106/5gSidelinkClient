# udp_receiver.py

import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 9999)) # 与 C 中 remote_addr 保持一致
print("Listening on UDP port 9999...")




while True:
    data, addr = sock.recvfrom(2048)
    print(f"Received from {addr}: {data.decode(errors='ignore')}")



