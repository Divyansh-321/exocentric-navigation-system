import socket

class UDPSender:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1) 

    def send(self, payload):
        """
        Takes byte-encoded payloads and sends them to the target IP.
        Fails silently to keep the main computer vision loop running at high FPS.
        """
        try:
            self.sock.sendto(payload, (self.ip, self.port))
        except Exception as e:
            pass

    def close(self):
        """Cleans up the socket connection on shutdown."""
        self.sock.close()