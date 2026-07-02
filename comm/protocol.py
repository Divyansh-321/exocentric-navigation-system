class RobotProtocol:
    @staticmethod
    def serialize(action_string):
        """Converts raw string movement actions into a standard ASCII byte array format."""
        return action_string.encode('utf-8')

    @staticmethod
    def deserialize(incoming_bytes):
        """Decodes telemetry data coming back from the microcontroller."""
        return incoming_bytes.decode('utf-8').strip()