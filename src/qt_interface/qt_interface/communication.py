INITIAL_WAYPOINT_CONFIG = 'initial-waypoint-config'
UPDATE_CAR_POSITION = 'update-car-position'

class Message:
    def __init__(self, msg_type, msg):
        assert(type(msg_type) == str)

        self.msg_type = msg_type
        self.msg = msg