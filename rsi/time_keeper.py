import time

class time_keeper:
    def __init__(self):
        self.initial_timestamp = None

    def get(self):
        if self.initial_timestamp is None:
            self.initial_timestamp = time.time()
        return time.time() - self.initial_timestamp
