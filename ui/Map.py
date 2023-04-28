class Map:
    def __init__(self, x, y, angle):
        self.real_width = 3000  # in millimeters
        self.real_height = 2000  # in millimeters
        self.sim_width = 1024  # in pixels
        self.sim_height = 600  # in pixels
        self.x = x
        self.y = y
        self.angle = angle

    def goto(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def getSimulationPos(self):
        pixel_x = int(self.x / self.real_width * self.sim_width)
        pixel_y = int(self.y / self.real_height * self.sim_height)
        return pixel_x, pixel_y

