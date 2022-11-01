class Colors:
    def __init__(self):
        self.red_color = [255, 0, 0]
        self.green_color = [0, 255, 0]
        self.blue_color = [0, 0, 255]
        self.black_color = [0, 0, 0]
        self.yellow_color = [255, 255, 0]

        self.if_red_value = 20
        self.if_green_value = 30
        self.if_blue_value = 40
        self.if_black_value = 50
        self.if_yellow_value = 60

        self.COLORS_ARRAY = [(self.red_color, self.if_red_value),
                             (self.green_color, self.if_green_value),
                             (self.blue_color, self.if_blue_value),
                             (self.black_color, self.if_black_value),
                             (self.yellow_color, self.if_yellow_value)]

    def getColors(self):
        return self.COLORS_ARRAY
