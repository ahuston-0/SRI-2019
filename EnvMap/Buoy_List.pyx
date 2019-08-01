class Buoy:
    def __init__(self):
        self._y = 0
        self._x = 0
        self._color = "none"
        self._green_hits = 0
        self._red_hits = 0

    def __init__(self, attribute_arr):
        self._y = attribute_arr[0]
        self._x = attribute_arr[1]
        self._color = "none"
        self._green_hits = attribute_arr[2].count("green")
        self._red_hits = attribute_arr[2].count("red")

    def __set_attributes(self, attribute_arr):
        self._y = attribute_arr[0]
        self._x = attribute_arr[1]

    def increment_hits(self, color):
        if color == "green":
            self._green_hits = self._green_hits + 1
        elif color == "red":
            self._red_hits = self._red_hits + 1

    def get_attributes(self):
        if self._red_hits > self._green_hits:
            self._color = "red"
        elif self._green_hits > self._red_hits:
            self._color = "green"
        else:
            self._color = "none"
        return [self._y, self._x, self._color]

    def get_hits(self):
        if self._red_hits > self._green_hits:
            return self._red_hits
        elif self._green_hits > self._red_hits:
            return self._green_hits
        return 0


class BuoyList:
    def __init__(self):
        self._buoy_list = []
        self._confirm_thresh = 8

    def add_buoy(self, buoy):
        if len(self._buoy_list) > 0:
            found = False
            for item in self._buoy_list:
                item_attr = item.get_attributes()
                if buoy[0] == item_attr[0] and buoy[1] == item_attr[1]:
                    item.increment_hits(buoy[2])
                    found = True
            if not found:
                self._buoy_list.append(Buoy(buoy))
        else:
            self._buoy_list.append(Buoy(buoy))

    def get_buoys(self):
        return self._buoy_list

    def set_confirmation_threshold(self, thresh):
        self._confirm_thresh = thresh

    def get_confirmation_threshold(self):
        return self._confirm_thresh

    def get_confirmed_buoys(self):
        arr = []
        for item in self._buoy_list:
            if item.get_hits() > self._confirm_thresh:
                arr.append(item.get_attributes())
        return arr