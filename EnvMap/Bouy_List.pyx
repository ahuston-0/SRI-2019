class Buoy:
    def __init__(self):
        self._y = 0
        self._x = 0
        self._color = "none"
        self._hits = 0

    def __init__(self, attribute_arr):
        self._y = attribute_arr[0]
        self._x = attribute_arr[1]
        self._color = attribute_arr[2]

    def __set_attributes(self, attribute_arr):
        self._y = attribute_arr[0]
        self._x = attribute_arr[1]
        self._color = attribute_arr[2]

    def increment_hits(self):
        self._hits = self._hits + 1

    def get_attributes(self):
        return [self._y, self._x, self._color]

    def get_hits(self):
        return self._hits


class BuoyList:
    def __init__(self):
        self._buoy_list = []
        self._confirm_thresh = 8

    def add_buoy(self, buoy):
        if len(self._buoy_list) > 0:
            found = False
            for item in self._buoy_list:
                if buoy == item.get_attributes():
                    item.increment_hits()
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
                arr.append(item)