import numpy as np
class EnvMap:
    def __init__(self, image, odom_coords):
        """
        Initializes the class variables and finds any buoys in the given map image

        :param image: A 2D list containing the current data in the map
        :param odom_coords: The position of the odometry reference on the map image
        """
        self._offset = odom_coords
        self._curr_map = image
        self._buoy_data = []
        self.__log_buoys()

    def __log_buoys(self):
        """
        Logs the current position and color of any buoys in the map

        :param image: A 2D list containing the current data in the map
        :return: None
        """
        if np.isin(self._curr_map, [30,50]).any():
            for i, row in enumerate(self._curr_map):
                if np.isin(row, [30,50]).any():
                    for j, pixel in enumerate(row):
                        if pixel == 30:
                            y = i - self._offset[0]
                            x = j - self._offset[1]
                            self._buoy_data.append([y, x, "green"])
                        elif pixel == 50:
                            y = i - self._offset[0]
                            x = j - self._offset[1]
                            self._buoy_data.count([y,x,"red"])
                            self._buoy_data.append([y, x, "red"])

    def get_map(self):
        """
        Returns the map stored in this instance of the class

        :return: A 2D list containing the current data in the map
        """
        return self._curr_map

    def get_buoys(self):
        """
        Returns the set of all buoys in the form of (y, x, color)

        :return: A list of lists, where each sublist is the coordinates and color of a buoy
        """
        return self._buoy_data

    def set_buoys(self, buoy_list):
        self._buoy_data = buoy_list
        for item in buoy_list:
            attr = item.get_attributes()
            self._curr_map[attr[0]][attr[1]] = 30 if attr[2] == "green" else 50 if "red" else self._curr_map[attr[0]][attr[1]]
