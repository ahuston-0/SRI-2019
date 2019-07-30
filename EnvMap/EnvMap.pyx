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
        for i in range(len(self._curr_map)):
            if self._curr_map[i]
            for j in range(len(self._curr_map[i])):
                if self._curr_map[i][j] == 30:
                    self._buoy_data.append([i - self._offset[0], j - self._offset[1], "green"])
                elif self._curr_map[i][j] == 60:
                    self._buoy_data.append([i - self._offset[0], j - self._offset[1], "red"])

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
