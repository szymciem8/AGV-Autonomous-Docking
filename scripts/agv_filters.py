#!/usr/bin/env python

from statistics import mean


class AGVFilter:
    def __init__(self, window_width, num_sensors):
        self.window_width = window_width
        self.num_sensors = num_sensors
        self.measurements = [[0 for measurement in range(window_width)] for sensor in range(num_sensors)]
        self.results = [-1 for _ in range(self.num_sensors)]

    def update_measurements(self, new_measurements):
        if len(new_measurements) != self.num_sensors:
            raise ValueError(f'Invalid length of new_measurements; expected {self.num_sensors}, got {len(new_measurements)}')

        for i, measurement in enumerate(self.measurements):
            measurement.pop(0)
            measurement.append(new_measurements[i])


class MedianFilter(AGVFilter):
    def __init__(self, window_width=5, num_sensors=4):
        super().__init__(window_width, num_sensors)

    def output(self):
        try:
            for i, sensor_measurements in enumerate(self.measurements):
                if self.window_width % 2 == 1:
                    self.results[i] = sorted(sensor_measurements)[self.window_width // 2]
                else:
                    self.results[i] = sorted(sensor_measurements)[self.window_width // 2]
                    self.results[i] += sorted(sensor_measurements)[(self.window_width // 2) - 1]
                    self.results[i] /= 2

        except IndexError:
            self.results = [-1 for _ in range(self.num_sensors)]

        return self.results


class MovingAverageFilter(AGVFilter):
    def __init__(self, window_width=3, num_sensors=4):
        super().__init__(window_width, num_sensors)

    def output(self):
        for i, sensor_measurements in enumerate(self.measurements):
            self.results[i] = mean(sensor_measurements)

        return self.results