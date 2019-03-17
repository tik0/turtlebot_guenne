import numpy as np
import scipy
num_samples = 20
offset = -276
scale = 0.05
class bandit_robot:
    #map is the greyscale image of the map
    def __init__(self, map, GREY):
        #num_samples =20
        #GREY = #TODO
        #BLACK = #TODO
        #UNKNOWN = #TODO
        #map = #TODO

        points = np.zeros(map.shape)
        point_coords = np.argwhere(map==GREY)
        points_found, _ = point_coords.shape
        indices = np.random.choice(np.asarray(range(points_found)), num_samples)

        self.waypoint_coords = point_coords[indices]
        self.values = np.ones(num_samples)
        self.current_point = 0
        self.next_point = 0
        self.n = [0 for i in range(num_samples)]

    def sample_action(self):
        distribution = scipy.special.softmax(values)
        action_index = np.random.choice(range(num_sample), p = distribution)
        action = self.waypoint_coords[action_index]
        self.current_point = self.next_point
        self.next_point = action_index
        return (action + offset)*scale

    def got_reward(self, reward):
        self.values[self.current_point] += reward 
        self.values[self.next_point] += reward
