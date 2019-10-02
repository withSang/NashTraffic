import math
import numpy as np
from collections import defaultdict



car_length = 5 #average car length is 5m 
d_i = car_length + 1 #minimal inter-car distance is 6m
v_min_ratio = 0.4 #minimal velocity is 40% of ideal velocity




def get_velocity(v_0 = 60, lanes=4, cars=100, distance=1000): #effective velocity of straight road based on number of cars
    d_f = v_0 * 0.5
    
    #compute inter-car distance by number of cars
    if cars > 1: d = (distance - (cars/lanes)*car_length) / (cars-1)
    else: d = d_f
        
    #compute effective velocity by inter-car distance
    if d < d_i: v_eff = 0
    elif d<d_f: v_eff = v_0 * math.sin( math.pi/2 * (d-d_i) / (d_f-d_i) )
    else: v_eff = v_0
        
    return (v_0*(v_min_ratio) + v_eff * (1-v_min_ratio), v_eff==0) #(real speed, if the road is full of cars(saturated))


def get_time(v_0 = 60, traffic_lights=0, light_on_sec=30, light_off_sec=60, distance=1000, lanes=4, cars=100): #compute effective time of a road
    v_0 /= 3.6
    velocity_eff, is_saturated = get_velocity(v_0 = v_0, lanes=lanes, cars=cars, distance=distance) #(real speed, if the road is full of cars)
    if light_on_sec+light_off_sec != 0: light_off_ratio = light_off_sec / (light_on_sec + light_off_sec) #ratio of time when traffic light is red
    else: light_off_ratio = 0 #if there is no traffic light, the extended distance is zero
        
    distance_eff = distance + light_off_sec * light_off_ratio/2 * velocity_eff * traffic_lights #compute effective distance caused by traffic lights
    if is_saturated==False: return distance_eff / velocity_eff #if the road is not saturated, the time can be calculated by (distance/velocity)
    else: #if the road is saturated, the extra extended time is proportional with (number of cars - min number of saturated road's cars)
        return distance_eff / velocity_eff + ( cars - (distance+d_i)//(d_i+car_length/lanes) )/lanes/10000*600/velocity_eff



def input_path(N): #N : number of vertices
    adj = np.where(np.eye(N)==1, 0, np.inf).tolist()
    
    while True:
        try:
            a, b = map(int, input().split(' ')) #a->b singular directional graph
            v_0, traffic_lights, light_on_sec, light_off_sec, distance, alpha, beta = map(int,input().split(' '))
            adj[a][b] = lambda x: get_time(v_0=v_0, traffic_lights=traffic_lights, light_on_sec=light_on_sec,\
                                  light_off_sec=light_off_sec, distance=distance, cars=alpha*x+beta)
        except: break
    return adj


def find_available_path(arr): #Find all available path in directional graph with Breadth-First Search
    N = len(arr)
    arr = np.array(arr)
    arr = np.where((arr!=0) & (arr!=np.inf), 1, 0)
    
    class Path:
        def __init__(self, now, prev=[]):
            self.now = now
            self.prev = prev
        
        def path(self):
            return self.prev+[self.now]
                    
    paths = []
    queue = [Path(0)]
    while queue!=[]:
        now = queue.pop(0)
        if now.now==N-1: paths.append(now.path())
        candidate = np.where(arr[now.now]==1)[0]
        candidate = [item for item in candidate if item not in now.prev]
        queue += [Path(item, now.path()) for item in candidate]
    return paths


def time_update(arr, paths, cars): #Calculate each path's time based on each path's number of cars
    paths = [[(path[index],path[index+1]) for index,_ in enumerate(path[:-1])] for path in paths]
    edge_cars = defaultdict(int)
    
    for index, path in enumerate(paths):
        for edge in path:
            edge_cars[edge] += cars[index] #calculate each edge's total cars
    #Transit time is determined by number of cars in each edge of a path
    times = [sum([arr[edge[0]][edge[1]](edge_cars[edge]) for edge in path]) for path in paths]
    return times




def compute_equilibrium(arr, total_cars=200, diff=5): #diff: minimal loss of calculation
    paths = find_available_path(arr)
    n_routes = len(paths)
    cars = [0 for _ in range(n_routes)]
    times = time_update(arr, paths, cars)
    fastest = times.index(min(times))
    cars[fastest] = total_cars
    
    
    while True:
        times = time_update(arr, paths, cars)
        fastest = times.index(min(times))
        slowest = times.index(max(times))
        
        if times[slowest]-times[fastest] <= diff: return cars, times
        
        if cars[fastest]<total_cars and cars[slowest]>0:
            cars[fastest] += 1
            cars[slowest] -= 1
        else: return cars, times

      
        
def print_traffic(cars, times): #when number of cars, times of each path is given, print the condition of each path.
    for route, cartime in enumerate(zip(cars, times)):
        car, time = cartime
        if car==0:
          print("route : {0:2d} // cars : 0".format(route+1))
          continue
        print("route : {0:2d} // cars : {1:3d}, time : {2:3d}m {3:3f}s".format(route+1, car, int(time//60), time%60))
    mean = sum([cars[i]*times[i] for i, _ in enumerate(times)]) / sum(cars)
    print("average time : {0:3d}m {1:3f}s\n".format(int(mean//60), mean%60))

