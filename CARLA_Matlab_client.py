import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla 
import math 
import random 
import time 
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm
import socket

#Conect the server MATLAB
sever_port = ('192.168.1.141', 3000)

try:
    # create an AF_INET, STREAM socket (TCP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

except:
    print('Failed to create a socket.')
    sys.exit()
print('Socket Created :)')

sock.connect(sever_port)
print("start a client")
time.sleep(0.01)

#s = "start"
#s = bytes(s,encoding='utf8')
#sock.send(s)
#print(s)

# Connect the client and set up bp library and spawn point
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points() 

# Add vehicle
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[3])

# Move spectator to view ego vehicle
spectator = world.get_spectator() 
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
spectator.set_transform(transform)


# Camera callback
#def camera_callback(image, data_dict):
#    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) 

# RADAR callbacks
def radar_callback(data, point_list):
    radar_data = np.zeros((len(data), 4))

    for i, detection in enumerate(data):
        x = detection.depth * math.cos(detection.altitude) * math.cos(detection.azimuth)
        y = detection.depth * math.cos(detection.altitude) * math.sin(detection.azimuth)
        z = detection.depth * math.sin(detection.altitude)
        
        radar_data[i, :] = [x, y, z, detection.velocity]

    sendDataMatlab(radar_data,'radar')     


    points = radar_data[:, :-1]
    points[:, :1] = -points[:, :1]
    #print(points)
# Spawn camera
#camera_bp = bp_lib.find('sensor.camera.rgb') 
#camera_init_trans = carla.Transform(carla.Location(z=2.5, x=-3), carla.Rotation())
#camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Set up dictionary for camera data
#image_w = camera_bp.get_attribute("image_size_x").as_int()
#image_h = camera_bp.get_attribute("image_size_y").as_int()
#camera_data = {'image': np.zeros((image_h, image_w, 4))} 

#RADAR, parameters are to assisst visualisation
radar_bp = bp_lib.find('sensor.other.radar')
radar_bp.set_attribute('range', '20') 
radar_bp.set_attribute('horizontal_fov', '20.0')
radar_bp.set_attribute('vertical_fov', '8.0')
radar_bp.set_attribute('points_per_second', '10000')
radar_init_trans = carla.Transform(carla.Location(z=0.5))
radar = world.spawn_actor(radar_bp, radar_init_trans, attach_to=vehicle)

# Add auxilliary data structures
point_list = o3d.geometry.PointCloud()
radar_list = o3d.geometry.PointCloud()

# Start sensors
radar.listen(lambda data: radar_callback(data, radar_list))
#camera.listen(lambda image: camera_callback(image, camera_data))

#Send radar data to MATLAB
def sendDataMatlab(data,Identificador):
    datos = np.array(data, float)
    s = Identificador + str(datos)
    send = bytes(s,encoding='utf8')
    #n = "new data"
    #n1= bytes(n,encoding='utf8')
    sock.send(send)
    #print(send)

# OpenCV window for camera
#cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
#cv2.imshow('RGB Camera', camera_data['image'])
#cv2.waitKey(1)

while True:  
    #cv2.imshow('RGB Camera', camera_data['image'])

    raw_data=sock.recv(2048).decode('utf-8')
    lst_str = str(raw_data)[1:-1] 
    array = (lst_str.split (" "))
    datos= list(map(float,array))
    #print (datos)
    acelerador =  datos [0]
    #print (acelerador)
    direccion =  datos [1]
    #print (direccion)
    vehicle.apply_control(carla.VehicleControl(throttle = acelerador, steer = direccion))

    # Break if user presses 'q'
    if cv2.waitKey(1) == ord('q'):
        break

# Close displayws and stop sensors
radar.stop()
radar.destroy()
sock.close()

for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for actor in world.get_actors().filter('*sensor*'):
    actor.destroy()