#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import math
import numpy as np
import open3d as o3d

s = serial.Serial('COM3', 115200, timeout = 10)
f = open("tof_radar.xyz", "w")                          
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()
lines = []
# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())

# recieve characters from UART of MCU
flag = 1
resolution = 32 #32 64 
counter = 0
frames = 3;
z = 0
while flag == 1:
    
    read1 = s.readline()
    data = read1.decode()                    #String that holds our data
    datarr = data.split(" ")
    if(datarr[0]=="Measure"):
        angle = math.radians(float(datarr[2]))
        distance = int(datarr[1])
        x = round(math.cos(angle) * distance,4)
        y = round(math.sin(angle) * distance,4)
        x = str(x)
        y = str(y)
        zs = str(z)
        f.write(x)
        f.write(" ")
        f.write(y)
        f.write(" ")
        f.write(zs)
        f.write("\n")
        counter = counter+1
        if(counter>=resolution):
            lines.append([counter-resolution,counter])
        if(counter>= 1):
            lines.append([counter-1, counter])
        if(counter%resolution==0 and counter!=0):
            z = z+100
        if(counter == resolution*frames):
            flag =0

            
        # create an empty list to hold the coordinates of the point
        #point = []
        #point.append(float(x))
        #point.append(float(y))
      #  point.append(0)
        # append the point to the lines array
        #lines.append(point)
	
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"
#Read the test data in from the file we created      
f.close()  
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])


line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
#close the port
print("Closing: " + s.name)
s.close()
