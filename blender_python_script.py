import bpy
import socket
import pickle
import numpy as np
from mathutils import Vector

def create_Vertices (name, verts):
    # Create mesh and object
    me = bpy.data.meshes.new(name+'Mesh')
    ob = bpy.data.objects.new(name, me)
    ob.show_name = False
    # Link object to scene
    bpy.context.scene.objects.link(ob)
    me.from_pydata(verts, [], [])
    # Update mesh with new data
    me.update()
    return ob



HOST, PORT_SND, PORT_RCV =  "localhost", 9999, 10000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT_SND))
print("waiting on port:", PORT_SND)



data = pickle.dumps((1))
s.sendto(data, (HOST, PORT_RCV))
coord_tot = np.zeros([10000,3])
n = 0
points = []
while True:
    try:
        coord = s.recvfrom(1024)
        coord = pickle.loads(coord[0])
        if n == 0:
            coord_old = coord
        print(coord)
        n += 1
        #if abs(coord[0]-coord_old[0])>6:
        #    coord[0] = coord_old[0]
        #if abs(coord[1]-coord_old[1])>6:
        #    coord[1] = coord_old[1]
        #if abs(coord[2]-coord_old[2])>6:
        #    coord[2] = coord_old[2]
        coord_tot[n,...] = coord
        points.append(Vector((coord[0],coord[1],coord[2],1.0)))
        data = pickle.dumps((1))
        s.sendto(data, (HOST, PORT_RCV))
        #coord_old = coord
    except:
        print("error")
        s.close()
        break
         
#create_Vertices ("scan", coord_tot)


surface_data = bpy.data.curves.new('wook', 'SURFACE')
surface_data.dimensions = '3D'

# set points per segments (gzU * V)
total_pts = 3321
lines = 81
for i in range(0, total_pts, lines):
    spline = surface_data.splines.new(type='NURBS')
    spline.points.add(lines-1)  # already has a default vector

    for p, new_co in zip(spline.points, points[i:i+lines]):
        p.co = new_co

surface_object = bpy.data.objects.new('NURBS_OBJ', surface_data)
scene = bpy.context.scene
scene.objects.link(surface_object)

splines = surface_object.data.splines
for s in splines:
    for p in s.points:
        p.select = True

bpy.context.scene.objects.active = surface_object
bpy.ops.object.mode_set(mode = 'EDIT') 
bpy.ops.curve.make_segment()
