'''
2-Degree of Freedom, 2D Robotic Arm simulator.

Simulator for a simple robotic arm with 2 limbs in this spescific 2D configuration.
Takes as input, the angles for the servomotors in a .csv file
To Run the simulation:
    >> clic on: Run Script 
    >> Time Line >> Play Animation (space bar)
    >> optional: change the .csv file (line22), or set angles manually (line26)
Author: ieferrari
'''

import bpy
import bmesh
from math import radians, sin, cos, pi
import csv

for a in bpy.data.actions:
    bpy.data.actions.remove(a)

# In Windoes use a full-path to the file, example: "C:\\Users\\user\\Desktop\\angulos.csv"
filename = 'cruz_brazo01.csv'          # change the .csv file to simulate another trajectory
with open(filename) as p:
        angulos = [list(map(int,rec)) for rec in csv.reader(p, delimiter=',')]

#angulos = [ [0, 0], [15, 15], [15, 30], [30, 30]]  # you can test angles manually here


ob1 = bpy.data.objects["cil1"]
ob2 = bpy.data.objects["cil2"]


# clean the scence: deletes everything but, camera, cil1 and cil2
bpy.ops.object.select_all(action='SELECT')
bpy.data.objects['Camera'].select_set(False)
bpy.data.objects['cil1'].select_set(False)
bpy.data.objects['cil2'].select_set(False)
bpy.ops.object.delete() 


# collection with the points/spheres for the tip of the tool position
collection = bpy.data.collections.new("tool_points")
bpy.context.scene.collection.children.link(collection)
bpy.ops.collection.objects_remove_all()


def create_and_locate_sphere( frame_num, x,y,z, diameter=1):
    ''' creates a sphere point in the position of the tip of the tool'''
    name='shpere'+str(frame_num)
    mesh = bpy.data.meshes.new("Basic_Sphere")
    basic_sphere = bpy.data.objects.new(name, mesh)

    # Add the object into the scene.
    bpy.context.collection.objects.link(basic_sphere)
    bpy.data.collections['tool_points'].objects.link(basic_sphere)

    # Select the newly created object
    bpy.context.view_layer.objects.active = basic_sphere
    basic_sphere.select_set(True)

    # Construct the bmesh sphere and assign it to the blender mesh.
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm,
                              u_segments=3,
                              v_segments=2,
                              diameter=diameter)
    bm.to_mesh(mesh)
    bm.free()

    bpy.ops.object.modifier_add(type='SUBSURF')
    bpy.ops.object.shade_smooth()
    
    bpy.context.scene.frame_set(frame_num-1)
    basic_sphere.location=[0,0,0]
    basic_sphere.keyframe_insert(data_path="location")
    
    bpy.context.scene.frame_set(frame_num)
    basic_sphere.location=[x,y,z]
    basic_sphere.keyframe_insert(data_path="location")


frame_num=0
for rho1,rho2 in angulos:
    rho1, rho2 = radians(rho1), radians(rho2)

    # center of the first articulation 
    th1 = rho1
    x1 = 5* cos(th1)
    x2 = 5* sin(th1)
    
    i = [x1,x2,0]
    bpy.context.scene.frame_set(frame_num)
    ob1.location=i
    ob1.keyframe_insert(data_path="location")#, index=1)
    ob1.rotation_euler=(radians(90),radians(0),pi/2+th1)
    ob1.keyframe_insert(data_path="rotation_euler")#, index=1)
    
    # center of the second articulation 
    th2 = rho1+ rho2
    x1 = 10 * cos(th1) + 5 * cos(th2)
    x2 = 10 * sin(th1) + 5 * sin(th2)
    
    i = [x1,x2,0]
    bpy.context.scene.frame_set(frame_num)
    ob2.location=i
    ob2.keyframe_insert(data_path="location")#, index=1)
    ob2.rotation_euler=(radians(90),radians(0),pi/2+th2)
    ob2.keyframe_insert(data_path="rotation_euler")#, index=1)
    
    # tip of the tool
    x1 = 10 * cos(th1) + 10 * cos(th2)
    x2 = 10 * sin(th1) + 10 * sin(th2)
    create_and_locate_sphere(frame_num, x1, x2, 0, diameter=1)
    
    # next frame in the scene 
    frame_num +=2


# update begin and end of the simulation in the scene 
bpy.data.scenes[0].frame_start = 0
bpy.data.scenes[0].frame_end = frame_num 
