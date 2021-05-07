'''
3-Degree of Freedom, Robotic Arm simulator

This script includes Foward kinematics for the robot arm of the scene. 
Reads a .csv file with three angles(degrees) in each row, corresponding to
the rotarion of each articulation, and make the trasformations to every object 
in the scene.

Instead of making the product of each transformation matrix, this script uses the
equivalent expression for each result, obtained by symbolic calculations. This is 
done in order to optimize the execution time, and to avoid the use of any python 
library outside of the basic ones included in blender (ie. numpy).

Author: ivanferrarigalizia@gmail.com
'''
import bpy
import csv
import bmesh
from math import radians, sin, cos, pi, asin, atan2

# cleaning up, this loop deletes every frame in the scene
for a in bpy.data.actions:
    bpy.data.actions.remove(a)

#read the csv file with each angle (in degrees) for each articulation
filename = '/home/user/Descargas/angulos.csv'
with open(filename) as p:
        angulos = [list(map(float,rec)) for rec in csv.reader(p, delimiter=',')]

# uncomment the following line to set any combination of angles manually        
#angulos=[[0,0,0],[90,0,0],[0,0,45],[0,0,0],[0,45,0],[45,45,0]]


# define objects of the robot, these have to be already present in the scene (check the .blend file)
ob1= bpy.data.objects["cil1"]             # first  limb
ob2= bpy.data.objects["cil2"]             # second limb
ob3= bpy.data.objects["cil3"]             # third limb
frame_num=0


# clean the scence
bpy.ops.object.select_all(action='SELECT')
bpy.data.objects['Camera'].select_set(False)
bpy.data.objects['cil1'].select_set(False)
bpy.data.objects['cil2'].select_set(False)
bpy.data.objects['cil3'].select_set(False)
bpy.ops.object.delete() 


# collection for the points for the tip of the tool position
collection = bpy.data.collections.new("tool_points")
bpy.context.scene.collection.children.link(collection)
bpy.ops.collection.objects_remove_all()



def create_and_locate_sphere( frame_num, x,y,z, diameter=0.2):
    ''' create a sphere point in the position of the tip of the tool
    '''
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
    bmesh.ops.create_uvsphere(bm, u_segments=3, v_segments=2, diameter=diameter)
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
    


def to_euler_(R):
    ''' phi1, th1, psi1, qw, qx, qy, qz  = to_euler_(R)
    input: 3x3 rotation matrix in radians
    output: euler angles yaw, pitch, roll, and quaternion rotation w, x, y, z
    rot_z . rot_y . rot_x'''
    
    if (R[2][0]!=1 and R[2][0]!=-1):
        th1 = -asin( R[2][0])
        psi1 = atan2( R[2][1]/cos(th1), R[2][2]/ cos(th1))
        phi1 = atan2( R[1][0]/cos(th1), R[0][0]/ cos(th1))
    else:
        phi1 = 0
        if( R[2][0]==-1):
            th1 = pi/2
            psi1 =phi1 + atan2(R[0][1], R[0][2])
        else:
            th1  =-pi/2
            psi1 = -phi1 + atan2( - R[0][1], -R[0][2])
    
    yaw, pitch, roll =   phi1, th1, psi1
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return ( phi1, th1, psi1, qw, qx, qy, qz)



print('tip of the tool:')
print('x,  y,  z')
# this loop reads each angle and make corresponding transformations
for th1, th2, th3 in angulos:
    th1, th2, th3 = radians(th1)*2, radians(th2)+pi/2, radians(th3)-pi/2
    
    # articulation 1 ==============================================================
    # rotation matrix
    R=[[cos(th1), -sin(th1), 0], 
       [sin(th1), cos(th1), 0], 
       [0, 0, 1]]
    
    x1 = 0
    x2 = 0
    x3 = 1
    
    thz, thy, thx, qw, qx, qy, qz  = to_euler_(R)
    
    i=[x1,x2,x3]
    bpy.context.scene.frame_set(frame_num)
    ob1.location=i
    ob1.keyframe_insert(data_path="location")
    ob1.rotation_mode = 'QUATERNION'
    ob1.rotation_quaternion=(qw, qx, qy, qz )
    ob1.keyframe_insert(data_path="rotation_quaternion")
    
    
    # articulation 2 ==============================================================
    # rotation matrix
    R=[[cos(th1)*cos(th2), -sin(th1), sin(th2)*cos(th1)], 
       [sin(th1)*cos(th2), cos(th1), sin(th1)*sin(th2)], 
       [-sin(th2), 0, cos(th2)]]    
    
    thx, thy, thz, qw, qx, qy, qz  = to_euler_(R)
    
    x1 = -0.2*sin(th1) + cos(th1)*cos(th2)
    x2 =  sin(th1)*cos(th2) + 0.2*cos(th1)
    x3 =  2 - sin(th2)
    

    bpy.context.scene.frame_set(frame_num)
    ob2.location=[x1,x2,x3]
    ob2.keyframe_insert(data_path="location")
    ob2.rotation_mode = 'QUATERNION'
    ob2.rotation_quaternion=(qw, qx, qy, qz )
    ob2.keyframe_insert(data_path="rotation_quaternion")
    
    
    # articulation 3 ==============================================================
    # rotation matrix
    R=[[-sin(th2)*sin(th3)*cos(th1) + cos(th1)*cos(th2)*cos(th3), 
        -sin(th1), 
        sin(th2)*cos(th1)*cos(th3) + sin(th3)*cos(th1)*cos(th2)], 
       [-sin(th1)*sin(th2)*sin(th3) + sin(th1)*cos(th2)*cos(th3),
        cos(th1), 
        sin(th1)*sin(th2)*cos(th3) + sin(th1)*sin(th3)*cos(th2)], 
       [-sin(th2)*cos(th3) - sin(th3)*cos(th2), 
        0, 
        -sin(th2)*sin(th3) + cos(th2)*cos(th3)]]

          
    thx, thy, thz, qw, qx, qy, qz  = to_euler_(R)
         
    x1 = -0.4*sin(th1) - sin(th2)*sin(th3)*cos(th1) + cos(th1)*cos(th2)*cos(th3) + \
         2*cos(th1)*cos(th2)
    x2 = -sin(th1)*sin(th2)*sin(th3) + sin(th1)*cos(th2)*cos(th3) +                 \
         2*sin(th1)*cos(th2) + 0.4*cos(th1)
    x3 = -sin(th2)*cos(th3) - 2*sin(th2) - sin(th3)*cos(th2) + 2

    i=[x1,x2,x3]
    bpy.context.scene.frame_set(frame_num)
    ob3.location=i
    ob3.keyframe_insert(data_path="location")
    ob3.rotation_mode = 'QUATERNION'
    ob3.rotation_quaternion=(qw, qx, qy, qz )
    ob3.keyframe_insert(data_path="rotation_quaternion")
    
    
    # tip of the tool ==============================================================    
    x1 = -0.4*sin(th1) - 2*sin(th2)*sin(th3)*cos(th1) + 2*cos(th1)*cos(th2)*cos(th3) + 2*cos(th1)*cos(th2)
    x2 = -2*sin(th1)*sin(th2)*sin(th3) + 2*sin(th1)*cos(th2)*cos(th3) + 2*sin(th1)*cos(th2) + 0.4*cos(th1)
    x3 = -2*sin(th2)*cos(th3) - 2*sin(th2) - 2*sin(th3)*cos(th2) + 2

    create_and_locate_sphere( frame_num, x1,x2,x3)
    
    print(x1,x2,x3)
    
    frame_num +=3     


bpy.data.scenes[0].frame_start = 0
bpy.data.scenes[0].frame_end = frame_num 