#!/usr/bin/python3
import maze_generate
import check_for_path
import rospy
from time import sleep


path_to_demo = "/home/alperenlcr/ros_workspaces/maze_ws/src/main/world/demofile.world"
path_to_maze = "/home/alperenlcr/ros_workspaces/maze_ws/src/main/world/maze.world"
path_to_size_txt = "/home/alperenlcr/Code/ros_workspaces/maze_ws/src/main/scripts/size.txt"

def mainnn(koordinatlar, size):
    #size = int(input('Enter a maze size: '))
    matrix = maze_generate.mainn(size, koordinatlar)
    goal = [len(matrix), len(matrix[0])]
    
    if check_for_path.solveMaze(matrix, goal) == False:
        mainnn()
    else :
#        rospy.loginfo("\n{}x{} size maze generated :\n".format(size, size))
        print("\n{}x{} size maze generated :\n".format(size, size))
        for i in matrix:
            print(*i)
#             rospy.loginfo(*i)

rospy.init_node('maze_main')
size = rospy.get_param("size")
size = int(size)
koordinatlar = []
mainnn(koordinatlar, size)

fpr = open(path_to_demo, "r")
fpw = open(path_to_maze, "w")
satir = "t"
count = 0

# start and end
#koordinatlar.remove([1,0])
#temp = koordinatlar.pop()
#koordinatlar.pop()
#koordinatlar.append(temp)
koordinatlar[-2][0] += 1

while satir != '':
    satir = fpr.readline()
    fpw.write(satir)
    if satir == '<!--@-->\n' and count == 0:
        count2 = 0
        for i in koordinatlar:
            ek = """
      <model name='unit_box_{}'>
        <pose>{} {} 0.5 0 -1e-05 -0.000818</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>{} {} 0.5 0 -1e-05 -0.000818</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.01062 -0.006191 -9.78231 0.012425 -0.021236 -1.9e-05</acceleration>
          <wrench>-0.01062 -0.006191 -9.78231 0 -0 0</wrench>
        </link>
      </model>
        """.format(count2, (i[1]-1)*0.5, (i[0]-1)*0.5, (i[1]-1)*0.5, (i[0]-1)*0.5)
            count2 += 1
            fpw.write(ek)
        count = 1
    elif satir == '<!--@-->\n':
        count2 = 0
        for i in koordinatlar:
            ek = """
    <model name='unit_box_{}'>
      <pose>{} {} 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

        """.format(count2, (i[1]-1)*0.5, (i[0]-1)*0.5)
            count2 += 1
            fpw.write(ek)

with open(path_to_size_txt, 'w') as f:
    f.write(str(size))

#sleep(2)
