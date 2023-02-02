# Maze Generator and Navigator
## What/How does it do?
### Project has a few steps in it:
- First it takes maze size as param and generates a maze which has at least one exit. This problem is solved by help of DFS algorithm.
- After maze is generated, a world file is also created which has the maze in it.
- Turtlebot and maze put in the same world and launched.
- Robot has its own grid mapping with the help of 2D LiDAR.
- And move base algorithm applied at last.
## How to run
- Install TurtleBot3. I recommend you this website.


https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
- After installations source and catkin_make.
- Then create maze with any size. size>3 is recommended.
```
$ roslaunch main main.launch size:=5
```
![maze](https://user-images.githubusercontent.com/75525649/216301316-fbf26a1f-3dbf-4e92-9f5c-0c41fae41b86.png)
- Launch robot and maze in the same world.
```
$ roslaunch main turtlebot_in_maze.launch
```
- Run mapping node.
```
$ rosrun main mapping.py
```
- For manuel drive with keyboard.
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
- For autonomous, first activate move base.
```
roslaunch turtlebot3_navigation move_base.launch
```
- Then run this.
```
$ rosrun main exit_maze.py
```

### Note
 There are some file paths in code. You should change them first. Some communication done with help of these files.
