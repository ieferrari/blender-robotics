# blender-robotics
Some 3D Robots simulators using python scripts inside Blender.

Each .blend file has a specific robot configuration with defined articulations.

The script 
* reads a .csv file with angles for each articulation in the robot 
* calculates forward kinematics, 
* makes a simulation inside the blender scene
* outputs a .csv file with the trajectory of the tip of the tool.

You can test your inverse kinematics calculations by creating a .csv file and running the script or just play and explore the different possible trajectories

https://user-images.githubusercontent.com/58748106/117384743-0c649300-aeba-11eb-9e85-6e22043f6d9d.mp4


## Tutorial
### Step 1 Download Blender
Go to the official download page https://www.blender.org/download/ and download the latest version
![1200px-Logo_Blender svg](https://user-images.githubusercontent.com/58748106/117386986-4fc10080-aebe-11eb-8bae-8a93257f607d.png)

### Step 2 Download this repository
Select a folder with robotic configuration, and open the .blend file with Blender

### Step 3 Set the path to .csv file and run the script
Go to script view, select "animar" file and set the path to the .csv file  then clic on "run script".
Each folder includes some demo .csv files with angles configurations for some trajectories. 
You can test your inverse kinematics calculations by creating a .csv file and running the script.

### Step 4 Explore the simulation
For a given .csv file with the angles configuration for each articulation at each step, the script will calculate the forward kinematics and make the animation. You can render the animation with Blender or just explore the robot behavior inside the viewport. Also check the ouput.csv file with the tip of the tool trajetory.


## Motivation & Disclaimer
This series of scripts were made primarily for educational purposes as a lightweight simulator for an introductory robotics lesson.


Also everything inside this repository is free software and it comes without any warranty.
