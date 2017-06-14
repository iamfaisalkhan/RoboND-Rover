## Project: Search and Sample Return

[//]: # (Image References)

[image1]: ./resources/original.jpg
[image2]: ./resources/perspective_transform.jpg
[image3]: ./resources/rock_detection.jpg 
[image4]: ./resources/terrain_detection.jpg 
[image5]: ./resources/coordinate_transform.jpg
[image6]: ./resources/world_map.jpg
[image7]: ./resources/state_diagram.png

The project code is broken up in the following files:

* code/Rover_Project_Test_Notebook.ipynb - A notebook to experiment with methods used in the project
* code/perception.py - All the logic to make robot sense the environment is implemented in this file.
* code/decision.py - The state machine to enable the robot interaction with the environment and carry out decisions based on the perception logic.
* code/drive_rover.py - Communicates with the simulator to drive the robot in autonomous mode.

The project had two main parts. In the first part we were to work with a Jupyter notebook to develop and test functions such as for persepctive transofmration, co-ordinate translation and terrain detection from images.. The second part was to take these functions and drive a Rover autonomously using the Udacity simulator. 

### Notebook Analysis

The steps done in the notebook analysis are explained below. All these methods are then called from the ```process_image()``` function towards the end of the notebook.

** 1. Input Image **

The input from the rover is an image from the front facing camera.

![alt text][image1]

** 2. Perspective Transform **

We then perform a perspective transform using the source points from the front facing image of the rover. A calibration image with a 1 square meter grid marked on the surface was used as the source of the transformation. The transformation is done using OpenCV's ```getPerspectiveTransform()``` and ```warpPerspective()``` methods.
![perspective transform][image2]

** 3. Terrain Detection **

After the perspective transform, we implemented methods to detect navigable terrain, obstacles and rock in the input image using some basic pixel intensity thresholding. 

Rocks were detected by thresholding on the 'Hue' channel after converting the RGB image to HSV (Hue Saturation Value) format. 

```
	hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Yello color range
    low = np.array([20, 100, 100])
    high = np.array([30, 255, 255])

    fltr = cv2.inRange(hsv, low, high)
 ```
 
 ![perspective transform][image3]


Similary the navaiable and non-naviable area was detected by thresholding on pixel intesnity as well. 

```
	rgb_thresh=(160, 160, 160
    terrain = np.zeros_like(img[:,:,0])
    obstacles = np.zeros_like(img[:, :, 0])
    
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    
    # Index the array of zeros with the boolean array and set to 1
    terrain[above_thresh] = 1
    obstacles[~above_thresh] = 1
```

![perspective transform][image4]

** 4. Coordinate Transformation **

The thresholded transform image is converted to robot co-ordinate for computing the angle of movement. Additionally, the terrain image is transformed to world coordinate to build the map. 

![coordinate transform][image5]

Below is the trnasformation of original image to world map by first applying a perspective transform and detecting terrain and obstacles and then finally applying a world coordinate translation. 

![world map][image6]

A great deal of experimentation was done using the notebook to perfect these methods. The extra data collected for this purpose is in ```test_dataset_2``` folder. Additionally the video for applying these methods to the sample dataset is [here](./output/test_mapping.mp4').

### Autonomous Driving (World Navigation)

** Perception Step **

At this step, we took these individual methods from the notebook and implemented a ```perception``` and a ```decision``` step for autonomously navigation the Rover's world. The perception step is implemented in ```perception.py``` file and the decision step is implemented in ```decision.py``. 

The perception step computes the navigation angle and distance to the yellow rock based on the steps mentioned above. The navigation angle is the mean of the navigable terrain that is to the left of the Rover This simple strategy keep the robot always moving towards the left side of the world thus introuducing some level of systematic exploration of the terrain. The code snippet below is from the line # 195:200 of perception.py file.

```
    dist, angles = to_polar_coords(navigable_x_rover, navigable_y_rover)

    idx = np.where((angles >= -5) & (angles <= 30))
    Rover.nav_dists = dist[idx]
    Rover.nav_angles = angles[idx]
```

We also identify the yellow rock at the perception step. If a rock is detected based on number of non-zero pixels in the thresholded image, we let the robot move towards the rock. 

```
   # perception.py:line 202-207
   if(np.count_nonzero(rock_x_rover) + np.count_nonzero(rock_y_rover)) > 20:
        rock_dist, rock_angles = to_polar_coords(rock_x_rover, rock_y_rover)
        Rover.nav_angles = rock_angles
        Rover.sample_dist = np.mean(rock_dist)
    else:
        Rover.sample_dist = np.inf
```

** Decision Step **

Decision step is implemented in ```decision.py``` as a simple state/action model. We implemented following states:

```forward``` - The default state that moves the rover towards the mean navigable area (Note: navigable area is set in perception state). During this state if robot get stuck behind a rock or wall, the robot can move to ```recovery``` state or ```stop``` state. The rover moves to the ```stop``` state if there is no navigable area ahead of the robot and to ```recovery``` state if it is stuck.

```
            if Rover.throttle > 0 and Rover.vel <= 0.01:
                Rover.stuck_epoch += 1
            else:
                Rover.stuck_epoch = 0 # Reset time the robot has been stuck.

            if Rover.stuck_epoch > 10:
                Rover.mode = 'recovery'
```

```stop``` - Stop state was not modified from the original code. 
```rock_picking``` - We come here from forward state and at this state we wil wait until we are in position to pick up the rock. The wait for rock picking is done in ```post_pickup``` state. There is recovery code embeded in the state that make sure, we are not stuck waiting for the robot to get into the position of picking the rock. After certain number of epoch, we bail out by moving to recovery state. 
```Recovery``` - Enabled if the robot is stuck at a given position for certain number of frames (epoch). Once active we move the robot back a certain distance and then try to move it forward by change the state back to foward. 

Below is rough sketch of states and the transition between different states. 

![state diagram][image7]


** Final Result **

The final results of 14 minutes of the rover driving around can be found in the vide (linked to youtube). All these results were collected using 1024 x 768 resolution of the Simulator at the 'Good' graphics quality, getting roughly 20 Frames per seconds. 

[![IMAGE ALT TEXT](http://img.youtube.com/vi/87KjuVPWNUg/0.jpg)](http://www.youtube.com/watch?v=87KjuVPWNUg "Video Title")



