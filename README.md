# robot_nav
---
Project to detect front legs of a rack in a warehouse using data from a rosbag file

## About
---
A usb camera stores message data of the warehouse in the topic **mono_front/usb_cam/image_rect_color** while navigating in the warehouse in a rosbag file.

The warehouse contains racks with brightly ultra-marine blue coloured legs. 

Using openCV and python, the messages stored in the bag file are obtained and converted into image frames and the blue colour is clearly extracted by creating and upper and lower blue masks. 

The contours of the detected image is extracted by using **cv2.findcontours** and a bounding rectangle is drawn around the rack legs.

For each image frame, the coordinates of the centre of the bounding rectangles are stored in a list and the distance between subsequent centre points in the bounding rectangles are stored. 

If the distance is greater than a minimum value(dependent on the image size) and position of robot with respect to the 2 rack legs is within 20% of image centre, a message is printed *Front rack detected...".

## Usage
---
In 1 terminal, start `roscore`.

*To set the environment variable, a single machine configuration can be done by pasting this in the terminal and then subsequently starting* `roscore` 
 
`$ export ROS_HOSTNAME=localhost`  
`$  export ROS_MASTER_URI=http://localhost:11311`

In another terminal, navigate to the directory where the the file `front_rack.py` and the rosbag file containing data from the navigation in the warehouse is stored.

If no error is shown,run the script with 
```
python2 front_rack.py
```
The program will then ask for a user input on the name of the rosbag file to be analysed.

## Reasons behind decisions made on the approach
---
1. If `roscore` is not started or shows errors while starting, the program will also show errors since a ros enviroment is required for the program to read the messages in the rosbag file
2. 2 blue masks were used, both containing the upper and lower ranges of blue hsv spectrum instead of just one so as to clearly distinguish the blue colour on the rack from any possible colour in the baackground which may lie in the hsv spectrum range of the individual masks
3. A gaussian blur was also applied to the image to accurately determine the contours of the image
4. The program also only took the bounding rectangles which had a height(h) > width(w) so as to prevent the upper rack from being detected
5. Ideally, 2 bounding rectangles around the front legs of the rack are to be drawn if the rack is in the frame of the camera. However, sometimes it is possible to detect more than 2 bounding rectangles, which just represent the upper vertical part of the rack legs.
6. It was ensured that the distance between each point in the bounding rectangle and the image centre was only deviating about 20% of the distance between the image centre and the subsequent points so as to account for slight variations in the robot's position when the video was being captured.

## Afterthoughts to proposed solution
---
1. If a similarly ultramarine blue coloured object is present on the rack, the camera might detet it as a rack leg and as such result in a false printing of "Front rack detected..."
2. A rack may not be detected in much darker areas where the colour may deviate significantly from the colour detected in the video.





