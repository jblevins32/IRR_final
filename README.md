## IRR Lab 6

This program navigates a maze with traffic signs. A camera is used with a preprocessing pipeline into a KNN classifier to determine which signs are in the robot's view. Once the robot reaches a wall, as determined by the LiDAR measurements, it classifies the sign and takes an action based on that sign. The signs lead the robot to the goal sign which is indicated by a star.

### How to run
- Run turtlebot bringup on robot `ros2 launch turtlebot3_bringup camera_robot.launch.py`
- Zero the odometry `ros2 run print_fixed_odometry zero_odom`
- Run control logic `ros2 run control get_control`

### How the sign classifier works
- `learn_signs.py` loads the sign data which includes splitting it into training and validation sets, then trains the data. Before training, image preprocessing occurs which includes:
    - Convert images to HSV
    - Mask HSV images to isolate the signs. This was tuned by hand. Combine the masks.
    - Find contours in the mask that are closest to center of the image and larger than some size (most signs should follow this)
    - Crop around the best contour, but if no contour that fits the description, return none class
    - Resize the image
- Training is run on an svm, knn, or rf with the processed training set. Finally, the validation dataset is used to predict and thus validate the trained model.
