## IRR Lab 6

This program navigates a maze using a KNN classifier to determine direction.

### How to run
- Run turtlebot bringup on robot `ros2 launch turtlebot3_bringup camera_robot.launch.py`
- Zero the odometry `ros2 run print_fixed_odometry zero_odom`
- Run control logic `ros2 run control get_control`

### File Structure
- `data`: sign image data folder
- `example_knn.py`: Unused, provided knn script
- `generate_requirement.py`: Generate requirements.txt file for environment installation
- `learn_signs.py`: Script for training model and processing images
- `model_grader.py`: Grading script
- `README.md`: YOU ARE HERE
- `requirements.txt`: You generated this, good job
- `saved_model_best.pkl`: My best saved model
- `take_picture.sh`: I have no idea what this is...

### How it works
- `learn_signs.py` loads the data which includes splitting it into training and validation sets, then trains the data. Before training, image preprocessing occurs which includes:
    - Convert images to HSV
    - Mask HSV images to isolate the signs. This was tuned by hand. Combine the masks.
    - Find contours in the mask that are closest to center of the image and larger than some size (most signs should follow this)
    - Crop around the best contour, but if no contour that fits the description, return none class
    - Resize the image
- Training is run on an svm, knn, or rf with the processed training set. Finally, the validation dataset is used to predict and thus validate the trained model.
