from sklearn import svm
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import joblib
# import pytorch

def load_data(data_path, label_path, test_data_set=False):

    # Labels:
    #0: none
    #1: left 
    #2: right 
    #3: reverse
    #4: stop
    #5: goal

    # Load images and labels
    image_files = sorted([os.path.join(data_path, fname) for fname in os.listdir(data_path) if fname.endswith('.png')],
                         key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))
    
    images = [cv2.imread(img_path) for img_path in image_files]
    labels = np.loadtxt(label_path, dtype=float, delimiter=',')
    labels_sorted = labels[labels[:,0].argsort()][:,1]
    num_signs = np.sum(labels[:,1] != 0) # Get number of images that are not none class

    # Test images to see if the preprocessing is good! Set view_preprocess = False below to run entire script without interuption
    if test_data_set:
        for test_img in range(len(images)):
            test_img = test_img + 473
            preprocess(images[test_img], set_type = 'test', view_preprocess = True, img_num = test_img)

    # Split into training and testing sets
    images_train, images_test, labels_train, labels_test = train_test_split(images, labels_sorted, test_size=0.3)

    return images_train, images_test, labels_train, labels_test

def preprocess(image, set_type = 'train', view_preprocess = False, img_num = 0):

    # Option to crop image before processing
    # h, w, d = image.shape
    # crop_frac = 0.9
    # ch = int(h * crop_frac)
    # cw = int(w * crop_frac)
    # start_x = (w - cw) // 2
    # start_y = (h - ch) // 2
    # image = image[start_y:start_y + ch, start_x:start_x + cw]

    # Separate the colored part from the background
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # RED
    lower_red1 = np.array([0, 180, 140])
    upper_red1 = np.array([0, 255, 255])
    lower_red2 = np.array([160, 90, 125])
    upper_red2 = np.array([180, 255, 255])

    # GREEN
    lower_green = np.array([20, 35, 35])
    upper_green = np.array([110, 255, 255])

    # BLUE
    lower_blue = np.array([90, 40, 40])
    upper_blue = np.array([150, 240, 240])

    # Extract the colored parts to create a mask
    mask_red1 = cv2.inRange(img, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(img, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(img, lower_green, upper_green)
    mask_blue = cv2.inRange(img, lower_blue, upper_blue)

    mask_combined = cv2.bitwise_or(mask_red,mask_blue)
    mask_combined = cv2.bitwise_or(mask_combined,mask_green)

    # Make img greyscale
    # img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find contours in the mask
    contours,_ = cv2.findContours(mask_combined,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    center_img = np.array([mask_combined.shape[1]/2,mask_combined.shape[0]/2])
    contour_chosen_dist = 10000000000
    contour_chosen = None
    for idx, contour in enumerate(contours):
        center_contour = np.mean(contour[:, 0, :], axis=0)
        contour_dist = np.linalg.norm(center_img - center_contour)

        # print(f"contour {idx} with {contour_dist} dist")

        # Check if there is a large contour in the center (stop sign)
        if contour_dist<70 and contour.shape[0] > 150:
            contour_chosen_dist = contour_dist
            # print(contour_dist)
            # print(contour.shape[0])
            contour_chosen = contour
            chosen_idx = idx
            break
        
        # Find the mask closest to the center and it must be greater than some number of pixels
        elif contour_dist < contour_chosen_dist and contour.shape[0] > 30:
            contour_chosen_dist = contour_dist
            contour_chosen = contour
            chosen_idx = idx

    # print(f"Chosen contour {chosen_idx}")

    # Find min and max and crop chosen contour
    if contour_chosen is not None:
        mins = np.min(contour_chosen,axis=0)
        maxes = np.max(contour_chosen,axis=0)
        contour_y_min = mins[0,0]
        contour_x_min = mins[0,1]
        contour_y_max = maxes[0,0]
        contour_x_max = maxes[0,1]            

        # Final cropped image
        img_cropped = mask_combined[contour_x_min:contour_x_max, contour_y_min:contour_y_max]
        
        # Found sign so do not classify as none
        none_class = False

    else:
        # Found no sign so go ahead and classify as none
        none_class = True
        img_cropped = mask_combined

    # Final modifications: Resize and norm
    img = cv2.resize(img_cropped, (128, 128))

    if view_preprocess:
        fig, ax = plt.subplots(1, 4, figsize=(15, 5))
        ax[0].imshow(mask_combined, cmap='gray')
        ax[0].set_title('Mask')

        ax[1].imshow(img_cropped, cmap='gray')
        ax[1].set_title('Selected Crop')

        ax[2].imshow(img, cmap='gray')
        ax[2].set_title('Sent Image')

        ax[3].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        ax[3].set_title('Original')

        plt.tight_layout()
        plt.show()  

    return img, none_class

def crop_image(img):
    if np.sum(img != 0) != 0:
        x,y = np.where(img != 0)
        x_min = np.min(x)
        x_max = np.max(x)
        y_min = np.min(y)
        y_max = np.max(y)

        # Crop the image according to the mask
        img_cropped = img[x_min:x_max, y_min:y_max]
        return img_cropped
    else:
        # print("No image to crop")
        return img
    
def fit_data(images, labels, alg='knn'):
    # Process training images
    images_train, _ = zip(*[preprocess(image, set_type = 'train') for image in images])

    images_train = np.array(images_train)
    labels = np.array(labels)

    images_flattened = images_train.reshape(len(images_train), -1)  # Flatten images into num_images x num_features (pixels)

    # Choose the model
    if alg == 'svm':
        clf = svm.SVC(kernel='rbf', C=2, gamma=0.0001)
    if alg == 'rf':
        clf = RandomForestClassifier(n_estimators=200)
    if alg == 'knn':
        clf = KNeighborsClassifier(algorithm='brute', n_neighbors=4, n_jobs=-1)

    # Fit the model
    clf.fit(images_flattened, labels)

    return clf

def test(model, images, labels, check_wrong):
    # Process test images
    images_test, none_class = zip(*[preprocess(image, set_type = 'test') for image in images])

    # Convert to numpy and flatten
    images_test = np.array(images_test)
    labels = np.array(labels)

    images_flattened = images_test.reshape(len(images), -1)  # Flatten images into num_images x num_features (pixels)

    # Predict
    labels_predicted = model.predict(images_flattened)

    # Apply the none class mask
    labels_predicted[np.array(none_class)] = 0

    confusion = np.zeros([6,6])
    for idx, image in enumerate(images_test):
        true = labels[idx]
        pred = labels_predicted[idx]

        # Determine which class is having trouble
        if pred != true:
            confusion[int(true),int(pred)] += 1
            # print(f"pred {pred}, true {true}")

            if check_wrong:
                fig, ax = plt.subplots(1, 2, figsize=(10, 5))
                ax[0].imshow(image, cmap='gray')
                ax[0].set_title('fed img')

                img = cv2.cvtColor(images[idx], cv2.COLOR_BGR2RGB)
                ax[1].imshow(img)
                ax[1].set_title('original img')

                plt.tight_layout()
                plt.show()  
            
                ### Check for HSV values in wrong predictions
                img_hsv = cv2.cvtColor(images[idx], cv2.COLOR_BGR2HSV)
                cv2.namedWindow('image')
                cv2.setMouseCallback('image', show_hsv, param=img_hsv)

                while True:
                    cv2.imshow('image', images[idx])
                    if cv2.waitKey(1) & 0xFF == 27:  # Press Esc to exit
                        break
                ### 



    print(f"Num of each class that was classified wrong (confusion just for wrong answers):\n {confusion} or {np.sum(confusion)}/{idx} wrong. Rows are pred, cols are true")

    acc = accuracy_score(labels, labels_predicted)
    print("Accuracy: ", acc)
    return acc

def show_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        img_hsv = param  # param gets passed from setMouseCallback
        pixel = img_hsv[y, x]
        print(f'HSV at ({x}, {y}): H={pixel[0]}, S={pixel[1]}, V={pixel[2]}')

if __name__ =="__main__":

    data_path = "data/2024F_imgs"
    label_path = "data/2024F_imgs/labels.txt"

    # Load data and fit model
    images_train, images_test, labels_train, labels_test = load_data(data_path, label_path)
    model = fit_data(images_train, labels_train, alg='knn')
    joblib.dump(model, 'saved_model.pkl')

    # Run inference
    acc = test(model, images_test, labels_test, check_wrong=False)