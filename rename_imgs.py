import os
import numpy as np

# parse through images and rename them
path = './data/2025S_imgs'

imgs = os.listdir(path)
imgs.sort()

labels = np.loadtxt(path + '/labels.txt', dtype=float, delimiter=',')
labels[:,0] = labels[:,0] + 321

# Replace the labels in the text file
np.savetxt(path + '/labels.txt', labels, fmt='%d', delimiter=',')

for i, file in enumerate(imgs):
    ext_old = os.path.splitext(file)[0]
    ext_new = str(int(ext_old) + 321)
    old_path = os.path.join(path, ext_old + '.png')
    new_path = os.path.join(path, ext_new + '.png')
    os.rename(old_path, new_path)



