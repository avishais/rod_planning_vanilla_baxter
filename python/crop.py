
import matplotlib.pyplot as plt

import numpy as np

t = 510/717.

w = 717+50
y = 250
plt.figure(1)
I1 = plt.imread('baxter1.png')
I1 = I1[y:y+int(w*t),1:w,:]
plt.imshow(I1)
plt.axis('off')
plt.imsave('baxter1p.png', I1)

plt.figure(2)
I2 = plt.imread('baxter2.png')
I2 = I2[y:y+int(w*t),1:w,:]
plt.imshow(I2)
plt.axis('off')
plt.imsave('baxter2p.png', I2)

plt.show()