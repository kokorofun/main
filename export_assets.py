import numpy as np
import PIL
from PIL import Image
from math import ceil

files = ["idle_front.png", "idle_side.png", "walk_side1.png", "walk_side2.png"]
for file in files:
    I = np.asarray(PIL.Image.open(file))
    print(I.shape)

    with open(file.replace('.png', '.raw'), 'wb+') as f: 
        for row in range(0, I.shape[0]):
            img_row = I[row]
            desired_width = ceil(I.shape[1]/8)*8
            if len(img_row) < desired_width:
                img_row = [*list(img_row), *[0 for _ in range(0, desired_width - len(img_row))] ]
            for col in range(0, desired_width//8):
                s = "".join([ ("0" if el != 1 else "1") for el in img_row[col*8:(col + 1)*8]])
                f.write(bytes([int(s, 2)]))


