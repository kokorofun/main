import glob
import numpy as np
import PIL
from PIL import Image
from math import ceil

import os
import shutil

TEXTURES_PATH = './assets/textures'
TEXTURES_TARGET = './src/assets/textures'
TEXTURES_GEN_TMP_PATH = './rs_assets_tmp'


dirpath = os.path.join(TEXTURES_GEN_TMP_PATH)
if os.path.exists(dirpath):
    shutil.rmtree(dirpath)

HEADER = """
// Kokorofun@2023

use crate::utils::Texture;

"""

def gen(find_path):

    to_create = find_path.replace(TEXTURES_PATH, '')
    
    final_folder = TEXTURES_GEN_TMP_PATH + to_create
    os.mkdir(final_folder)

    mod_rs = HEADER

    next_folders = []
    for file in glob.iglob(find_path+ '/*'):


        # Guard clause 
        if os.path.isdir(file):
            next_folders.append(file)
            continue


        name = file.split('/')[-1].split('.')[0]


        resized_file = final_folder + '/' + name + '_resized.png'



        I = np.asarray(PIL.Image.open(file))
        if 'uru' not in file:
            os.system(f'convert {file} -interpolate Nearest -filter point -resize 400% {resized_file}')
            # print(f"convert -size {I.shape[0]*4}x{I.shape[1]*4} {file} {resized_file}")
            I = PIL.Image.open(resized_file)
            # I = I.resize([I.size[0]*2, I.size[1]*2], PIL.Image.NEAREST)
            I = np.asarray(I)
            os.system(f'rm {resized_file}')
        else:
            I = PIL.Image.open(file)
            I = I.resize([I.size[0]*2, I.size[1]*2], PIL.Image.NEAREST)
            I = np.asarray(I)

        arr_size = ceil((I.shape[1])/8) * I.shape[0
        ]

        mod_rs += '\n\n'

        base_array = '['
        alpha_array = '['

        for row in range(0, I.shape[0]):
            img_row = I[row]
            desired_width = ceil(I.shape[1]/8)*8
            if len(img_row) < desired_width:
                img_row = [
                    *list(img_row),
                    *[0 for _ in range(0, desired_width - len(img_row))]]
            for col in range(0, desired_width//8):
                chunk = img_row[col * 8: (col + 1) * 8]
                s = "".join(
                    [("0" if el != 1 else "1")
                        for el in chunk])
                
                s_alpha = "".join(
                    [("0" if el != 2 else "1")
                        for el in chunk])
                
                base_array += f'0x{int(s, 2):02x},'
                alpha_array += f'0x{int(s_alpha, 2):02x},'
        
        base_array = base_array[0:-1] + ']'
        alpha_array = alpha_array[0:-1] + ']'

        mod_rs += f'pub const base_{name}: [u8; {arr_size}] = {base_array};\n'
        mod_rs += f'pub const alpha_{name}: [u8; {arr_size}] = {alpha_array};\n'


        mod_rs += f"""
pub const {name}: Texture = Texture {'{'}
    base: &base_{name},
    alpha: Some(&alpha_{name}),
    width: {I.shape[1]},
    height: {I.shape[0]}
{'}'};\n\n
"""


    for folder in next_folders:
        mod_rs += f"pub mod {folder.split('/')[-1]};\n"
    
    with open(TEXTURES_GEN_TMP_PATH + to_create + '/mod.rs', 'w+') as f:
        # print(mod_rs)
        f.write(mod_rs)

        # print(file)
    
    for folder in next_folders:
        gen(folder)

gen(TEXTURES_PATH)

dirpath = os.path.join(TEXTURES_TARGET)
if os.path.exists(dirpath):
    shutil.rmtree(dirpath)
shutil.move(TEXTURES_GEN_TMP_PATH, TEXTURES_TARGET)

# files = ["idle_front.png", "idle_side.png", "walk_side1.png", "walk_side2.png"]
# for file in files:
