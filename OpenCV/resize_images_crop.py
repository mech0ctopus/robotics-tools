# Utility functions for HW7-8
from PIL import Image
from glob import glob
from os.path import basename, exists
from os import mkdir


def resize_image(input_image_path, output_image_path, size=(224, 224)):
    '''Resize image then save.'''
    # read image
    input_image = Image.open(input_image_path)

    w, h = input_image.size
    y = h / 2 - w / 2
    x = 0

    #cropped image
    cropped_image = input_image.crop((x, y, x+w, y+w))
    # resize image
    element_image = cropped_image.resize((size[0], size[1]), Image.ANTIALIAS)
    # save in output folder
    element_image.save(output_image_path, quality=95)  # 95 is best


def resize_images(input_image_folderpath, size=(224, 224)):
    '''Resize all images in folder then save.'''
    # Append folderpath if needed
    if input_image_folderpath.endswith('\\') == False:
        input_image_folderpath = str(input_image_folderpath) + '\\'
    output_image_folderpath = input_image_folderpath + 'resize' + '\\'
    # Make output folder if it doesn't exist
    if exists(output_image_folderpath) == False:
        mkdir(output_image_folderpath)
    # Build list of input images
    input_images = glob(input_image_folderpath + '*' + '.PNG')
    for input_image in input_images:
        out_filename = output_image_folderpath + basename(input_image).split('.')[0] + '.png'
        print('Resizing ' + str(out_filename))
        resize_image(input_image, out_filename, size=(size[0], size[1]))


if __name__ == '__main__':
    input_image_folderpath = r"G:\WPI\Courses\2019\Deep Learning for Advanced Robot Perception, RBE595\Project\VEHITS\KITTI\Raw"
    resize_images(input_image_folderpath, size=(640, 480))
