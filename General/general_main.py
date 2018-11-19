# Nimble.AI General Programming Challenge
# Ryan Kelly
# November 2018

import numpy as np
import time
import cv2
import ctypes
from multiprocessing import Process, Value, Array, Queue, Event


color_to_rgb = {"black"   : (0,0,0),
                "white"   : (255,255,255),
                "red"     : (255,0,0),
                "yellow"  : (255,255,0),
                "lime"    : (0,255,0),
                "aqua"    : (0,255,255),
                "blue"    : (0,0,255),
                "fuschia" : (255,0,128)}
rgb_to_color = {(0,0,0)       : "black",
                (255,255,255) : "white",
                (255,0,0)     : "red",
                (255,255,0)   : "yellow",
                (0,255,0)   : "lime",
                (0,255,255)   : "aqua",
                (0,0,255)     : "blue",
                (255,0,128)   : "fuschia"}
complementary = {"black"   : "white",
                 "white"   : "black",
                 "red"     : "aqua",
                 "yellow"  : "blue",
                 "lime"    : "fuschia",
                 "aqua"    : "red",
                 "blue"    : "yellow",
                 "fuschia" : "lime"} # From  https://en.wikipedia.org/wiki/Complementary_colors in RGB model

color_list = np.array(["black", "white", "red", "yellow", "lime", "aqua", "blue", "fuschia"])

# Process one as defined in the pdf
# Uses numpy to generate RGB image of solid color from color_list
# Next creates wrapper for multiprocessing.Array and passes image 
# to second process using queue
# Not really sure how to create the wrapper for multiprocessing.Array
def process_one(queue_a):
    for _ in range(num_images.value):
        color_choice = np.random.choice(color_list)
        rgb_value = color_to_rgb[color_choice]
        im = np.zeros((width.value,height.value,3))
        im[:] = rgb_value[::-1]
        queue_a.put(im)
    queue_a.put(None)
    return


# Receives images from process one using the queue
# Inspects color of image and watermarks with color name
# Draws a filled circle radius width/4 with complementary color
# Passes to Process 3
def process_two(queue_a, queue_b, event):
    while not event.is_set():
        im = queue_a.get()
        if im is None: break
        
        rgb_top_left = np.array(im[0,0,:], dtype='int32')
        rgb_top_left = tuple(rgb_top_left[::-1])
        color_name = rgb_to_color[rgb_top_left]
        complementary_name = complementary[color_name]
        complementary_rgb = color_to_rgb[complementary_name]
        cv2.putText(im, color_name, (0,height.value), cv2.FONT_HERSHEY_PLAIN, 5, complementary_rgb[::-1])
        
        circle_center = (int(width.value/2), int(height.value/2))
        circle_radius = int(width.value/4)
        thickness = -1 # negative thickness fills the circle
        cv2.circle(im, circle_center, circle_radius, complementary_rgb[::-1], thickness)
        
        queue_b.put(im)
    queue_b.put(None)
    return

# Continually reads from array_a and displays image using opencv.imshow
def process_three(array_a, event):
    while not event.is_set():
        with array_a.get_lock():
            im = np.frombuffer(array_a.get_obj()).reshape(width.value,height.value,3)
            cv2.imshow('im',im)
            c = cv2.waitKey(1)
            if c == ord('q'):
                event.set()


if __name__ == "__main__":
    queue_a = Queue()
    queue_b = Queue()
    print("Welcome to the random colored image generator")
    input_is_int = False
    while not input_is_int:
        num_images = input("Number of Images: ")
        height = input("Image height: ")
        width = input("Image width: ")
        try:
            num_images = Value("i", int(num_images))
            height = Value("i", int(height))
            width = Value("i", int(width))
            if num_images.value < 1 or height.value < 1 or width.value < 1:
                raise Exception
            input_is_int = True
        except:
            print("Please input integers greater than zero for these values")
    print("Press the 'q' key to quit")
     
    array_a_size = height.value*width.value*3
    array_a = Array(ctypes.c_double, array_a_size)
    quit_event = Event()

    p1 = Process(target=process_one, args=(queue_a,))
    p2 = Process(target=process_two, args=(queue_a,queue_b,quit_event))
    p3 = Process(target=process_three, args=(array_a,quit_event))
    p1.start()
    p2.start()
    p3.start()


    while not quit_event.is_set():
        im = queue_b.get()
        if im is None: break
        with array_a.get_lock():
            window = np.frombuffer(array_a.get_obj()).reshape(width.value,height.value,3)
            window[:] = im
    cv2.destroyAllWindows()
    p1.join()
    p2.join()
    p3.join()





