#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from norbit_fls_driver.msg import Fls
import numpy as np
import cv2
import time
import math

__author__ = "Rayan Cali"
__author_email = "rayanc@kth.se"

def image_callback(msg):
    global queue, amount_of_pics, pub, sub_goal,done_once,iteration, width, height, fused, cutoff_perc
        #delete the first element and add the new one
    t = time.time()
    bin_max = ((msg.t0 + msg.num_samples)*msg.snd_velocity)/(2*msg.sample_rate)
    if len(queue) < amount_of_pics:
        
        if height != msg.num_samples or width != msg.num_beams:
            queue = []
            fused.width = msg.num_beams
            fused.height = msg.num_samples
            fused.step = msg.num_beams
        height = msg.num_samples
        width = msg.num_beams
        imagearray = np.array(msg.fls_raw.data[:-width])
        imagearray = np.reshape(imagearray, (height,width))
        imagearray[0:int(height*cutoff_perc), :] = 0
        # imagearray[:, 0:int(width*0.3)] = 0
        imagearray, vgamask=remove_gains(imagearray, msg.gain, msg.vga_t1,msg.vga_t2,msg.vga_g1,msg.vga_g2,msg.snd_velocity, msg.tx_freq, int(height*cutoff_perc), bin_max)
        queue.append(imagearray)
    if len(queue) == amount_of_pics:
        fused_pic = fuse_the_pics(vgamask, msg.gain)
        fused_stretched = linear_stretch(fused_pic)
        median_filtered_pic = median_filtered(fused_stretched)
        otsu = otsu_algorithm(median_filtered_pic)
        morphed = morphological_operations(otsu)
        
   
        del queue[0]
    
    elapsed = time.time() - t
    rospy.loginfo(elapsed)
    return


def remove_gains(imagearray, gain, t1, t2, g1, g2, snd_vel, freq, start_range, max_range):
    global height, remove_vga, use_self_calc_gain
    vgamask = np.zeros(imagearray.shape[0])
    vgamask[:t1] = g1
    vgamask[t2:] = g2
    vgamask[t1:t2] = np.linspace(g1,g2,t2-t1)
    vgamask = vgamask.transpose()
    f= lambda x: 10**(x/20)
    vgamask = f(vgamask)
    if remove_vga:
        imagearray = (imagearray.transpose()/vgamask).transpose()
    imagearray = imagearray/((10**gain)/20)
    if use_self_calc_gain:
        start_range = start_range*max_range/height
        viscosity = 1.18e-3 #viscosity of seawater 15 degrees celcius
        angfreq = 2*math.pi*freq*1e3 #angular frequency
        density = 1020 #density of seawater
        ampllist = np.ones(imagearray.shape[0])*2*viscosity*angfreq**2/(3*density*snd_vel**3)
        rangelist = np.linspace(start_range,max_range,imagearray.shape[0])*2
        ampllist = ampllist*rangelist**2
        imagearray = (imagearray.transpose()*ampllist).transpose()
        absorbcoeff = 0.06388 #absorption coefficient of seawater 15 degrees celcius, calculated with 10ppt salinity, PH 8, depth 50m, and freq 400kHz,
        absorb = rangelist*absorbcoeff
        absorb = f(absorb)
        imagearray = (imagearray.transpose()*absorb).transpose()

    max_value = np.max(imagearray)
    imagearray_scaled = (255/max_value)*imagearray
    imagearray_int = imagearray_scaled.astype(np.uint8)    
    fused.data = imagearray_int.flatten().tolist()
    pub0.publish(fused)
    return imagearray,vgamask



def fuse_the_pics(vgamask,gain):
    global queue,amount_of_pics, pub, sub_goal,done_once,fused, remove_vga, use_self_calc_gain, use_log
    summed_pics = np.sum(queue, axis=0)
    summed_pics = summed_pics*((10**gain)/20)
    if use_log:
        summed_pics_nozeros = summed_pics[summed_pics != 0]
        summed_pics_nozeros = np.log(summed_pics_nozeros)
        sorted_summed_pics_non_zeros = -np.sort(-summed_pics_nozeros, axis=None)
    
    
        top1percent_summed_pics = sorted_summed_pics_non_zeros[:int(len(sorted_summed_pics_non_zeros)*0.01)]
        th = np.sum(top1percent_summed_pics)/(len(top1percent_summed_pics)*amount_of_pics)
        summed_pics = np.log(summed_pics)
        summed_pics[summed_pics <= th] = -np.inf  
        summed_pics = np.exp(summed_pics)
    else: 
        sorted_summed_pics = -np.sort(-summed_pics, axis=None)
        sorted_summed_pics_non_zeros = sorted_summed_pics[sorted_summed_pics != 0]
        top1percent_summed_pics = sorted_summed_pics_non_zeros[:int(len(sorted_summed_pics_non_zeros)*0.01)]
        th = np.sum(top1percent_summed_pics)/(len(top1percent_summed_pics)*amount_of_pics)
        summed_pics[summed_pics <= th] = 0
    
    if remove_vga and not use_self_calc_gain:
        summed_pics = (summed_pics.transpose()*vgamask).transpose()
    
    max_value = np.max(summed_pics)#sorted_summed_pics_non_zeros[0]
    fused_pic = (255/max_value)*summed_pics
    fused_pic_int = fused_pic.astype(np.uint8)
    
 
    fused.data = fused_pic_int.flatten().tolist()
    pub.publish(fused)
   
    return fused_pic

def linear_stretch(fused_pic):
    global fused, pub2
    flat_sorted = np.sort(fused_pic, axis=None)
    flat_sorted = flat_sorted[flat_sorted != 0]
    
    clipped_flat_sorted = flat_sorted[int(0.02*len(flat_sorted)):int(0.98*len(flat_sorted))]
    max_value = clipped_flat_sorted[-1]
    min_value = clipped_flat_sorted[0]

    f = lambda x: 255*(x-min_value)/(max_value-min_value)
    fused_pic[fused_pic<min_value] = min_value
    fused_pic[fused_pic>max_value] = max_value
    contrast_stretched = f(fused_pic)
    fused_pic_int = contrast_stretched.astype(np.uint8)

    fused.data = fused_pic_int.flatten().tolist()
    pub2.publish(fused)
    return contrast_stretched

def median_filtered(pic):
    global fused,pub3, size_medianfilter
    cv_pic = pic.astype(np.uint8)
    median_filtered_pic = cv2.medianBlur(cv_pic, size_medianfilter)
    fused.data = median_filtered_pic.flatten().tolist()
    pub3.publish(fused)
    return median_filtered_pic
   
def otsu_algorithm(pic):
    global fused,pub4

    ret, otsu = cv2.threshold(pic.astype(np.uint8),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
  
    fused.data = otsu.flatten().tolist()
    pub4.publish(fused)
    return otsu
def morphological_operations(pic):
    global fused,pub5, morph_width, morph_height
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (morph_width, morph_height))
    dilated = cv2.dilate(pic,kernel,iterations = 1)
    eroded = cv2.erode(dilated,kernel,iterations = 1)
    morphed = eroded
  
    
    fused.data = morphed.flatten().tolist()

    pub5.publish(fused)
    return morphed

    

    



rospy.init_node('filter')

sub_goal = rospy.Subscriber('/fls/data', Fls, image_callback)

pub0 = rospy.Publisher('/gain_removed', Image, queue_size=1)
pub = rospy.Publisher('/fused', Image, queue_size=1)
pub2 = rospy.Publisher('/fused_and_stretched', Image, queue_size=1)
pub3 = rospy.Publisher('/median_filtered', Image, queue_size=1)
pub4 = rospy.Publisher('/otsu', Image, queue_size=1)
pub5 = rospy.Publisher('/morphed', Image, queue_size=1)

amount_of_pics = rospy.get_param("/nr_of_pics")
cutoff_perc = rospy.get_param("/cutoff_percentage")
remove_vga = rospy.get_param("/remove_vga")
use_self_calc_gain = rospy.get_param("/use_self_calc_gain")
use_log = rospy.get_param("/use_log")
size_medianfilter = rospy.get_param("/size_medianfilter")
morph_width = rospy.get_param("/morph_width")
morph_height = rospy.get_param("/morph_height")
queue = []
iteration = 0
done_once = False
width = 256
height = 650
fused = Image()
fused.height = height
fused.width = width
fused.encoding = 'mono8'
fused.step = width
fused.header.frame_id = ''
fused.is_bigendian = 0
if __name__ == '__main__':
   
    rospy.spin()
