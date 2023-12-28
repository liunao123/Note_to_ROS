#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 
class VideoRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/hk_camera/image_color', Image, self.image_callback)
        self.frames = []
        self.recording = False
 
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.recording:
                self.frames.append(cv_image)
        except Exception as e:
            print(e)
 
    def start_recording(self):
        self.frames = []
        self.recording = True
 
    def stop_recording(self):
        self.recording = False
        if self.frames:
            self.save_frames()
 
    def save_frames(self):
        for i, frame in enumerate(self.frames):
            filename = 'frame_{:04d}.jpg'.format(i)
            cv2.imwrite(filename, frame)
        print('Saved {} frames.'.format(len(self.frames)))
 
if __name__ == '__main__':
    rospy.init_node('video_recorder_node', anonymous=True)
    recorder = VideoRecorder()
 
    try:
        while not rospy.is_shutdown():
            cmd = raw_input("Enter 'start' to begin recording or 'stop' to stop recording: ")
            if cmd == 'start':
                recorder.start_recording()
            elif cmd == 'stop':
                recorder.stop_recording()
    except rospy.ROSInterruptException:
        pass