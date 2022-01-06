#!/usr/bin/env python
import rospy
from cv2 import cv2
import numpy as np
import imutils
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class ballDetect:
    
    def __init__(self):
        rospy.init_node('detect_and_control')
        rospy.Subscriber("finish", Bool, self.detect_cb)
        self.stepper_pub = rospy.Publisher("stepper_deg", Int32, queue_size = 1)
        self.is_finish = 1
        self.ratio = 1
        self.w = 640
        self.h = 480

        self.deg2step = 16/1.8

        self.x_margin = 0
        # self.x_Kp = 5.3
        # self.x_Ki = 0.025
        self.x_Kp = 5.75
        self.x_Ki = 0.02
        self.x_Kd = 0.0

        self.x_vel_Kp = 0.0
        self.x_vel_Ki = 0.000
        self.x_vel_Kd = 0.00

        self.last_x = 0
        self.last_y = 0
        self.x_last_error = 0
        self.x_sum_error = 0
        self.x_vel_last_error = 0
        self.x_vel_sum_error = 0
        self.is_reset = 0
        self.pos_limit = 80
        self.controller_output = 0
        self.total_step = 0

        self.debug = -1

    def detect_cb(self, data):
        if data.data == True:
            self.is_finish = 1
        else:
            self.is_finish = 0
        
    def detect(self):
        cap = cv2.VideoCapture(2)

        if cap.isOpened():
            ret, frame = cap.read()
        else:
            ret = False
        
        while ret:

            frame = frame[int(self.h/2-self.h/2*self.ratio) : int(self.h/2+self.h/2*self.ratio), int(self.w/2-self.w/2*self.ratio): int(self.w/2+self.w/2*self.ratio)]
            blurred_frame = cv2.GaussianBlur(frame, (1, 1), 0)     

            lower = np.array([204, 0, 0])
            upper = np.array([255, 255, 51])
            mask = cv2.inRange(blurred_frame, lower, upper)
            cv2.imshow("raw", mask)

            kernel = np.ones((15, 15), np.uint8)
            newMask = cv2.erode(mask, kernel)
            newMask = cv2.dilate(newMask, kernel)
            cv2.imshow("newMask", newMask)

            x = 160
            y = 120
            cnts = cv2.findContours(newMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = (0, 0)
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if(not M["m00"] == 0 ): #in case of divide by 0
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) 
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    cv2.circle(blurred_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2) 
                    cv2.circle(blurred_frame, center, 2, (0, 0, 255), -1) 


            cv2.circle(blurred_frame, (int(self.w/2*self.ratio),int(self.h/2*self.ratio)), 5, (0, 255, 0), -1)
            cv2.circle(blurred_frame, (int(self.w/2*self.ratio),int(self.h/2*self.ratio-50)), 5, (0, 255, 0), -1)
            cv2.circle(blurred_frame, (int(self.w/2*self.ratio+50),int(self.h/2*self.ratio)), 5, (0, 255, 0), -1) #
            cv2.imshow("blurred_frame", blurred_frame)
            x_velocity = 0
            y_velocity = 0
            if(self.is_reset == 1):
                x_velocity = int(x) - self.last_x
                y_velocity = -int(y) + self.last_y
                self.last_x = int(x)
                self.last_y = int(y)
            else:
                self.is_reset = 1
                self.last_x = int(x)
                self.last_y = int(y)
            
            #########print#########
            #print('x = {x_val},x_velocity = {x_v_val}'.format(x_val=int(x) - 320, x_v_val=x_velocity))

            ######### serial output data #########
            
            x_error = int(x) - 320
            x_vel_error = x_velocity
            tmp_ptr_deg = 0
            tmp_ptr_dir = 0
            pos_output = 0
            vel_output = 0
            print ('x_error = {deg}, vel= {sec}'.format(deg=int(x) - 320, sec=x_vel_error))
            if(1):
                # if(abs(x_vel_error) > 50 and abs(x_vel_error) < 5):
                #     x_vel_error = 0
                # else:
                #     x_vel_error = x_vel_error
                self.is_finish = False
                pos_output = (x_error / 24 * self.x_Kp + (x_error - self.x_last_error) / 24 * self.x_Kd + self.x_sum_error / 24 * self.x_Ki) 
                #vel_output = (x_vel_error / 24 * self.x_vel_Kp + (x_vel_error - self.x_vel_last_error) / 24 * self.x_vel_Kd + self.x_vel_sum_error / 24 * self.x_vel_Ki)
                vel_output = 0
                if(abs(x_vel_error) > 20):
                    vel_output = x_vel_error / 24 
                self.controller_output = -(pos_output + vel_output)
                self.x_sum_error += x_error
                self.x_vel_sum_error += x_vel_error
                
                self.x_last_error = x_error
                self.x_vel_last_error = x_vel_error

                print('control output = {deg}'.format(deg=self.controller_output))
                # deg_str = bytes(str(int(tmp_ptr_deg)))
                # dir_str = bytes(str(int(tmp_ptr_dir)))
                if(self.controller_output > self.pos_limit):
                    self.controller_output = self.pos_limit
                if(self.controller_output < -self.pos_limit):
                    self.controller_output = -self.pos_limit

                print('acturator = {deg}'.format(deg=self.controller_output))
                msg = Int32()
                msg.data = (self.controller_output - self.total_step)
                
                self.stepper_pub.publish(msg)
                print('now = {ori_deg}, spin = {deg}'.format(ori_deg=(self.total_step),deg=(msg.data)))
                self.total_step = self.controller_output

            if (cv2.waitKey(1) & 0xFF == ord('q')) or rospy.is_shutdown(): # exit on q
                break
            ret, frame = cap.read()
        cv2.destroyAllWindows()
        cap.release()
        

        

if __name__ == "__main__":   
    final = ballDetect() 
    final.detect()
    