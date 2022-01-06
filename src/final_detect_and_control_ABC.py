#!/usr/bin/env python
import rospy
from cv2 import cv2
import numpy as np
import imutils
import math
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

#             C
#            / \
#           /   \
#          /     \
#         /       \
#        /         \
#       /   ^       \
#      /    |        \
#     /     y         \
#    /          x->    \
#   /                   \
#  A ------------------- B

class ballDetect:
    
    def __init__(self):
        rospy.init_node('detect_and_control')
        rospy.Subscriber("finish_A", Bool, self.detect_cb_A)
        rospy.Subscriber("finish_B", Bool, self.detect_cb_B)
        rospy.Subscriber("finish_C", Bool, self.detect_cb_C)
        self.stepper_pub = rospy.Publisher("stepper_deg", Vector3, queue_size = 1)
        
        self.radius_top = 100
        self.radius_bot = 86.5
        self.upper_link_dis = 85
        self.lower_link_dis = 55

        self.offset_a = 0
        self.offset_b = 10
        self.offset_c = 0

        self.top_a_x = 0
        self.top_a_y = 0
        self.top_a_z = 0

        self.top_b_x = 0
        self.top_b_y = 0
        self.top_b_z = 0

        self.top_c_x = 0
        self.top_c_y = 0
        self.top_c_z = 0

        self.bot_a_x = 0 - float(self.radius_bot/2 * math.sin(math.pi/3))
        self.bot_a_y = 0 - float(self.radius_bot/2 * math.cos(math.pi/3)) 
        self.bot_a_z = 0

        self.bot_b_x = 0 + float(self.radius_bot/2 * math.sin(math.pi/3))
        self.bot_b_y = 0 - float(self.radius_bot/2 * math.cos(math.pi/3))  
        self.bot_b_z = 0

        self.bot_c_x = 0
        self.bot_c_y = 0 + float(self.radius_bot/2)
        self.bot_c_z = 0

        self.angle_x = 0
        self.angle_y = 0
        self.angle_z = 0

        self.is_finish_A = 1
        self.is_finish_B = 1
        self.is_finish_C = 1
        self.ratio = 1
        self.w = 640
        self.h = 480


        self.x_Kp = 7
        self.x_Ki = 0.02
        self.x_Kd = 0.0

        self.y_Kp = 7
        self.y_Ki = 0.02
        self.y_Kd = 0.0

        self.x_vel_Kp = 0.0
        self.x_vel_Ki = 0.0
        self.x_vel_Kd = 0.0

        self.last_x = 0
        self.last_y = 0
        self.x_last_error = 0
        self.y_last_error = 0
        self.x_sum_error = 0
        self.y_sum_error = 0
        self.x_vel_last_error = 0
        self.y_vel_last_error = 0
        self.x_vel_sum_error = 0
        self.y_vel_sum_error = 0

        self.is_reset = 0
        self.pos_limit = 11
        self.x_controller_output = 0
        self.y_controller_output = 0

        self.total_step_A = 0
        self.total_step_B = 0
        self.total_step_C = 0

    def motorCmd(self, stepper_A, stepper_B, stepper_C):
        stepper_A = int(stepper_A * 16 /1.8) + self.offset_a
        stepper_B = int(stepper_B * 16 /1.8) + self.offset_b
        stepper_C = int(stepper_C * 16 /1.8) + self.offset_c
        msg = Vector3()
        msg.x = (stepper_A - self.total_step_A)
        msg.y = (stepper_B - self.total_step_B)
        msg.z = (stepper_C - self.total_step_C)
        print ("rotate: ")
        print ('A = {deg1}, B= {deg2}, C= {deg3}'.format(deg1=msg.x, deg2=msg.y, deg3=msg.z))
        self.stepper_pub.publish(msg)
        
        self.total_step_A = stepper_A
        self.total_step_B = stepper_B
        self.total_step_C = stepper_C
        print ("now: ")
        print ('A = {deg1}, B= {deg2}, C= {deg3}'.format(deg1=stepper_A, deg2=stepper_B, deg3=stepper_C))

    def lawOfCosine(self, link_dis, angle):
        ans = math.degrees(math.acos((-math.pow(self.upper_link_dis, 2) + math.pow(link_dis, 2) + math.pow(self.lower_link_dis, 2)) / (2 * link_dis * self.lower_link_dis)))
        return 180 - (ans + angle)

    def inverseKinematics(self, x, y, z, row, pitch):
        row = math.radians(row)
        pitch = math.radians(pitch)

        tmp_a_x = 0 - float(self.radius_top/2 * math.sin(math.pi/3))
        tmp_a_y = 0 - float(self.radius_top/2 * math.cos(math.pi/3))
        tmp_a_z = 0
        tmp_b_x = 0 + float(self.radius_top/2 * math.sin(math.pi/3))
        tmp_b_y = 0 - float(self.radius_top/2 * math.cos(math.pi/3))
        tmp_b_z = 0
        tmp_c_x = 0
        tmp_c_y = 0 + float(self.radius_top/2)
        tmp_c_z = 0

        self.top_a_x = x + tmp_a_x * math.cos(pitch) + tmp_a_y * math.sin(pitch) * math.sin(row) + tmp_a_z * math.cos(row) * math.sin(pitch)
        self.top_a_y = y + tmp_a_y * math.cos(row) - tmp_a_z * math.sin(row)
        self.top_a_z = z - tmp_a_x * math.sin(pitch) + tmp_a_y * math.sin(row) * math.cos(pitch) + tmp_a_z * math.cos(row) * math.cos(pitch)

        self.top_b_x = x + tmp_b_x * math.cos(pitch) + tmp_b_y * math.sin(pitch) * math.sin(row) + tmp_b_z * math.cos(row) * math.sin(pitch)
        self.top_b_y = y + tmp_b_y * math.cos(row) - tmp_b_z * math.sin(row)
        self.top_b_z = z - tmp_b_x * math.sin(pitch) + tmp_b_y * math.sin(row) * math.cos(pitch) + tmp_b_z * math.cos(row) * math.cos(pitch)

        self.top_c_x = x + tmp_c_x * math.cos(pitch) + tmp_c_y * math.sin(pitch)  * math.sin(row) + tmp_c_z * math.cos(row) * math.sin(pitch)
        self.top_c_y = y + tmp_c_y * math.cos(row) - tmp_c_z * math.sin(row)
        self.top_c_z = z - tmp_c_x * math.sin(pitch) + tmp_c_y * math.sin(row) * math.cos(pitch) + tmp_c_z * math.cos(row) * math.cos(pitch)

        a_xy = math.sqrt(math.pow(self.top_a_x - self.bot_a_x, 2) + math.pow(self.top_a_y - self.bot_a_y ,2))
        a_dis = math.sqrt(math.pow(self.top_a_x - self.bot_a_x, 2) + math.pow(self.top_a_y - self.bot_a_y ,2) + math.pow(self.top_a_z - self.bot_a_z, 2))
        a_angle = math.degrees(math.asin(abs(self.top_a_z - self.bot_a_z)/a_dis))
        
        b_xy = math.sqrt(math.pow(self.top_b_x - self.bot_b_x, 2) + math.pow(self.top_b_y - self.bot_b_y ,2))
        b_dis = math.sqrt(math.pow(self.top_b_x - self.bot_b_x, 2) + math.pow(self.top_b_y - self.bot_b_y ,2) + math.pow(self.top_b_z - self.bot_b_z, 2))
        b_angle = math.degrees(math.asin(abs(self.top_b_z - self.bot_b_z)/b_dis))
        
        c_xy = math.sqrt(math.pow(self.top_c_x - self.bot_c_x, 2) + math.pow(self.top_c_y - self.bot_c_y ,2))
        c_dis = math.sqrt(math.pow(self.top_c_x - self.bot_c_x, 2) + math.pow(self.top_c_y - self.bot_c_y ,2) + math.pow(self.top_c_z - self.bot_c_z, 2))
        c_angle = math.degrees(math.asin(abs(self.top_c_z - self.bot_c_z)/c_dis))

        stepper_command_A = self.lawOfCosine(a_dis, a_angle)
        stepper_command_B = self.lawOfCosine(b_dis, b_angle)
        stepper_command_C = self.lawOfCosine(c_dis, c_angle)

        self.motorCmd(stepper_command_A, stepper_command_B, stepper_command_C)


    def detect_cb_A(self, data):
        if data.data == True:
            self.is_finish_A = 1
        else:
            self.is_finish_A = 0
    def detect_cb_B(self, data):
        if data.data == True:
            self.is_finish_B = 1
        else:
            self.is_finish_B = 0
    def detect_cb_C(self, data):
        if data.data == True:
            self.is_finish_C = 1
        else:
            self.is_finish_C = 0
           
    def detect(self):
        
        cap = cv2.VideoCapture(0)

        if cap.isOpened():
            ret, frame = cap.read()
        else:
            ret = False
        
        while ret:

            frame = frame[int(self.h/2-self.h/2*self.ratio) : int(self.h/2+self.h/2*self.ratio), int(self.w/2-self.w/2*self.ratio): int(self.w/2+self.w/2*self.ratio)]
            blurred_frame = cv2.GaussianBlur(frame, (1, 1), 0)     

            lower = np.array([240, 240, 240])
            upper = np.array([255, 255, 255])
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
                self.inverseKinematics(0, 0, 100, 0, 0)
                time.sleep(5)
            
            #########print#########
            #print('x = {x_val},x_velocity = {x_v_val}'.format(x_val=int(x) - 320, x_v_val=x_velocity))

            ######### serial output data #########
            
            x_error = int(x) - 320
            y_error = -int(y) + 240
            x_vel_error = x_velocity
            y_vel_error = y_velocity
            
            x_pos_output = 0
            x_vel_output = 0

            y_pos_output = 0
            y_vel_output = 0
            print ('x_error = {deg}, vel= {sec}'.format(deg=x_error, sec=x_vel_error))
            print ('y_error = {deg}, vel= {sec}'.format(deg=y_error, sec=y_vel_error))
            if(self.is_finish_A == True and self.is_finish_B == True and self.is_finish_C == True):
                
                #self.is_finish = False
                
                x_pos_output = (x_error / 24 * self.x_Kp + (x_error - self.x_last_error) / 24 * self.x_Kd + self.x_sum_error / 24 * self.x_Ki)
                y_pos_output = (y_error / 24 * self.y_Kp + (y_error - self.y_last_error) / 24 * self.y_Kd + self.y_sum_error / 24 * self.y_Ki) 
                #x_vel_output = (x_vel_error / 24 * self.x_vel_Kp + (x_vel_error - self.x_vel_last_error) / 24 * self.x_vel_Kd + self.x_vel_sum_error / 24 * self.x_vel_Ki)
                x_vel_output = 0
                y_vel_output = 0

                
                
                self.x_controller_output = -(x_pos_output + x_vel_output)
                self.y_controller_output = -(y_pos_output + y_vel_output)

                self.x_sum_error += x_error
                self.y_sum_error += y_error
                self.x_vel_sum_error += x_vel_error
                self.y_vel_sum_error += y_vel_error
                self.x_last_error = x_error
                self.y_last_error = y_error
                self.x_vel_last_error = x_vel_error
                self.y_vel_last_error = y_vel_error

                print('x_control output = {deg}'.format(deg=self.x_controller_output))
                print('y_control output = {deg}'.format(deg=self.y_controller_output))


                if(self.x_controller_output > self.pos_limit):
                    self.x_controller_output = self.pos_limit
                if(self.x_controller_output < -self.pos_limit):
                    self.x_controller_output = -self.pos_limit
                if(self.y_controller_output > self.pos_limit):
                    self.y_controller_output = self.pos_limit
                if(self.y_controller_output < -self.pos_limit):
                    self.y_controller_output = -self.pos_limit
                # time.sleep(2)
                # self.inverseKinematics(0, 0, 100, 0, 0)
                # time.sleep(0.5)
                
                
                self.inverseKinematics(0, 0, 100, self.y_controller_output, -self.x_controller_output)

                

            if (cv2.waitKey(1) & 0xFF == ord('q')) or rospy.is_shutdown(): # exit on q
                break
            ret, frame = cap.read()
        cv2.destroyAllWindows()
        cap.release()
        

        

if __name__ == "__main__":   
    final = ballDetect() 
    final.detect()
    