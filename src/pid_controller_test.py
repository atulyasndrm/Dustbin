import math 
import time
import tf
import rospy
from mav_msgs.msg import RollPitchYawrateThrust
# from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

rospy.init_node('pid_try', anonymous=True)

class pidcontroller:
    # Defining the P,I,D control parameters
    def __init__(self, kp=[0.05,0.05,0.01], kd=[0,0,0],ki=[0.0,0.000,0.01]):
        # self.talker =  Protocol(IP, PORT)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # self.breaker = True 
        self.prev_time = [0.0,0.0,0.0]
        self.error_tol=0.01
        self.prev_error = [0.0,0.0,0.0]
        self.e_i = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        self.speed = 1 
        self.repeat_pilot = False
        self.single_point_error = 0.001
        self.multiple_point_error = 0.2 #0.1
        self.ang_error = 0.05 #0.02
        self.length = 1
        self.curr_pos=[0.0,0.0,0.0]
        self.curr_ori=[0,0,0,15]
        self.equilibrium_pose = [0,0,0,15]#need to do something about thrust

    def callback(self,msg):
        self.curr_pos[0]=msg.pose.pose.position.x
        self.curr_pos[1]=msg.pose.pose.position.y
        self.curr_pos[2]=msg.pose.pose.position.z

    def talker(self ,roll, pitch, yaw, thrust):
        p=RollPitchYawrateThrust()
        p.pitch=pitch
        p.roll=roll
        p.thrust.z=thrust
        p.yaw_rate=yaw
        pub = rospy.Publisher('/firefly/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
        # while not rospy.is_shutdown():  
        pub.publish(p)

    def listener(self):
        rospy.Subscriber("/firefly/ground_truth/odometry",Odometry,self.callback)
        
            
    def calc_error(self,i,error):
        print(i) # Calculates the error, its `derivative' and `integral'
        curr_time = time.time() 
        dt = 0.0
        if curr_time != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        e_p = error
        self.e_i[i] += error * dt
        e_d = 0
        if dt > 0:
            e_d = de/dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        return (self.kp[i] * e_p) + (self.ki[i]*self.e_i[i]) + (self.kd[i]*e_d)


    def pos_change(self,targ_pos=([0,0,0]),curr_pos = ([0,0,0]), curr_ori = ([0,0,0,15])):# Corrects only position
        # if not self.breaker:
        errors = (targ_pos[0]-curr_pos[0], targ_pos[1]-curr_pos[1], targ_pos[2]-curr_pos[2])
        for i in range(len(errors)):
            self.vel[i] = self.calc_error(i,errors[i])
        if max(errors) > self.error_tol or min(errors) < -self.error_tol:
            for(i) in range(2): 
                curr_temp=self.equilibrium_pose[i]+self.speed*self.vel[i]
                if((curr_temp<40) and (curr_temp>0)):
                    curr_ori[i]=curr_temp
                else:
                    if(curr_temp>40):
                        curr_ori[i]=40
                    else:
                        curr_ori[i]=0
            
            curr_ori[3] = curr_ori[3]+self.speed*self.vel[2]
            if(curr_ori[3]+self.speed*self.vel[2]<0):
                curr_ori[3]=0


            # self.talker.set_RPY_THR(curr_ori[0], curr_ori[1], curr_ori[2], curr_ori[3]+self.speed*self.vel[2])
            self.talker(curr_ori[0], curr_ori[1], curr_ori[2], curr_ori[3])

            # Thrust will be changed by self.speed*self.vel[2]
            # Roll will be changed by self.speed*self.vel[0]
            # Pitch will be changed by self.speed*self.vel[1]
            self.reach_pose = False
        else:
            self.reach_pose = True
            print ("reached destination coordinates")
            print ("error:",np.linalg.norm(errors)) 

    def autopilot(self,targ_pos):
        # self.curr_pos and orientation needs to be read
        self.listener()
        if  self.curr_pos != targ_pos:
            self.reach_pose = False
        # while not self.reach_pose:
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.listener()
            self.pos_change(targ_pos,self.curr_pos,self.curr_ori)
            r.sleep()
            # if self.reach_pose:
            #     break
            
pluto = pidcontroller()
pluto.autopilot([0,0,3])
    

# from protocol import Protocol 
# class heightcontroller:

#     def __init__(self, kp, kd, ki):
#         self.talker = Protocol( IP, PORT)
#         self.kp = kp
#         self.kd = kd
#         self.ki = ki
#         self.vel = 0
#         self.breaker = True
#         self.prev_time = 0
#         self.prev_error = 0
#         self.e_i = 0
#         self.speed = 0.0001
#         self.error_tol = 0.01
#         self.height = 0

#     def calc_error(self,error): # Calculates the error, its `derivative' and `integral' and their sum along with constants multiplied
#         curr_time = time.time()
#         dt = 0.0
#         if curr_time != 0.0:
#             dt = curr_time - self.prev_time
#         de = error - self.prev_error
        
#         e_p = self.kp * error
#         self.e_i += error * dt
#         e_d = 0
#         if dt > 0:
#             e_d = de/dt
#         self.prev_time = curr_time
#         self.prev_error = error
#         print("time", dt, de)
#         return e_p + (self.ki*self.e_i) + (self.kd*e_d)

#     def pos_change(self,targ_pos): # Corrects only position
#         # if not self.breaker:
#         errors = targ_pos - self.height
#         self.vel = self.calc_error(errors)
#         self.height = self.height + self.vel*self.speed
#         print("height:", self.height)

#         # if abs(errors) > abs(self.error_tol) :
#         #     #here we need to tell it to inc or dec its height
#         #     self.height =self.height + self.vel*self.speed #how does this sound for testing?
#         #     self.reach_pose = False
#         #     print("height:", self.height)
#         # else:
#         #     self.reach_pose = True
#         #     print ("reached destination coordinates")
#         #     print ("error:",np.linalg.norm(errors))

#     def autopilot(self, targ_pos):
#         # if self.height != targ_pos:
#         #     self.breaker = False
#         # while not self.breaker:
#         while 1:
#             self.pos_change(targ_pos)
#             # if self.reach_pose:
#             #     break
# start =time.time()
# hello = heightcontroller(0.5, 0.05, 0.05)
# hello.autopilot(target)
# end=time.time()
# print(end-start)



    
    
    


        
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass