import lab10_map
import math
import particle_filter
import odometry
import pid_controller
import create2
from math import *
import random
import scipy
from scipy import stats
import numpy as np
from numpy import *
from numpy.random import *

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab10_map.Map("lab10.map")

        self.odometry = odometry.Odometry()
        #self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        #self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pf = particle_filter.ParticleFilter()

    def sleep(self, time_in_sec, dt = 0):
        start = self.time.time()
        last_update = None
        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:
                if last_update == None or t - last_update >= dt:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                
                    last_update = t
            if start + time_in_sec <= t:
                    break



    def movement(self, move):
        # goal_x = 0.5
        # goal_y = 0
        base_speed = 100

        goal_theta = -math.pi / 2.0
        # start_time = self.time.time()

        
        # start_distance = 0

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        if move == "forward":
            # while True:
            #     state = self.create.update()
            #     if state is not None:
            #         self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            #         goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            #         theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))


            #         output = self.pidDistance.update(start_distance, goal_distance, self.time.time())
            #         # use odometry 
            #         start_distance = self.odometry.x
            #         print("output is {}".format(output))
            #         self.create.drive_direct(int(base_speed - output), int(base_speed + output))
            #         if (math.fabs(output - goal_distance) < 0.2):
            #             print("returning")
            #             self.create.stop()
            #             return
            x = self.odometry.x
            y = self.odometry.y
            while True:
                self.create.drive_direct(500, 500)
                self.sleep(0.001)
                dx = self.odometry.x - x
                dy = self.odometry.y - y
                # print(dx, dy, math.sqrt(dx * dx + dy * dy))
                if math.sqrt(dx * dx + dy * dy) >= 0.5:
                    break
            self.create.drive_direct(0, 0)


        if move == "left":
            # while True:
            #     state = self.create.update()
            #     if state is not None:
            #         self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            #         output_theta = self.pidTheta.update(self.odometry.theta, -goal_theta, self.time.time())
            #         self.create.drive_direct(int(output_theta), int(-output_theta))
            #         if (math.fabs(output_theta - goal_theta) < 0.5):
            #             print("returning")
            #             self.create.stop()
            #             return

            target_theta = self.odometry.theta + (math.pi/2)
            while True:
                self.create.drive_direct(100, -100)
                self.sleep(0.001)
                # print(math.degrees(self.odometry.theta), math.degrees(target_theta))
                if particle_filter.angle_adjust(target_theta - self.odometry.theta) <= 0.01:
                    break
            self.create.drive_direct(0, 0)



        if move == "right":
            # while True:
            #     state = self.create.update()
            #     if state is not None:
            #         self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            #         output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            #         self.create.drive_direct(int(output_theta), int(-output_theta))
            #         if (math.fabs(output_theta - goal_theta) < 0.5):
            #             print("returning")
            #             self.create.stop()
            #             return

            target_theta = self.odometry.theta - (math.pi/2)
            while True:
                self.create.drive_direct(-100, 100)
                self.sleep(0.001)
                # print(math.degrees(self.odometry.theta), math.degrees(target_theta))
                if particle_filter.angle_adjust(self.odometry.theta - target_theta) <= 0.01:
                    break
            self.create.drive_direct(0, 0)

    def run(self):
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        self.virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)

        # self.virtual_create.set_point_cloud(data)
        self.updateVREP()

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        print(self.map.closest_distance((0.5,0.5), math.pi))

        # Start the robot
        self.create.start()
        self.create.safe()

        # This is an example on how to detect that a button was pressed in V-REP
        start_time = 0
        finish = False
        start_time = self.time.time()

        # Automatic
        # while True:    

        #     distance = self.sonar.get_distance()

        #     if distance > 0.7:
        #         self.movement("forward")
        #         self.pf.move("forward")
        #         self.updateVREP()

        #         mean = self.sonar.get_distance()
        #         self.measureProb(mean)
        #         self.resample()
        #         self.updateVREP()
        #         finish = self.testFinish()
        #         if finish is True:
        #             break

        #     self.movement("right")
        #     self.pf.move("right")
        #     self.updateVREP()

        #     mean = self.sonar.get_distance()
        #     self.measureProb(mean)
        #     self.resample()
        #     self.updateVREP()
        #     finish = self.testFinish()
        #     if finish is True:
        #         break

        #     distance = self.sonar.get_distance()
        #     if distance > 0.7:
        #         self.movement("forward")
        #         self.pf.move("forward")
        #         self.updateVREP()

        #         mean = self.sonar.get_distance()
        #         self.measureProb(mean)
        #         self.resample()
        #         self.updateVREP()
        #         finish = self.testFinish()
        #         if finish is True:
        #             break

        #     self.movement("right")
        #     self.pf.move("right")
        #     self.updateVREP()

        #     mean = self.sonar.get_distance()
        #     self.measureProb(mean)
        #     self.resample()
        #     self.updateVREP()
        #     finish = self.testFinish()
        #     if finish is True:
        #         break

        #     self.movement("right")
        #     self.pf.move("right")
        #     self.updateVREP()

        #     mean = self.sonar.get_distance()
        #     self.measureProb(mean)
        #     self.resample()
        #     self.updateVREP()
        #     finish = self.testFinish()
        #     if finish is True:
        #         break

             
        # end_time = self.time.time() - start_time        
        # print("Done.")
        # print("Time used: {}".format(end_time))
        # while True:
        #     pass

        # Manual
        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                print("Forward pressed!")
                # real robot movement (0.5m forward)
                self.movement("forward")
                self.pf.move("forward")
                self.updateVREP()



            elif b == self.virtual_create.Button.TurnLeft:
                print("Turn Left pressed!")
                # real robot movement (90 degrees left)
                self.movement("left")
                self.pf.move("left")
                self.updateVREP()

            elif b == self.virtual_create.Button.TurnRight:
                print("Turn Right pressed!")
                # real robot movement (90 degrees right)
                self.movement("right")
                self.pf.move("right")
                self.updateVREP()

            elif b == self.virtual_create.Button.Sense:
                print("Sense pressed!")
                # get sensor reading
                mean = self.sonar.get_distance()
                self.measureProb(mean)
                self.resample()
                self.updateVREP()
                self.testFinish()
                if finish is True:
                    while True:
                        pass         

            self.time.sleep(0.01)

    def measureProb(self,mean):
        sum = 0
        # assignment weight using pdf
        for i in range(500):
            x = float(self.pf.pList[i].x)
            y = float(self.pf.pList[i].y)
            d = float(self.pf.pList[i].d)
            w_prev = self.pf.pList[i].w_prev
            # get each individual particle distance
            dp = self.map.closest_distance((x,y), d) 
            # assign probablility to each partcile
            if dp is not None:
     
                # the third variable is the variance for sonar sensor
                psr = stats.norm.pdf(mean, dp, 0.1)

                pr = w_prev

                numerator = psr*w_prev

                self.pf.pList[i].w_prev = numerator

            else:
                self.pf.pList[i].w_prev = 0

            sum = sum + (self.pf.pList[i].w_prev)

        # for i in range(500):
        #     print("before normalization, the {}th particle's numberator is {}".format(i, self.pf.pList[i].w_prev))

        n = 1/sum
        
        sum2 = 0
        # normalization
        for i in range(500):
            self.pf.pList[i].w_prev = self.pf.pList[i].w_prev*n
            #print("after normailization, numerator for {}th item is {}".format(i,self.pf.pList[i].w_prev))
            sum2 = sum2 + (self.pf.pList[i].w_prev)
        print("normailzied sum is {} ".format(sum2) )

    def resample(self):
        temp = []
        for i in range(500):
            temp.append(self.pf.pList[i].w_prev)

        self.pf.pList = np.random.choice(self.pf.pList, len(self.pf.pList), True, temp)
    

        for i in range(500):
            x = self.pf.pList[i].x
            y = self.pf.pList[i].y
            d = self.pf.pList[i].d
            w_prev = self.pf.pList[i].w_prev
            w = 0
            p = particle_filter.Particle(x,y,d,w_prev,w)
            self.pf.pList[i] = p
            #print("selected particle in resample function -- x:{}, y:{}, w_prev:{}, w:{}".format(x,y,w_prev,w))


    def testFinish(self):
        xloc = []
        yloc = []
        dloc = []
        for i in range(500):
            xloc.append(self.pf.pList[i].x)
            yloc.append(self.pf.pList[i].y)
            dloc.append(self.pf.pList[i].d)

        stdx = np.std(xloc)
        stdy = np.std(yloc)
        stdd = np.std(dloc)
        print("standard deviation is stdx: {} and stdy: {} and stdd: {}".format(stdx, stdy, stdd))

        if stdx < 0.05 and stdy < 0.05 and stdd < 0.1:
            xfinal = np.mean(xloc)
            yfinal = np.mean(yloc)
            dfinal = np.mean(dloc)
            print("Robot location detected at x: {} and y: {} and d: {}".format(xfinal, yfinal, dfinal))
            return True
        return False


    def updateVREP(self):
        data = []
        for i in range(500):
            p = self.pf.pList[i]
            data.append(p.x)
            data.append(p.y)
            data.append(0.1)
            data.append(p.d)

        self.virtual_create.set_point_cloud(data)
        self.time.sleep(0.1)



