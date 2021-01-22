import vrep
import math
import time
import array
import PIL.Image
import numpy
import cv2

# General class
class Robot():
    # Initialization
    def __init__(self,Settings):
        # Common parameters
        self.ClientID = Settings.ClientID
        self.Rk = 0.04
        self.l = 0.16

        #Initialization parameters
        State = [0]*19
        reply = -1

        # Initialization of handles
        State[0],self.Camera = vrep.simxGetObjectHandle(self.ClientID,  'Midl_camera', vrep.simx_opmode_blocking)
        State[1],self.Right = vrep.simxGetObjectHandle(self.ClientID,  'Right', vrep.simx_opmode_blocking)
        State[2], self.Right0 = vrep.simxGetObjectHandle(self.ClientID,  'Right0', vrep.simx_opmode_blocking)
        State[3], self.Right1 = vrep.simxGetObjectHandle(self.ClientID,  'Right1', vrep.simx_opmode_blocking)
        State[4],self.Left = vrep.simxGetObjectHandle(self.ClientID,  'Left', vrep.simx_opmode_blocking)
        State[5],self.Left0 = vrep.simxGetObjectHandle(self.ClientID,  'Left0', vrep.simx_opmode_blocking)
        State[6],self.Left1 = vrep.simxGetObjectHandle(self.ClientID,  'Left1', vrep.simx_opmode_blocking)
        State[7],self.Midl = vrep.simxGetObjectHandle(self.ClientID,  'Midl', vrep.simx_opmode_blocking)
        State[8],self.Midl_sector = vrep.simxGetObjectHandle(self.ClientID,  'Midl_sector', vrep.simx_opmode_blocking)
        State[9],self.line1 =vrep.simxGetObjectHandle(self.ClientID,  'Line_sens1', vrep.simx_opmode_blocking)
        State[10],self.line2 =vrep.simxGetObjectHandle(self.ClientID,  'Line_sens2', vrep.simx_opmode_blocking)
        State[11],self.J1 = vrep.simxGetObjectHandle(self.ClientID,  'J1', vrep.simx_opmode_blocking)
        State[12],self.J2 = vrep.simxGetObjectHandle(self.ClientID,  'J2', vrep.simx_opmode_blocking)
        State[13],self.CJ1 = vrep.simxGetObjectHandle(self.ClientID,  'Claw1J', vrep.simx_opmode_blocking)
        State[14],self.CJ2 = vrep.simxGetObjectHandle(self.ClientID,  'Claw2J', vrep.simx_opmode_blocking)
        State[15],self.CD1 = vrep.simxGetObjectHandle(self.ClientID,  'down4', vrep.simx_opmode_blocking)
        State[16],self.CD2 = vrep.simxGetObjectHandle(self.ClientID,  'down5', vrep.simx_opmode_blocking)
        State[17],self.G = vrep.simxGetObjectHandle(self.ClientID,  'Gate', vrep.simx_opmode_blocking)
        State[18],self.Load = vrep.simxGetObjectHandle(self.ClientID,  'Loading', vrep.simx_opmode_blocking)

        # Enable data streaming
        state, R, self.image =  vrep.simxGetVisionSensorImage(self.ClientID, self.Camera, 0, vrep.simx_opmode_streaming)
        state,self.J1_pos = vrep.simxGetJointPosition(self.ClientID, self.J1, vrep.simx_opmode_streaming)
        state,self.J2_pos = vrep.simxGetJointPosition(self.ClientID, self.J2, vrep.simx_opmode_streaming)
        state, R, self.image1 = vrep.simxGetVisionSensorImage(self.ClientID, self.line1, 1, vrep.simx_opmode_streaming)
        state, R, self.image2 = vrep.simxGetVisionSensorImage(self.ClientID, self.line2, 1, vrep.simx_opmode_streaming)

        self.stop()
        if State.count(vrep.simx_return_ok) == len(State):
            reply = 1
        return reply

    # Return position function
    def get_position(self):
        reply = -1
        State1, self.J1_pos = vrep.simxGetJointPosition(self.ClientID, self.J1, vrep.simx_opmode_buffer)
        State2, self.J2_pos = vrep.simxGetJointPosition(self.ClientID, self.J2, vrep.simx_opmode_buffer)
        if State1 == vrep.simx_return_ok and State2 == vrep.simx_return_ok:
            reply = 1
        return reply

    # Return proximity function
    def get_proximity(self):
        State = [0]*9
        reply = -1
        State[0], dstate, self.Righpoint, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Right,vrep.simx_opmode_oneshot_wait)
        State[1], dstate, self.Leftpoint, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Left,vrep.simx_opmode_oneshot_wait)
        State[2], dstate, self.Leftpoint0, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Left0,vrep.simx_opmode_oneshot_wait)
        State[3], dstate, self.Leftpoint1, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Left1,vrep.simx_opmode_oneshot_wait)
        State[4], dstate, self.Midlpoint, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Midl,vrep.simx_opmode_oneshot_wait)
        State[5], dstate, self.Midl_sectorpoint, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Midl_sector,vrep.simx_opmode_oneshot_wait)
        State[6], dstate, self.Righpoint1, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Right1,vrep.simx_opmode_oneshot_wait)
        State[7], dstate, self.Righpoint0, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Right0,vrep.simx_opmode_oneshot_wait)
        State[8], dstate, self.Loadpoint, obj, nv = vrep.simxReadProximitySensor(self.ClientID, self.Load,vrep.simx_opmode_oneshot_wait)
        if State.count(vrep.simx_return_ok) == len(State):
            reply = 1
        return reply

    # Return image from line sensor function
    def get_line(self):
        reply = -1
        State1, R, self.image1 = vrep.simxGetVisionSensorImage(self.ClientID, self.line1, 1, vrep.simx_opmode_buffer)
        State2, R, self.image2 = vrep.simxGetVisionSensorImage(self.ClientID, self.line2, 1, vrep.simx_opmode_buffer)
        if State1 == vrep.simx_return_ok and State2 == vrep.simx_return_ok:
            reply = 1
        return reply

    # Return image from camera function
    def get_image(self):
        reply = -1
        State, R, image =  vrep.simxGetVisionSensorImage(self.ClientID, self.Camera, 0, vrep.simx_opmode_buffer)
        if State == vrep.simx_return_ok:
            image.reverse()
            imb=[]
            for i in range(len(image)):
                if image[i] <0:
                    imb.append(image[i]+256)
                else:
                    imb.append(image[i])
            image_byffer = PIL.Image.frombuffer("RGB", (R[0],R[1]), bytes(imb), "raw", "RGB", 0, 1)
            self.image = numpy.asarray(image_byffer)
            reply = 1
        return reply

    # Joint stop function
    def stop(self):
        reply = -1
        State1 = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, 0,vrep.simx_opmode_oneshot)
        State2 = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, 0,vrep.simx_opmode_oneshot)
        if State1 == vrep.simx_return_ok and State2 == vrep.simx_return_ok:
            reply = 1
        return reply

    # Main function
    def main(self):
        pass

# Сlass for the robot passing along the line
class Liner(Robot):

    # Initialization
    def __init__(self,Settings):
        super().__init__(Settings)

    # Obstacle avoidance function
    def rotate(self, mod):
        state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, 1*mod,vrep.simx_opmode_oneshot)
        state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, -1*mod,vrep.simx_opmode_oneshot)
        a = 0
        while a == 0:
            self.get_proximity()
            if 0.01 < self.Righpoint[2] < 0.5 or 0.01 < self.Leftpoint[2] < 0.5:
                a = 1
        if mod == 1:
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, 0.9,vrep.simx_opmode_oneshot)
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, 1.4,vrep.simx_opmode_oneshot)
        else:
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, 1.4,vrep.simx_opmode_oneshot)
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, 0.9,vrep.simx_opmode_oneshot)
        time.sleep(1)
        while a == 1:
            if self.get_line():
                u1 = self.image1.count(23) + self.image1.count(0)
                u2 = self.image2.count(23) + self.image2.count(0)
                if u1 != 0 and u2 !=0:
                    a = 0

    # Main function
    def main(self):
        vrep.simxStopSimulation(self.ClientID,vrep.simx_opmode_blocking)
        time.sleep(2)
        vrep.simxStartSimulation(self.ClientID,vrep.simx_opmode_blocking)
        mod = 1
        last_mod = mod
        Vnom = 2.8
        Stop_flag = False
        flag = False
        V1 = 0
        V2 = 0
        Kp = 1.4
        flag1 = 0
        while Stop_flag != True:
            self.get_proximity()
            if 0.01 < self.Midl_sectorpoint[2] < 0.25:
                self.rotate(mod)
                flag1 = 1
                last_mod = mod
                flag = True
                mod = -mod
            if self.get_line() == 1:
                u1 = self.image1.count(23) + self.image1.count(0)
                u2 = self.image2.count(23) + self.image2.count(0)
                alfa = u2 -u1
                if alfa == 0 and u1 != 0 and u2 != 0 and flag1 == 1:
                    alfa = 64*3.7*last_mod
                    flag1 == 0
                Vj1 = Vnom + alfa*Vnom*1.5625*0.01*0.6
                Vj2 = Vnom - alfa*Vnom*1.5625*0.01*0.6
                e1 = Vj1 - V1
                e2 = Vj2 - V2
                V1 = V1 + Kp*e1
                V2 = V2 + Kp*e2
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, V1,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, V2,vrep.simx_opmode_oneshot)
                if (self.image1.count(95) > 5 or self.image2.count(95) > 5) and flag == True:
                    time.sleep(0.4)
                    Stop_flag = True
                    self.stop()






# Class to control the robot during the passage of the maze
class Maze_runner(Robot):

    # Initialization
    def __init__(self,Settings):
        super().__init__(Settings)
        self.Command = Settings.Command_for_maze

    # Main function
    def main(self):
        vrep.simxStopSimulation(self.ClientID,vrep.simx_opmode_blocking)
        time.sleep(2)
        vrep.simxStartSimulation(self.ClientID,vrep.simx_opmode_blocking)
        state = [0,0,0]
        i = -1
        Stop_flag = False
        V1 = 0
        V2 = 0
        Kp = 4
        state_last = [0,0,0]
        rotateF = 0
        flagn = 0
        while self.get_position() != 1 and self.get_proximity() !=1 and self.get_line() !=1:
            pass
        while Stop_flag != True:
            self.get_proximity()
            self.get_line()
            if 0.001 < self.Righpoint1[2] < 0.25:
                state[0] = 1
            else:
                state[0] = 0
            if 0.001 < self.Midl_sectorpoint[2] < 0.3:
                state[1] = 1
            else:
                state[1] = 0
            if 0.001 < self.Leftpoint[2] < 0.25:
                state[2] = 1
            else:
                state[2] = 0

            if rotateF == 1 and state == self.Command[1][i+1] and state == state_last:
                self.get_position()
                rotateF = 0
                last_J1_pos = self.J1_pos
                last_J2_pos = self.J2_pos
                i = i+1
                course = 0
            state_last = state.copy()

            if self.Command[0][i] == 'Вперед':
                rotateF = 1
                if 0.1 < self.Midlpoint[2] < 0.7:
                    e1 = (self.Midlpoint[2] - 0.5)
                    e2 = (self.Midlpoint[2] - 0.5)
                    flagn = 1
                else:
                    e1 = 0.2
                    e2 = 0.2
                ea = self.Righpoint[2] - self.Righpoint0[2]
                if math.fabs(ea) > 0.0005:
                    e1 = e1 - ea/math.fabs(ea) * V1/23
                    e2 = e2 + ea/math.fabs(ea) * V2/23
                    flagn = 1
                ea = self.Leftpoint[2] - self.Leftpoint0[2]
                if math.fabs(ea) > 0.0005:
                    e1 = e1 + ea/math.fabs(ea) * V1/23
                    e2 = e2 - ea/math.fabs(ea) * V2/23
                    flagn = 1
                if flagn != 1:
                    e1 = 1
                    e2 = 1
                flagn = 0
            if self.Command[0][i] == 'Направо':
                self.get_position()
                delta_J1 = self.J1_pos - last_J1_pos
                delta_J2 = self.J2_pos - last_J2_pos
                delta_course = math.degrees((delta_J2-delta_J1)*self.Rk / self.l)
                course =  course + delta_course
                last_J1_pos = self.J1_pos
                last_J2_pos = self.J2_pos
                e = -90 - course
                e1 = - e
                e2 = + e
                if math.fabs(e) < 15:
                    rotateF = 1
            if self.Command[0][i] == 'Налево':
                self.get_position()
                delta_J1 = self.J1_pos - last_J1_pos
                delta_J2 = self.J2_pos - last_J2_pos
                delta_course = math.degrees((delta_J2-delta_J1)*self.Rk / self.l)
                course =  course + delta_course
                last_J1_pos = self.J1_pos
                last_J2_pos = self.J2_pos
                e = 90 - course
                e1 = - e
                e2 = + e
                if math.fabs(e) < 15:
                    rotateF = 1
            V1 = V1 + Kp*e1
            V2 = V2 + Kp*e2
            if V1 > 5.5:
                V1 = 5.5
            if V2 > 5.5:
                V2 = 5.5
            if V1 < 1.5:
                V1 = 1.5
            if V2 < 1.5:
                V2 = 1.5
            state_last = [state[0],state[1],state[2]]

            if (self.image1.count(23) + self.image2.count(23)) > 40:
                Stop_flag = True
                time.sleep(2)
                V1 = 0
                V2 = 0
            state1 = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, V1,vrep.simx_opmode_oneshot)
            state2 = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, V2,vrep.simx_opmode_oneshot)
            print(self.Command[1][i],self.Command[0][i], V1, V2)


class Collector(Robot):

    # Initialization
    def __init__(self,Settings):
        super().__init__(Settings)
        self.color_maxB = (700,70,70)
        self.color_minB = (100,0,0)
        self.color_maxG = (70,700,70)
        self.color_minG = (0,100,0)
        self.start_position = Settings.Start_position
        self.target_points = Settings.Target_points
        self.position = Settings.Start_position
        self.map_size =  Settings.map_size
        self.way = [0]*len(self.target_points)*2
        self.way_flag = []
        self.robot_course = 0

    # Grip opening function
    def open_claw(self):
        state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD1, 0.015,vrep.simx_opmode_oneshot)
        state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD2, 0.015,vrep.simx_opmode_oneshot)
        time.sleep(0.1)
        state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ2, math.radians(20),vrep.simx_opmode_oneshot)
        state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ1, math.radians(-20),vrep.simx_opmode_oneshot)

    # Grip closing function
    def close_claw(self):
        Grip_flag = False
        End_flag = 0
        while End_flag != 4:
            if self.get_proximity() == 1:
                if self.Loadpoint[2] > 0.000001:
                    Grip_flag = True
            if End_flag == 0:
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.G, math.radians(-90),vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ2, math.radians(-30),vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ1, math.radians(30),vrep.simx_opmode_oneshot)
                timer = time.time() + 1
                End_flag = End_flag + 1
            elif End_flag == 1 and (timer - time.time()) <= 0:
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD1, -0.04,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD2, -0.04,vrep.simx_opmode_oneshot)
                timer = time.time() + 2
                End_flag = End_flag + 1
            elif End_flag == 2 and (timer - time.time())  <= 0:
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ2, math.radians(-90),vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CJ1, math.radians(90),vrep.simx_opmode_oneshot)
                timer = time.time() + 2
                End_flag = End_flag + 1
            elif End_flag == 3 and (timer - time.time())  <= 0:
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD1, -0.02,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.CD2, -0.02,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetPosition(self.ClientID, self.G, math.radians(90),vrep.simx_opmode_oneshot)
                End_flag = End_flag + 1
        return Grip_flag



    # Move to S function
    def run(self, S, Vnom = 1, Vmin = 0.2):
        e = S
        V = Vnom
        Kp = 0.12
        way = 0
        while self.get_position() != 1:
            pass
        last_J1_pos = self.J1_pos
        last_J2_pos = self.J2_pos
        while e > 0.001:
            self.get_proximity()
            if 0.001 < self.Midlpoint[2] < 0.135:
                e = 0
            self.get_position()
            delta_J1 = self.J1_pos - last_J1_pos
            delta_J2 = self.J2_pos - last_J2_pos
            delta_way = (delta_J2+delta_J1)*self.Rk /2
            way = way + delta_way
            e = S - way
            last_J1_pos = self.J1_pos
            last_J2_pos = self.J2_pos
            V = V - Kp/math.fabs(e)*0.01
            if V < Vmin:
                V = Vmin
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, V,vrep.simx_opmode_oneshot)
            state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, V,vrep.simx_opmode_oneshot)
        self.stop()

    # Rotate to angl function
    def rotate(self, angl, Vnom = 1, Vmin = 0.2):
        V = Vnom
        Kp = 0.9
        e = angl
        course = 0
        while self.get_position() != 1:
            pass
        last_J1_pos = self.J1_pos
        last_J2_pos = self.J2_pos
        while math.fabs(e) > 0.2:
            self.get_position()
            delta_J1 = self.J1_pos - last_J1_pos
            delta_J2 = self.J2_pos - last_J2_pos
            delta_course = math.degrees((delta_J2-delta_J1)*self.Rk / self.l)
            course =  course + delta_course
            last_J1_pos = self.J1_pos
            last_J2_pos = self.J2_pos
            e = angl - course
            if e > 0:
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, -V,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, V,vrep.simx_opmode_oneshot)
            else:
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J1, V,vrep.simx_opmode_oneshot)
                state = vrep.simxSetJointTargetVelocity(self.ClientID, self.J2, -V,vrep.simx_opmode_oneshot)
            V = V - Kp/math.fabs(e)*0.01
            if V < Vmin:
                V = Vmin
        self.stop()
        return course



    # The search and capture function in the crater
    def find(self):
        Stop_flag = False
        Grip_flag = 0
        while Stop_flag != True:
            if self.get_image() == 1 and self.get_proximity() == 1:
                brg = cv2.inRange(self.image[50:], self.color_minB, self.color_maxB)
                moments = cv2.moments(brg,1)
                if moments['m00']  > 0:
                    if Grip_flag == 0:
                        self.open_claw()
                        Grip_flag = 1
                    if moments['m00'] > 6000 and Grip_flag !=0:
                        Stop_flag = self.close_claw()
                        Grip_flag = 0
                    centr = moments['m10']/moments['m00']
                    target = centr - 64
                    if math.fabs(target) > 3:
                        self.robot_course = self.robot_course + self.rotate(2 *target/math.fabs(target))
                    elif Stop_flag != True:
                        self.run(0.05)
                else:
                    self.robot_course = self.robot_course + self.rotate(20, 2)

    # Distance measurement function
    def distance(self, color):
        flag = 0
        distance = []
        angl = 0
        if color == 'G':
            color_max = self.color_maxG
            color_min = self.color_minG
            Color_flag= 1
        elif color == 'B':
            color_max = self.color_maxB
            color_min = self.color_minB
            Color_flag= 0
        elif color == 'R':
            color_max = self.color_maxR
            color_min = self.color_minR
            Color_flag = 1
        elif color == 'Y':
            color_maxY = self.color_maxY
            color_minY = self.color_minY
            Color_flag = 0
        while flag != 3:
            minp = []
            maxp = []
            if self.get_image() == 1:
                brg = cv2.inRange(self.image[0:80], color_min, color_max)
                if flag != 2:
                    list_brg = brg.tolist()
                    for i in range(len(list_brg)):
                        try:
                            minp.append(list_brg[i].index(255))
                            list_brg[i].reverse()
                            maxp.append(list_brg[i].index(255))
                        except:
                            pass
                    minp.sort()
                    maxp.sort()
                    try:
                        min = minp[0]
                        max = maxp[0]
                    except:
                        min = -1
                        max = -1
                    if flag == 0:
                        target = -(64 - min)
                    elif flag == 1:
                        target = (64 - max)
                    if math.fabs(target) > 0.5:
                        if math.fabs(target) < 2:
                            anglr = self.rotate(target, 0.3)
                        else:
                            anglr = self.rotate(target/2, 0.3)
                        if flag == 1:
                            angl = angl + anglr 
                    else:
                        self.get_proximity()
                        flag = flag + 1
                        distance.append( self.Midlpoint[2])
                elif flag == 2:
                    if len(self.map_size) < 2:
                        c = (distance[0]**2+distance[1]**2-2*distance[0]*distance[1]*math.cos(math.radians(angl)))**(1/2)
                        self.map_size.append(c)
                    else:
                        c = self.map_size[Color_flag]
                    p = (distance[0] + c+ distance[1])/2
                    hc = 2/c * (p*(p-c)*(p-distance[0])*(p-distance[1]))**(1/2)
                    flag = 3
                hsv = cv2.resize(brg, (300,300),interpolation = cv2.INTER_AREA)
                cv2.imshow('lineM',hsv)
                cv2.waitKey(1)
        print(hc, distance)
        return hc, angl, distance

    def finde_pos(self):
        hx, angla, da = self.distance('B')
        hy, anglb, db = self.distance('G')
        self.position = [self.map_size[1] - hx,self.map_size[0] - hy]
        self.robot_course = anglb+math.degrees(math.acos(hx/da[1]))

    def navigation(self, target_points):
        dy = target_points[1]-self.position[1]
        dx = target_points[0]-self.position[0]
        target_course = math.atan2(dy,dx)
        self.robot_course = self.robot_course + self.rotate(math.degrees(target_course) - self.robot_course, 2)
        self.run((dx**2+dy**2)**(1/2), 4)

    def main(self):
        vrep.simxStopSimulation(self.ClientID,vrep.simx_opmode_blocking)
        time.sleep(2)
        vrep.simxStartSimulation(self.ClientID,vrep.simx_opmode_blocking)
        self.planing()
        self.position = self.start_position
        i = 0
        while i < len(self.way):
            self.navigation(self.way[i])
            if self.way_flag[i] == -1:
                self.find()
                self.position = self.way[i]
            else:
                self.finde_pos()
            i = i+1
        self.navigation(self.start_position)

    def planing(self):
        self.lab_map = numpy.zeros((self.map_size[0],self.map_size[1]))
        i = 0
        while i < len(self.target_points):
            x = math.floor(self.target_points[i][0])
            y = math.floor(self.target_points[i][1])
            self.lab_map[y,x] = -1
            if y+1 < len(self.lab_map):
                self.lab_map[y+1,x] = -2
                if x+1 < len(self.lab_map[i]):
                    self.lab_map[y+1,x+1] = -3
                if x-1 >= 0:
                    self.lab_map[y+1,x-1] = -3
            if y-1 >= 0:
                self.lab_map[y-1,x] = -2
                if x+1 < len(self.lab_map[i]):
                    self.lab_map[y-1,x+1] = -3
                if x-1 >= 0:
                    self.lab_map[y-1,x-1] = -3
            if x+1 < len(self.lab_map[i]):
                self.lab_map[y,x+1] = -2
            if x-1 >= 0:
                self.lab_map[y,x-1] = -2    
            i = i+1
        x = math.floor(self.position[0])
        y = math.floor(self.position[1])
        self.lab_map[y][x] = 1

        self.way = [0]*len(self.target_points)*2
        flag = -1
        p = self.way.count(0)
        while p != 0:
            i = 0
            last_dl = 100
            while i < len(self.lab_map-1):
                j = 0
                while j < len(self.lab_map[i]-1):
                    if self.lab_map[i][j] == flag:
                        dx = self.position[0] - j
                        dy = self.position[1] - i
                        dl = (dx**2+dy**2)**(1/2)
                        
                        if dl < last_dl:
                            last_dl = dl
                            self.way[len(self.way)-p] = [j,i]
                    j = j+1
                i = i+1
            self.way_flag.append(flag)
            if flag == -1:
                self.lab_map[self.way[len(self.way)-p][1]][self.way[len(self.way)-p][0]] = 1
                flag = 0
            else:
                flag = -1
            self.position = self.way[len(self.way)-p]
            p = self.way.count(0)
        i = 0
        while i < len(self.way):
            self.way[i] = [self.way[i][0]+0.5,self.way[i][1]+0.5]
            i = i+1
        
