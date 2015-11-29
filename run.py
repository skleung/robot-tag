'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information created by David Zhu

   Kre8 Technology retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of Kre8 Technology is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this file.
   ========================================================================*/
'''

import Tkinter as tk
import time
from HamsterAPI.comm_ble import RobotComm
import math
import numpy as np
import Queue
from fsm import *
import threading
import math
from tk_hamster_GUI import *

UPDATE_INTERVAL = 30

comm = None
gMaxRobotNum = 3 # max number of robots to control
# gRobotList = None
gQuit = False
m = None

collision_queue = Queue.Queue()

class VirtualWorldGui:
    def __init__(self, vWorld, m, rCanvas):
        self.vworld = vWorld
        self.m = m
        self.rCanvas = rCanvas
        self.joysticks = []
        self.gRobotList = None
        self.existingThreads = []
        self._period = 0.05
        self._freq_cutoff = 40
        self._RC = 1/(2 * math.pi * self._freq_cutoff)
        self._alpha = self._period / (self._RC + self._period)
        self._acc_x = [0]*gMaxRobotNum
        self._acc_y = [0]*gMaxRobotNum
        self._mag_diff2 = 15 * 15

        self.fsms = []

        self.button0 = tk.Button(m,text="Grid")
        self.button0.pack(side='left')
        self.button0.bind('<Button-1>', self.drawGrid)

        self.button1 = tk.Button(m,text="Clear")
        self.button1.pack(side='left')
        self.button1.bind('<Button-1>', self.clearCanvas)

        self.button2 = tk.Button(m,text="Reset")
        self.button2.pack(side='left')
        self.button2.bind('<Button-1>', self.resetvRobot)

        self.button3 = tk.Button(m,text="Map")
        self.button3.pack(side='left')
        self.button3.bind('<Button-1>', self.drawMap)

        self.button4 = tk.Button(m,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(m,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(m,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button7 = tk.Button(m,text="Localize")
        self.button7.pack(side='left')
        self.button7.bind('<Button-1>', self.localize)

        self.button8 = tk.Button(m,text="Collide")
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>', self.collide)

        self.button9 = tk.Button(m,text="FSM")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', self.fsm)

        self.button10 = tk.Button(m,text="TAG")
        self.button10.pack(side='left')
        self.button10.bind('<Button-1>', self.tag)

        self.button11 = tk.Button(m,text="Exit")
        self.button11.pack(side='left')
        self.button11.bind('<Button-1>', stopProg)

    # reset to the starting position for the beginning of tag
    def resetvRobot(self, event=None):
        coords = [(-100,-100), (-100,100), (100,100), (100,-100)]
        angles = [0, 90, 180, 270]
        self.joysticks = []
        for i, robot in enumerate(comm.robotList):
            joystick = self.create_joystick(robot)
            self.vworld.add_vrobot(joystick.vrobot)
            x, y = coords[i]
            joystick.vrobot.x = x
            joystick.vrobot.y = y
            joystick.vrobot.a = angles[i]
            self.joysticks.append(joystick)

    def toggleTrace(self, event=None):
        if self.vworld.trace:
            self.vworld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vworld.trace = True
            self.button4["text"] = "No Trace"

    def toggleProx(self, event=None):
        if self.vworld.prox_dots:
            self.vworld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vworld.prox_dots = True
            self.button5["text"] = "No Prox Dots"

    def toggleFloor(self, event=None):
        if self.vworld.floor_dots:
            self.vworld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vworld.floor_dots = True
            self.button6["text"] = "No Floor Dots"

    def drawMap(self, event=None):
        self.vworld.draw_map()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def clearCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        for vrobot in self.vworld.vrobots:
            vcanvas.delete("all")
            poly_points = [0,0,0,0,0,0,0,0]
            vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
            vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
            vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
            vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
            vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
        while (drawQueue.qsize() > 0):
            drawCommand = drawQueue.get()
            drawCommand()

    def tag(self, event=None):
        self.collide()
        self.fsm()

    def fsm(self, event=None):
        for i, robot in enumerate(comm.robotList):
            fsm = None
            if i == 0:
                fsm = StateMachine(robot, self.joysticks[i])
                fsm.queue.put("got tagged") # it 
            else:
                fsm = StateMachine(robot, self.joysticks[i])
                fsm.queue.put("tagged") # not it
            self.fsms.append(fsm)
            fsm_thread = threading.Thread(target=fsm.run)
            fsm_thread.daemon = True
            fsm_thread.start()

            thread = threading.Thread(target=collectData, args=(fsm.queue, robot))
            thread.daemon = True
            thread.start()

    def create_joystick(self, robot):
        rCanvas = self.rCanvas
        joystick = Joystick(comm, self.m, rCanvas, robot)

        # visual elements of the virtual robot
        poly_points = [0,0,0,0,0,0,0,0]
        joystick.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
        joystick.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
        joystick.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
        joystick.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
        joystick.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")

        time.sleep(0.5)

        update_vrobot_thread = threading.Thread(target=joystick.update_virtual_robot)
        update_vrobot_thread.daemon = True
        update_vrobot_thread.start()

        return joystick

    def localize(self):
        xs = []
        ys = []
        print "proximity left = ", self.vworld.vrobot.dist_l
        print "proximity right = ", self.vworld.vrobot.dist_r
        for _ in range(10):
            xs.append(2)
            ys.append(self.vworld.vrobot.dist_l)
            xs.append(38)
            ys.append(self.vworld.vrobot.dist_r)
        m, b = calculate_least_sqs(xs, ys)
        perp_dist = sum(ys)/float(len(ys)) + 20 # 20 is half of the robot's length
        angle = math.atan(m)
        # self.localize_helper(angle, perp_dist)

    def collide(self, event=None):
        if self.existingThreads:
            pass
        else:
            for i in range(len(comm.robotList)):
                sensor_thread = threading.Thread(target=self.senseCollision, args=(i,))
                sensor_thread.daemon = True
                sensor_thread.start()
                self.existingThreads.append(sensor_thread)

            detect_thread = threading.Thread(target=self.detectCollision)
            detect_thread.daemon = True
            detect_thread.start()
            self.existingThreads.append(detect_thread)

    def maintainItState(self):
        n_its = 0
        for fsm in self.fsms:
            if fsm.currentState[0:2] == "It":
                n_its += 1
        if n_its > 1:
            print "too many its"
            for fsm in self.fsms:
                fsm.queue.put("tagged")
        if n_its < 1:
            print "too few its"
            self.fsms[2].queue.put("got tagged")



    def detectCollision(self):
        time.sleep(0.5)

        while not gQuit:
            while(collision_queue.empty()):
                time.sleep(0.1)
                self.maintainItState()

            collide_index_1, ts1 =  collision_queue.get()
            self.maintainItState()    
            while(not collision_queue.empty()):
                # two independent collisions are detected
                collide_index_2, ts2 = collision_queue.get()
                if collide_index_1 != collide_index_2 and abs(ts2 - ts1) < 0.5:
                    if (self.fsms[collide_index_1].currentState[0:2] == "It" or self.fsms[collide_index_2].currentState[0:2] == "It"):
                        it_index = collide_index_1 if self.fsms[collide_index_1].currentState[0:2] == "It" else collide_index_2
                        not_it_index = collide_index_2 if self.fsms[collide_index_1].currentState[0:2] == "It" else collide_index_1
                        self.fsms[it_index].queue.put("tagged")
                        self.fsms[not_it_index].queue.put("got tagged")
                        print "tag detected", "it:", it_index, "not it:", not_it_index
                        print "successfull tag:", self.fsms[it_index].currentState[0:2] == "It"
                self.maintainItState()


    def lowpass(self, alpha, old, new):
        return alpha * new + (1.0 - alpha) * old

    def difference(self, old, new):
        return old - new

    def collision_detection(self, acc_x, acc_y, i):
        dx = self.difference(self._acc_x[i], acc_x)
        dy = self.difference(self._acc_y[i], acc_y)
        mag2 = dx*dx + dy*dy
        if (mag2 > self._mag_diff2):
            return math.sqrt(mag2)
        else:
            return -mag2

    def senseCollision(self, i):
        time.sleep(0.5)

        lastX = None
        lastY = None
        smoothingFactor = 0.8
        collisionThreshold = 700
        while not gQuit:
            x, y, z = self.joysticks[i].read_accelerometer_data()
            # if lastY != None and lastX != None:
            #     filteredY = (1-smoothingFactor)*lastY + smoothingFactor*(y)
            #     filteredX = (1-smoothingFactor)*lastX + smoothingFactor*(x)
            #     magnitude = math.sqrt(math.pow(abs(filteredY - lastY),2) + math.pow(abs(filteredX - x),2))
            #     if magnitude > collisionThreshold:
            #         collision_queue.put("collision")
            #         return
            #     lastY = filteredY
            #     lastX = filteredX
            # else:
            #     lastY = y
            #     lastX = x
            acc_x = x/100.0
            acc_y = y/100.0
            acc_x = self.lowpass(self._alpha, self._acc_x[i], acc_x)
            acc_y = self.lowpass(self._alpha, self._acc_y[i], acc_y)
            mag = self.collision_detection(acc_x, acc_y, i)
            # print "from ", str(i), " mag = ", mag
            self._acc_x[i] = acc_x
            self._acc_y[i] = acc_y
            if (mag > 0):
                collision_queue.put((i, time.time()))
                print "collision detected from "+str(i), mag
                elem = {}
                elem[0] = self._acc_x
                elem[1] = self._acc_y
            time.sleep(self._period)

PROXIMITY_THRESHOLD = 30
PROXIMITY_ERROR = 5
def collectData(queue, robot):
    while not gQuit:

        # sense floor
        floor_left = robot.get_floor(0)
        floor_right = robot.get_floor(1)
        proximity_left = robot.get_proximity(0)
        proximity_right = robot.get_proximity(1)

        if (proximity_left > PROXIMITY_THRESHOLD or proximity_right > PROXIMITY_THRESHOLD): # obj detected
            if (proximity_left > proximity_right and (proximity_left - proximity_right) > PROXIMITY_ERROR):
                queue.put("obj left")
            elif (proximity_left < proximity_right and (proximity_right - proximity_left) > PROXIMITY_ERROR):
                queue.put("obj right")
            else:
                queue.put("obj ahead")
        else:
            queue.put("clear")

        # print "floor:", floor_left, floor_right
        if (floor_left < 30):
            queue.put("floor left")
        elif (floor_right < 30):
            queue.put("floor right")
        else:
            queue.put("floor clear")

        time.sleep(0.05)

def calculate_least_sqs(xvalues, yvalues):
    x = np.array(xvalues)
    y = np.array(yvalues)
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y)[0]
    return (m, b)

class Joystick:
    def __init__(self, comm, m, rCanvas, robot):
        self.gMaxRobotNum = 1
        self.gRobotList = comm.robotList
        self.m = m
        self.robot = robot
        self.speed = 30
        self.vrobot = virtual_robot()
        self.vrobot.t = time.time()

        rCanvas.bind_all('<w>', self.move_up)
        rCanvas.bind_all('<s>', self.move_down)
        rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.move_right)
        rCanvas.bind_all('<l>', self.read_psd_distance)
        rCanvas.bind_all('<x>', self.stop_move)
        rCanvas.pack()

    # joysticking the robot
    def move_up(self, event=None):
        if self.gRobotList:
            robot = self.robot
            # print "moving up with robot = ", robot
            self.vrobot.sl = self.speed
            self.vrobot.sr = self.speed
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList:
            robot = self.robot
            self.vrobot.sl = -1*self.speed
            self.vrobot.sr = -1*self.speed
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList:
            robot = self.robot
            self.vrobot.sl = -1*self.speed/2
            self.vrobot.sr = self.speed/2
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_right(self, event=None):
        if self.gRobotList:
            robot = self.robot
            self.vrobot.sl = self.speed/2
            self.vrobot.sr = -1*self.speed/2
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def stop_move(self, event=None):
        if self.gRobotList:
            robot = self.robot
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def write_servo_position(self, robot, angle):
        robot.set_io_mode(1, 8)
        robot.set_port(1, angle)

    def read_psd_distance(self, event=None):
        if self.gRobotList:
            robot = self.robot
            print robot.get_port(0)
            return robot.get_port(0)

    def read_accelerometer_data(self):
        if self.gRobotList:
            robot = self.robot
            x = robot.get_acceleration(0)
            y = robot.get_acceleration(1)
            z = robot.get_acceleration(2)
            return (x,y,z)
        return None

    def look(self, angle):
        if self.gRobotList:
            robot = self.robot
            self.write_servo_position(robot, angle)

    def turn_clockwise(self, angle):
        if self.gRobotList:
            self.move_right()
            while(self.vrobot.a < angle):
                time.sleep(0.1)
            self.stop_move()

    def turn_counterclockwise(self, angle):
        if self.gRobotList:
            self.move_left()
            while(self.vrobot.a > angle):
                time.sleep(0.1)
            self.stop_move()

    def play_sound(self):
        robot = self.robot
        robot.set_musical_note(40)
        time.sleep(0.2)
        robot.set_musical_note(0)
        time.sleep(0.1)
        robot.set_musical_note(40)
        time.sleep(0.1)
        robot.set_musical_note(45)
        time.sleep(0.4)
        robot.set_musical_note(0)

    def update_virtual_robot(self):
        # this is the robot modeling code - below is a very simple and inaccurate
        # model, as example of how to use the GUI toolkit you need to create you
        # own model

        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.2 #proximity conversion - assuming linear
        d_factor = 1.1 #travel distance conversion
        a_factor = 16 # rotation conversion, assuming linear

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot"

        while not gQuit:
            if self.gRobotList is not None:
                robot = self.robot

                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr:
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor
                #update sensors
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                if (prox_l > noise_prox):
                    self.vrobot.prox_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False

                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False
            time.sleep(0.1)

def stopProg(event=None):
    global gQuit
    global m
    m.quit()
    gQuit = True
    print "Exit"

def draw_virtual_world(virtual_world):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if comm.robotList is not None:
            virtual_world.draw_robots()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def main(argv=None):
    global m
    global comm
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    m = tk.Tk() #root
    drawQueue = Queue.Queue(0)

    #creating tje virtual appearance of the robot
    canvas_width = 700 # half width
    canvas_height = 380 # half height
    rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)

    joystick = Joystick(comm, m, rCanvas, None)

    #create the virtual worlds that contains the virtual robot
    vWorld = virtual_world(drawQueue, rCanvas, canvas_width, canvas_height)

    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld,))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, m, rCanvas)

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    m.mainloop()


    for robot in comm.robotList:
        robot.reset()
    comm.stop()
    comm.join()


if __name__ == "__main__":
    main()