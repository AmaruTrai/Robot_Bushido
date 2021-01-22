import vrep
import math
import time
import array
import PIL.Image
import numpy
import cv2
import socket
import struct
from tkinter import *
import re
from tkinter import messagebox
import multiprocessing as mult
import Robot
import Map_reader


class Settings():
    def __init__(self):
        self.defaultIP = '127.0.0.1'
        self.defaultPort = 19997
        self.Start_point = [30,99]
        self.Stop_point = [45,0]
        self.map_name = 'map1.bmp'
        self.ClientID = -1
        self.IP = self.defaultIP
        self.Port = self.defaultPort
        self.Command_for_maze = []
        self.Target_points = [3.5,2.5]
        self.map_size = [4,8]
        self.Start_position = [0.5,0.5]
        self.flag = 0

    def set_default(self):
        self.IP = self.defaultIP
        self.Port = self.defaultPort


# Graphical user interface
class GUI():
    # Init function
    def __init__(self):

        self.settings = Settings()
        self.run_flag = 0

        # Windows configuration
        self.window = Tk()
        self.window.title('Bushido v2')
        self.window.geometry('250x260')
        self.window.configure(background='white')


        # All active frame configuration
        self.flabel_ip = LabelFrame(self.window,text='V-rep IP',relief=GROOVE,bg ='white')
        self.flabel_ip.place( height=50, width=210, x=20,y=10)
        self.flabel_port = LabelFrame(self.window,text='V-rep Port',relief=GROOVE,bg ='white')
        self.flabel_port.place( height=50, width=210, x=20,y=65)

        # Label in config frame
        self.label_ip = Entry(self.flabel_ip, bg ='white', relief = FLAT, validate="key", font = (12))
        self.label_ip.insert(INSERT, self.settings.IP)
        self.label_ip['validatecommand'] = (self.label_ip.register(self.onValidate),'%S','%P',len(self.settings.IP))
        self.label_ip.place(x=75,y=-3, height=30, width=100)

        self.label_port = Entry(self.flabel_port, bg ='white', relief = FLAT, validate="key",font = (12))
        self.label_port.insert(INSERT, str(self.settings.Port))
        self.label_port['validatecommand'] = (self.label_port.register(self.onValidate),'%S','%P', 5)
        self.label_port.place(x=90,y=-3, height=30, width=100)

        # All active button configuration
        self.change_button = Button(self.window, bg ='white', text = 'Change settings', command = self.change_config)
        self.change_button.place(x=20,y=160, height=30, width=100)
        self.start_button = Button(self.window, bg ='white', text = 'Start', command = self.start)
        self.start_button.place(x=20,y=195, height=30, width=100)
        self.default_button = Button(self.window, bg ='white', text = 'Default settings', command = self.set_default)
        self.default_button.place(x=130,y=160, height=30, width=100)
        self.stop_button = Button(self.window, bg ='white', text = 'Stop', command = self.stop)
        self.stop_button.place(x=130,y=195, height=30, width=100)
        

        # All active radio button configuration
        self.var = IntVar()
        self.var.set(0)
        self.R1 = Radiobutton(self.window, text="Line", variable=self.var, value=1,  bg ='white')
        self.R1.place( height = 30, width=70, x=15,y=120)
        self.R2 = Radiobutton(self.window, text="Labyrinth", variable=self.var, value=2,  bg ='white')
        self.R2.place( height = 30, width=70, x=85,y=120)
        self.R3 = Radiobutton(self.window, text="Сraters", variable=self.var, value=3,  bg ='white')
        self.R3.place( height = 30, width=70, x=160,y=120)
       
        self.window.after(1,self.updates())
        self.window.mainloop()

    # Input validation function
    def onValidate(self,S,P, maxlen):
        if ['0','1','2','3','4','5','6','7','8','9','.','\b'].count(S) == 1 and len(P) <= int(maxlen):
            return True
        else:
            return False

    # Return to initial parameters
    def set_default(self):
        self.settings.set_default()
        self.label_ip['validate'] = NONE
        self.label_ip.delete('0',END)
        self.label_ip.insert(INSERT, self.settings.defaultIP)
        self.label_ip['validate'] = 'key'

        self.label_port['validate'] = NONE
        self.label_port.delete('0',END)
        self.label_port.insert(INSERT, self.settings.defaultPort)
        self.label_port['validate'] = 'key'

    # Configuration function    
    def change_config(self):
        IP = self.label_ip.get()
        port = self.label_port.get()
        if re.match(r'\d\d\d[.]\d[.]\d[.]\d',IP):
            self.settings.IP = IP
            mess = 'V-rep IP is configured\n'
        else:
            mess = 'Invalid V-rep IP format\n'
        if re.findall(r'[.]',port) == []:
            if int(port) <= 65535:
                self.settings.Port = int(port)
                mess = mess + 'V-rep port is configured\n'
            else:
                mess = mess + 'Invalid V-rep port value\n'
        else:
            mess = mess + 'Invalid V-rep port format\n'
        messagebox.showinfo('',mess)

    # Stop function, terminate process
    def stop(self):
        if  self.run_flag == 0:
            messagebox.showinfo('','Process is down')
        else:
            self.run_flag = 0
            self.proc_serv.terminate()
            vrep.simxStopSimulation(self.ClientID,vrep.simx_opmode_blocking)
            vrep.simxFinish(self.settings.ClientID)
            messagebox.showinfo('','Process is stoped')

    # Server status update function
    def updates(self):
        while True:
            self.window.update()

    # Start function
    def start(self):
        if  self.run_flag == 1:
            messagebox.showinfo('','Samurai is already running')
        else:
            var = self.var.get()
            if var != 1 and var != 2 and var !=3:
                messagebox.showinfo('','Please select a mode')
            else:
                self.settings.ClientID = vrep.simxStart(self.settings.IP,self.settings.Port,True,True,15000,5)
                if self.settings.ClientID == -1:
                    messagebox.showinfo('','The connection to the server was not possible')
                elif var == 1:
                    messagebox.showinfo('','Line motion initialized')
                    self.run_flag = 1
                    liner = Robot.Liner(self.settings)
                    self.proc_serv = mult.Process(target=liner.main(), name = 'Liner Process', args =())
                    self.proc_serv.start()
                elif var == 2:
                    self.run_flag = 1
                    window2 = Window_for_map(self.settings)
                    if self.settings.flag == 1:
                        map  = Map_reader.Map_reader(self.settings)
                        self.settings.Command_for_maze = map.make_command()
                        Maze_runner = Robot.Maze_runner(self.settings)
                        messagebox.showinfo('','Labyrinth motion initialized')
                        self.proc_serv = mult.Process(target=Maze_runner.main(), name = 'Maze_runner Process', args =())
                        self.proc_serv.start()
                    self.settings.flag = 0
                elif var == 3:
                    self.run_flag = 1
                    self.window3 = Window_for_collector(self.settings)
                    print(self.settings.flag)
                    if self.settings.flag == 1:
                        Collector = Robot.Collector(self.settings)
                        messagebox.showinfo('','Collector motion initialized')
                        self.proc_serv = mult.Process(target=Collector.main(), name = 'Collector Process', args =())
                        self.proc_serv.start()
                    self.settings.flag = 0
                self.run_flag = 0


            vrep.simxFinish(self.settings.ClientID)



class Window_for_map():
    def __init__(self,settings):
        self.settings = settings
        self.window = Toplevel()
        self.window.title('Maze_runner configuration')
        self.window.geometry('250x250')
        self.window.configure(background='white')

        # Active frame configuration for window 2
        self.flabel_map = LabelFrame(self.window,text='Map name',relief=GROOVE,bg ='white')
        self.flabel_map.place( height=50, width=210, x=20,y=10)
        self.flabel_start = LabelFrame(self.window,text='Start point',relief=GROOVE,bg ='white')
        self.flabel_start.place( height=50, width=210, x=20,y=65)
        self.flabel_stop = LabelFrame(self.window,text='Stop point', relief=GROOVE, bg ='white')
        self.flabel_stop.place( height=50, width=210, x=20,y=115)

        # Label in config frame
        self.label_map = Entry(self.flabel_map, bg ='white', relief = FLAT, validate="key", font = (12))
        self.label_map.insert(INSERT, self.settings.map_name)
        self.label_map.place(x=75,y=-3, height=30, width=100)

        self.label_start = Entry(self.flabel_start, bg ='white', relief = FLAT, validate="key",font = (12))
        self.label_start.insert(INSERT, str(self.settings.Start_point))
        self.label_start['validatecommand'] = (self.label_start.register(self.onValidate),'%S','%P', len(str(self.settings.Start_point))+5)
        self.label_start.place(x=90,y=-3, height=30, width=100)

        self.label_stop = Entry(self.flabel_stop, bg ='white', relief = FLAT, validate="key",font = (12))
        self.label_stop.insert(INSERT, str(self.settings.Stop_point))
        self.label_stop['validatecommand'] = (self.label_start.register(self.onValidate),'%S','%P', len(str(self.settings.Stop_point))+5)
        self.label_stop.place(x=90,y=-3, height=30, width=100)

        #Active button configuration
        self.change_button = Button(self.window, bg ='white', text = 'Accept', command = self.accept_map)
        self.change_button.place(x=75,y=180, height=30, width=100)

        self.window.after(1 ,self.window.mainloop())

    # Input validation function
    def onValidate(self,S,P, maxlen):
        if ['0','1','2','3','4','5','6','7','8','9','.','\b'].count(S) == 1 and len(P) <= int(maxlen):
            return True
        else:
            return False

    def accept_map(self):
        name = self.label_map.get()
        start = self.label_start.get()
        stop = self.label_stop.get()
        start = re.findall(r'\d+',start)
        stop = re.findall(r'\d+',stop)
        format = re.findall(r'[.]bmp',name)
        if len(start) != 2 or len(stop) != 2:
            mess = 'Wrong format for point'
            self.settings.flag = 0
        elif len(format) != 1:
            mess = 'Please, choose bmp format image'
            self.settings.flag = 0
        else:
            self.settings.Start_point = [int(start[0]),int(start[1])]
            self.settings.Stop_point = [int(stop[0]),int(stop[1])]
            self.settings.map_name = name
            mess = 'Start and stop point is configured\n' + 'Name accept'
            self.settings.flag = 1
        messagebox.showinfo('',mess )
        self.window.quit()
        self.window.destroy()

class Window_for_collector():
    def __init__(self,settings):
        self.target_buff = []

        self.settings = settings
        self.window = Toplevel()
        self.window.title('Maze_runner configuration')
        self.window.geometry('250x200')
        self.window.configure(background='white')

        # Active frame configuration for window 2
        self.flabel_map = LabelFrame(self.window,text='Map size',relief=GROOVE,bg ='white')
        self.flabel_map.place( height=50, width=100, x=20,y=10)
        self.flabel_start = LabelFrame(self.window,text='Start position',relief=GROOVE,bg ='white')
        self.flabel_start.place( height=50, width=100, x=20,y=65)
        self.flabel_stop = LabelFrame(self.window,text='Target position', relief=GROOVE, bg ='white')
        self.flabel_stop.place( height=50, width=100, x=20,y=120)

        # Label in config frame
        self.label_map = Entry(self.flabel_map, bg ='white', relief = FLAT, validate="key", font = (12))
        self.label_map.insert(INSERT, str(self.settings.map_size) )
        self.label_map['validatecommand'] = (self.label_map.register(self.onValidate),'%S','%P', len(str(self.settings.map_size))+5)
        self.label_map.place(x=30,y=-3, height=30, width=50)

        self.label_start = Entry(self.flabel_start, bg ='white', relief = FLAT, validate="key",font = (12))
        self.label_start.insert(INSERT, str(self.settings.Start_position))
        self.label_start['validatecommand'] = (self.label_start.register(self.onValidate),'%S','%P', len(str(self.settings.Start_point))+5)
        self.label_start.place(x=20,y=-3, height=30, width=70)

        self.label_stop = Entry(self.flabel_stop, bg ='white', relief = FLAT, validate="key",font = (12))
        self.label_stop.insert(INSERT, str(self.settings.Target_points))
        self.label_stop['validatecommand'] = (self.label_start.register(self.onValidate),'%S','%P', len(str(self.settings.Stop_point))+5)
        self.label_stop.place(x=20,y=-3, height=30, width=70)

        #Active button configuration
        self.change_button = Button(self.window, bg ='white', text = 'Accept', command = self.accept)
        self.change_button.place(x=140,y=20, height=30, width=100)

        self.change_button = Button(self.window, bg ='white', text = 'Add new point', command = self.add)
        self.change_button.place(x=140,y=80, height=30, width=100)

        self.change_button = Button(self.window, bg ='white', text = 'Delet last point', command = self.delet_last)
        self.change_button.place(x=140,y=135, height=30, width=100)

        self.window.after(1 ,self.window.mainloop())

    # Input validation function
    def onValidate(self,S,P, maxlen):
        if ['0','1','2','3','4','5','6','7','8','9','.','\b'].count(S) == 1 and len(P) <= int(maxlen):
            return True
        else:
            return False

    def accept(self):
        size = self.label_map.get()
        start = self.label_start.get()
        start = re.findall(r'[.\d]+',start)
        size = re.findall(r'\d+',size)
        if len(start) != 2:
            mess = 'Wrong format for point'
            self.settings.flag = 0
        elif len(size) != 2:
            mess = 'Wrong format for size'
            self.settings.flag = 0
        elif len(self.target_buff) == 0:
            mess = 'Target point buffr is empty'
            self.settings.flag = 0
        else:
            self.settings.Start_position = [float(start[0]),float(start[1])]
            self.settings.map_size = [int(size[0]),int(size[1])]
            self.settings.Target_points = self.target_buff
            mess = 'Start point and size is configured\n'
            self.settings.flag = 1
        messagebox.showinfo('',mess )
        self.window.quit()
        self.window.destroy()

    def add(self):
        point = self.label_stop.get()
        point = re.findall(r'[.\d]+',point)
        if len(point) != 2:
            mess = 'Wrong format for target point'
        elif len(self.target_buff) > 3:
            mess = 'Too much target points'
        else:
            mess = 'Target point append'
            self.target_buff.append([float(point[0]),float(point[1])])
        messagebox.showinfo('',mess)


    def delet_last(self):
        if len(self.target_buff) != 0:
            self.target_buff = self.target_buff[:len(self.target_buff)-2]
            mess = 'Target point deleted'
        else:
            mess = 'Target point buffr is empty'
        messagebox.showinfo('',mess)





