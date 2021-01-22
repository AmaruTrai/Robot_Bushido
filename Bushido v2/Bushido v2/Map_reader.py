import PIL.Image
import numpy
import cv2

# Class to build a route in the maze
class Map_reader():

    # Initialization
    def __init__(self, settings):
        self.settings = settings
        self.map_name = self.settings.map_name
        self.map = cv2.imread(self.map_name)
        self.map = cv2.inRange(self.map, (100,100,100), (300,300,300))
        self.start_point = [self.settings.Start_point[1],self.settings.Start_point[0]]
        self.stop_point = [self.settings.Stop_point[1],self.settings.Stop_point[0]]
        self.len_y = self.map.shape[0]      #Количество строк в массиве
        self.len_x = self.map.shape[1]      #Количество столбцов в массиве
        self.way = [self.stop_point]        # Буфер для пути

        self.Command = []
        self.Command_state = []
        self.find_way()


    # The function of creating a new route list
    def new_map(self, on, off):
        self.lab_map = numpy.zeros((self.map.shape[0],self.map.shape[1]))
        i = 0
        while i < len(self.map):
            j = 0
            while j < len(self.map[i]):
                if self.map[i][j] == 0:
                    self.lab_map[i][j] = off
                else:
                    self.lab_map[i][j] = on
                j = j + 1
            i = i + 1

    # The function of finding the way in the maze
    def find_way(self):
        self.new_map(-2,-1)
        i = 0
        variation = [[self.start_point]]
        self.lab_map[self.start_point[0]][self.start_point[1]] = 0
        while  i != len(variation):
            j = 0
            buff = []
            while j != len(variation[i]):
                y = variation[i][j][0]
                x = variation[i][j][1]
                if y-1 > -1 and self.lab_map[y-1][x] == -2:
                    self.lab_map[y-1][x] = i + 1
                    buff.append([y-1,x])
                if x+1 < self.len_x and self.lab_map[y][x+1] == -2:
                    self.lab_map[y][x+1] = i + 1
                    buff.append([y,x+1])
                if x-1 > -1 and self.lab_map[y][x-1] == -2:
                    self.lab_map[y][x-1] = i + 1
                    buff.append([y,x-1])
                if y+1 > self.len_y and self.lab_map[y+1][x] == -2:
                    self.lab_map[y+1][x] = i + 1 
                    buff.append([y+1,x])
                j = j + 1
            if len(buff) > 0:
                variation.append(buff)
            i = i + 1
        i = 1
        Stop_flag = False
        while Stop_flag !=  True:
            if variation[len(variation)-i].count(self.stop_point) > 0:
                Stop_flag = True
            i = i+1
        k = 0
        i = i -1
        Stop_flag = False
        while Stop_flag != True:
            flag = False
            j = 0
            i = i + 1
            while flag != True:
                y = variation[len(variation)-i][j][0]
                x = variation[len(variation)-i][j][1]
                flag1 = False
                if y - 1 == self.way[k][0] and x == self.way[k ][1] or x - 1 == self.way[k ][1] and \
                   y == self.way[k ][0] or y + 1 == self.way[k ][0] and x == self.way[k ][1] or \
                   x + 1 == self.way[k ][1] and y == self.way[k ][0]:
                    if self.lab_map[y][x] == self.lab_map[self.way[k ][0]][self.way[k ][1]]-1:
                        flag = True
                        self.way.append([y,x])
                        flag1 = True
                j = j +1
            if [y,x] == self.start_point:
                Stop_flag = True
            k = k +1
        i = 0
 
    # The construction of a sequence of commands function    
    def make_command(self):
        self.new_map(0,1)
        self.way.reverse()
        orientation = 1
        State = [0,0,0,0]
        S = [0,0,0]
        Buff_S =[[]]

        i = 0
        while i < len(self.way)-1:
            last_orientation = orientation
            xl = self.way[i+1][1] - self.way[i][1]
            yl = self.way[i+1][0] - self.way[i][0]

            if yl == -1:
                if orientation == 1:
                    state = "Вперед"
                elif orientation == 2:
                    state = "Направо"
                    orientation = 1
                elif orientation == 4:
                    state = "Налево"
                    orientation = 1
            elif yl == 1:
                if orientation == 2:
                    state = "Налево"
                    orientation = 3
                elif orientation == 3:
                    state = "Вперед"
                elif orientation == 4:
                    state = "Направо"
                    orientation = 3
            elif xl == -1:
                if orientation == 1:
                    state = "Налево"
                    orientation = 2
                elif orientation == 2:
                    state = "Вперед"
                elif orientation == 4:
                    state = "Направо"
                    orientation = 2
            elif xl == 1:
                if orientation == 1:
                    state = "Направо"
                    orientation = 4
                elif orientation == 3:
                    state = "Налево"
                    orientation = 4
                elif orientation == 4:
                    state = "Вперед"
            try:
                State[0] = self.lab_map[self.way[i][0]+1,self.way[i][1]]
                State[1] = self.lab_map[self.way[i][0],self.way[i][1]+1]
                State[2] = self.lab_map[self.way[i][0]-1,self.way[i][1]]
                State[3] = self.lab_map[self.way[i][0],self.way[i][1]-1]
            except:
                pass
            if state == "Вперед":
                if orientation == 1:
                    S = [State[1],State[2],State[3]]
                elif orientation == 2:
                    S = [State[2],State[3],State[0]]
                elif orientation == 3:
                    S = [State[3],State[0],State[1]]
                elif orientation == 4:
                    S = [State[0],State[1],State[2]]
            else:
                if last_orientation == 1:
                    S = [State[1],State[2],State[3]]
                elif last_orientation == 2:
                    S = [State[2],State[3],State[0]]
                elif last_orientation == 3:
                    S = [State[3],State[0],State[1]]
                elif last_orientation == 4:
                    S = [State[0],State[1],State[2]]
            Buff_S.append(S)
            try:
                if Buff_S[i+1] != Buff_S[i]:
                    self.Command.append(state)
                    self.Command_state.append(S)
            except:
                pass
            i = i + 1

        i = 0
        self.map = cv2.imread(self.map_name)
        while i < len(self.way):
            self.map[self.way[i][0],self.way[i][1]] = [255,0,0]
            i = i + 1

        image = cv2.resize(self.map, (300,300),interpolation = cv2.INTER_AREA)
        cv2.imshow("Original image", image)
        cv2.waitKey(1)
        return[self.Command, self.Command_state]
