import random
import yaml
from yaml import load,dump

class Point:
    def __init__(self, reachable, visited=False, timecost = 0, changeProb = 0.2):
        self.reachable = reachable
        self.visited = visited
        self.timecost = timecost
        self.changeProb = changeProb
        self.timeCap = 0
        self.ischargingp = False
    def step(self):
        e=random.random()
        if e < self.changeProb:
            self.reachable = not self.reachable

class ChargingPoint(Point):
    def __init__(self,timecost = 30, cap = 8 ):
        super(Point.__init__(reachable=True,visited=False,timecost=timecost, changeProb=0))
        self.cap = cap
        self.stop_num = 0
        self.chargingList = []
        self.ischargingp = True

class Edge:
    def __init__(self, from_p, to_p, traveltime = 1):
        self.from_p = from_p
        self.to_p = to_p
        self.traveltime = traveltime

class Env:
    def __init__(self, scale=(0, 0), initMode='Default', topology='Grid'):
        self.scale = scale
        self.Points = []
        self.Edges = []
        self.initMode = initMode
        self.topology = topology
        if self.topology == 'Grid':
            if initMode == "Default":
                self.initDefault()
            elif initMode == 'Config':
                self.initConfig()
            elif initMode == 'Random':
                self.initRandom()
            elif initMode == 'View':
                self.initView()
        else:
            self.initTopology()

    def initView(self):
        self.initDefault()

    def initDefault(self):
        x, y = self.scale
        default_file = open("./env/env_default.yaml",'r')
        config = load(default_file)
        charge_stations = config['charge_stations']
        self.stations_num = len(charge_stations)
        unreachable = config['unreachable']
        for i in range(x):
            self.Points.append([])
            for j in range(y):
                self.Points[i].append(Point(reachable=True))
        for item in charge_stations:
            if item['x'] in range(x) and item['y'] in range(y):
                self.Points[item['x']][item['y']] = ChargingPoint()
        for item in unreachable:
            if item['x'] in range(x) and item['y'] in range(y):
                self.Points[item['x']][item['y']].reachable = False
        for i in range(x*y):
            self.Edges.append([])
        for i in range(x*y):
            if i%y-1>=0:# left
                self.Edges[i].append(Edge(from_p=(i/y,i%y),to_p=(i/y,i%y-1)))
            if i%y+1<y:# right
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y, i % y + 1)))
            if i/y-1 >=0:# up
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y-1, i % y)))
            if i/y +1 <y:# down
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y+1, i % y)))

    def initConfig(self):
        x, y = self.scale
        default_file = open("./env/env_config.yaml", 'r')
        config = load(default_file)
        charge_stations = config['charge_stations']
        self.stations_num = len(charge_stations)
        unreachable = config['unreachable']
        for i in range(x):
            self.Points.append([])
            for j in range(y):
                self.Points[i].append(Point(reachable=True))
        for item in charge_stations:
            if item['x'] in range(x) and item['y'] in range(y):
                self.Points[item['x']][item['y']] = ChargingPoint()
        for item in unreachable:
            if item['x'] in range(x) and item['y'] in range(y):
                self.Points[item['x']][item['y']].reachable = False
        for i in range(x*y):
            self.Edges.append([])
        for i in range(x*y):
            if i%y-1>=0:# left
                self.Edges[i].append(Edge(from_p=(i/y,i%y),to_p=(i/y,i%y-1)))
            if i%y+1<y:# right
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y, i % y + 1)))
            if i/y-1 >=0:# up
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y-1, i % y)))
            if i/y +1 <y:# down
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y+1, i % y)))

    def initRandom(self):
        x, y = self.scale
        for i in range(x):
            self.Points.append([])
            for j in range(y):
                self.Points[i].append(Point(reachable=random.choice([True, False])))
        self.stations_num = random.randint(0,x+1)
        temp_stat_list = []
        for i in range(self.stations_num):
            rand_pos = random.randint(0, x*y)
            while rand_pos in temp_stat_list:
                rand_pos = random.randint(0, x*y)
            self.Points[rand_pos/y][rand_pos%y]=ChargingPoint()
            temp_stat_list.append(rand_pos)
        for i in range(x*y):
            self.Edges.append([])
        for i in range(x*y):
            if i%y-1>=0:# left
                self.Edges[i].append(Edge(from_p=(i/y,i%y),to_p=(i/y,i%y-1)))
            if i%y+1<y:# right
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y, i % y + 1)))
            if i/y-1 >=0:# up
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y-1, i % y)))
            if i/y +1 <y:# down
                self.Edges[i].append(Edge(from_p=(i / y, i % y), to_p=(i / y+1, i % y)))

    def initTopology(self):
        return False

    def resetEnv(self):
        self.Points = []
        self.Edges = []
        if self.topology == 'Grid':
            if self.initMode == "Default":
                self.initDefault()
            elif self.initMode == 'Config':
                self.initConfig()
            elif self.initMode == 'Random':
                self.initRandom()
        else:
            self.initTopology()

    def EnvStep(self):
        for i in range(len(self.Points)):
            changeP=self.Points[i].changeProb

