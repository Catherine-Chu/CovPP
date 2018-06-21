import random
import yaml
import gym
import gym.spaces as spaces
from yaml import load,dump
import numpy as np
from gym.envs.classic_control import rendering


class Point:
    def __init__(self, reachable, visited=False, timecost = 0, changeProb = 0.2):
        self.reachable = reachable
        self.visited = visited
        self.timecost = timecost
        self.changeProb = changeProb
        self.timespent = 0
        self.ischargingp = False
        pass

    def update(self):
        e = random.random()
        if e < self.changeProb:
            self.reachable = not self.reachable
        pass

class ChargingPoint(Point):
    def __init__(self,timecost = 30, cap = 8 ):
        super(ChargingPoint, self).__init__(reachable=True,visited=False,timecost=timecost, changeProb=0)
        self.cap = cap
        self.stop_num = random.randint(0, self.cap)
        self.chargingList = []
        self.ischargingp = True

    def update(self):
        super(ChargingPoint, self).update()
        if self.ischargingp:
            self.stop_num = len(self.chargingList)+random.randint(0, self.cap-len(self.chargingList))

class Edge:
    def __init__(self, from_p, to_p, traveltime = 1):
        self.from_p = from_p
        self.to_p = to_p
        self.traveltime = traveltime

class Env(gym.Env):

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 2
    }

    def __init__(self, scale=(0, 0), initMode='Default', topology='Grid'):
        self.interval = 5
        self.countStep = self.interval
        self.state = []
        self.viewer = None
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
        self.action_space = spaces.Discrete(5) # 0, 1, 2，3，4: 不动，右左下上
        self.observation_space = spaces.Dict({'global': spaces.Box(np.array([0, 0]), np.array([self.scale[0], self.scale[1]])),
                                              'local': spaces.Discrete(5)})
        pass

    def reset(self):
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

    def step(self, action):
        reward = []
        done = False

        for i in range(len(self.state)):
            # 到达该点环境反馈的奖励
            self.state[i][0] += action[i][0][0]
            self.state[i][1] += action[i][0][1]
            P=self.Points[self.state[i][0]][self.state[i][1]]
            print("x %d"%self.state[i][0])
            print("y %d"%self.state[i][1])
            if not P.reachable:
                done = True
                reward.append(-10)
            elif P.visited:
                reward.append(-1)
            elif P.ischargingp:
                reward.append(0)
            else:
                reward.append(1)

            # 在该点停留环境反馈的奖励
            staytime = action[i][1]
            if not P.ischargingp and P.reachable:
                if P.timespent + staytime < P.timecost:
                    reward[i] += 5
                    print("E1")
                elif P.timespent + staytime == P.timecost:
                    reward[i] += 10
                    self.Points[self.state[i][0]][self.state[i][1]].visited = True
                else:
                    reward[i] += 10 - (P.timespent + staytime - P.timecost)
                    self.Points[self.state[i][0]][self.state[i][1]].visited = True
                    print("E2")
                    print(staytime)

            if P.ischargingp and staytime > 0:
                if P.stop_num < P.cap:
                    P.stop_num += 1

        if not done:
            done = self.checkCovProcess()
        # change the env
        self.countStep -= 1
        while self.countStep == 0:
            for x in range(self.scale[0]):
                for y in range(self.scale[1]):
                    self.Points[x][y].update()
        self.countStep = self.interval

        return self.state, reward, done, {}

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
            i_rr = int(i / y)
            i_cc = int(i % y)
            for j in range(x * y):
                j_rr = int(j / y)
                j_cc = int(j % y)
                if j == i:
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc), traveltime=0))
                elif (i_rr == j_rr and (i_cc == j_cc + 1 or i_cc == j_cc - 1)) or (
                    (i_rr == j_rr + 1 or i_rr == j_rr - 1) and i_cc == j_cc):
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc)))
                else:
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc), traveltime=100000))
            # rr = int(i/y)
            # cc = int(i%y)
            # if cc-1>=0:# left
            #     self.Edges[i].append(Edge(from_p=(rr,cc),to_p=(rr,cc-1)))
            # if cc+1<y:# right
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr, cc + 1)))
            # if rr-1 >=0:# up
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr-1, cc)))
            # if rr +1 <y:# down
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr+1, cc)))

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
            i_rr = int(i / y)
            i_cc = int(i % y)
            for j in range(x * y):
                j_rr = int(j / y)
                j_cc = int(j % y)
                if j == i:
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc), traveltime=0))
                elif (i_rr == j_rr and (i_cc == j_cc + 1 or i_cc == j_cc - 1)) or (
                    (i_rr == j_rr + 1 or i_rr == j_rr - 1) and i_cc == j_cc):
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc)))
                else:
                    self.Edges[i].append(Edge(from_p=(i_rr, i_cc), to_p=(j_rr, j_cc), traveltime=100000))
            # rr = int(i/y)
            # cc = int(i%y)
            # if cc-1>=0:# left
            #     self.Edges[i].append(Edge(from_p=(rr,cc),to_p=(rr,cc-1)))
            # if cc+1<y:# right
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr, cc + 1)))
            # if rr-1 >=0:# up
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr-1, cc)))
            # if rr +1 <y:# down
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr+1, cc)))

    def initRandom(self):
        x, y = self.scale
        for i in range(x):
            self.Points.append([])
            for j in range(y):
                rc = True
                if random.uniform(0,1) < 0.2:
                    rc = False
                self.Points[i].append(Point(reachable=rc))
        self.stations_num = random.randint(0,x)
        temp_stat_list = []
        for i in range(self.stations_num):
            rand_pos = random.randint(0, x*y-1)
            while rand_pos in temp_stat_list:
                rand_pos = random.randint(0, x*y-1)
            try:
                self.Points[int(rand_pos/y)][int(rand_pos%y)]=ChargingPoint()
            except IndexError:
                print("error pos_x: %d" %int(rand_pos/y))
                print("error pos_y: %d" %int(rand_pos%y))
            temp_stat_list.append(rand_pos)
        for i in range(x*y):
            self.Edges.append([])
        for i in range(x*y):
            i_rr = int(i / y)
            i_cc = int(i%y)
            for j in range(x*y):
                j_rr = int(j / y)
                j_cc = int(j % y)
                if j == i:
                    self.Edges[i].append(Edge(from_p=(i_rr,i_cc),to_p=(j_rr,j_cc),traveltime=0))
                elif (i_rr == j_rr and (i_cc == j_cc+1 or i_cc == j_cc-1)) or ((i_rr==j_rr+1 or i_rr==j_rr-1) and i_cc == j_cc):
                    self.Edges[i].append(Edge(from_p=(i_rr,i_cc),to_p=(j_rr,j_cc)))
                else:
                    self.Edges[i].append(Edge(from_p=(i_rr,i_cc),to_p=(j_rr,j_cc),traveltime=100000))

            # rr = int(i/y)
            # cc = int(i%y)
            # if cc-1>=0:# left
            #     self.Edges[i].append(Edge(from_p=(rr,cc),to_p=(rr,cc-1)))
            # if cc+1<y:# right
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr, cc + 1)))
            # if rr-1 >=0:# up
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr-1, cc)))
            # if rr +1 <y:# down
            #     self.Edges[i].append(Edge(from_p=(rr, cc), to_p=(rr+1, cc)))



    def initTopology(self):
        return False

    def render(self, mode='human', close=False):
        print(self.state)
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 500
        screen_height = 500

        w = 400 / (self.scale[0] - 1)
        h = 400 / (self.scale[1] - 1)

        if self.viewer is None:
            self.viewer = rendering.Viewer(screen_width, screen_height)
            h_lines = []
            v_lines = []
            agents = []
            obstacles = []
            stations = []
            self.agents_trans = []

            # 创建网格线
            for r in range(self.scale[0]):
                h_lines.append([])
                for c in range(self.scale[1]-1):
                    line = rendering.Line((50+c*w,450-r*h),(50+(c+1)*w,450-r*h))
                    line.set_color(0,0,0)
                    h_lines[r].append(line)
                    self.viewer.add_geom(line)
            for c in range(self.scale[1]):
                v_lines.append([])
                for r in range(self.scale[0]-1):
                    line = rendering.Line((50+c*w,450-r*h),(50+c*w,450-(r+1)*h))
                    line.set_color(0,0,0)
                    v_lines[c].append(line)
                    self.viewer.add_geom(line)
            # 创建障碍物/充电站
            for r in range(self.scale[0]):
                for c in range(self.scale[1]):
                    if not self.Points[r][c].reachable:
                        obs = rendering.make_circle(min(w,h)/2)
                        obs_trans = rendering.Transform(translation=(50+c*w,450-r*h))
                        obs.add_attr(obs_trans)
                        obs.set_color(0,0,0)
                        obstacles.append(obs)
                        self.viewer.add_onetime(obs)
                    if self.Points[r][c].ischargingp:
                        station = rendering.make_polygon([(50+c*w,450-r*h+h/2),(50+c*w-w/2,450-r*h-h/2),(50+c*w+w/2,450-r*h-h/2)])
                        stat_trans = rendering.Transform()
                        station.add_attr(stat_trans)
                        station.set_color(0,0.8,0.3)
                        stations.append(station)
                        self.viewer.add_geom(station)
            # 创建UAVs
            for i in range(len(self.state)):
                agent_pos=self.state[i]
                agent = rendering.make_circle(min(w,h)/2)
                agent_trans = rendering.Transform(translation=(50+agent_pos[1]*w,450-agent_pos[0]*h))
                self.agents_trans.append(agent_trans)
                agent.add_attr(self.agents_trans[i])
                agent.set_color(0.8, 0.6, 0.4)
                agents.append(agent)
                self.viewer.add_geom(agent)

            self.h_lines = h_lines
            self.v_lines = v_lines
            self.agents = agents
            self.obstacles = obstacles
            self.stations = stations


        if self.state is None or len(self.state) == 0: return None

        self.obstacles.clear()
        for r in range(self.scale[0]):
            for c in range(self.scale[1]):
                if not self.Points[r][c].reachable:
                    obs = rendering.make_circle(min(w, h) / 2)
                    obs_trans = rendering.Transform(translation=(50 + c * w, 450 - r * h))
                    obs.add_attr(obs_trans)
                    obs.set_color(0, 0, 0)
                    self.obstacles.append(obs)
                    self.viewer.add_onetime(obs)
        for i in range(len(self.state)):
            agent_pos = self.state[i]
            self.agents_trans[i].set_translation(50+agent_pos[1]*w,450-agent_pos[0]*h)

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer: self.viewer.close()

    def checkCovProcess(self):
        result = True
        for x in range(self.scale[0]):
            for y in range(self.scale[1]):
                if not self.Points[x][y].ischargingp and self.Points[x][y].reachable:
                    result= result and self.Points[x][y].visited
        return result


