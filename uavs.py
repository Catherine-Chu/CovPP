import random
from maddpg.trainer.replay_buffer import ReplayBuffer
from center import *
import time
import copy

class AdaptiveConfig:
    def __init__(self):
        self.WarnTimeRatio = 0.6
        self.ForbidTimeRatio = 0.8


class UAV:
    def __init__(self, pos_x, pos_y, workcap=40, sense_r = 1, global_view = None):
        self.config = AdaptiveConfig()
        self.init_pos = [pos_x,pos_y]
        self.pos = [pos_x, pos_y]
        self.old_pos = [pos_x,pos_y]
        self.workcap = 40
        self.worktime = 0
        self.land_interval = 1
        self.floor_interval = 1
        # TODO: Establish relation between sense_r and sense_p
        self.sense_r=sense_r
        self.sense_p = [(1,0),(-1,0),(0,1),(0,-1),(0,0)]#下，上，右，左,当前
        self.global_view = global_view
        self.local_view = None
        self.islanding = False
        self.isCharging = False
        self.actions=[(1,0,False),(-1,0,False),(0,1,False),(0,-1,False),(0,0,False),(0,0,True)]# (0,0,False)表示悬浮不动,(0,0,True)表示降落
        self.staytime = 0
        self.experience = ReplayBuffer(1e6)
        self.terminal = False

        self.step_in_path = 0
        self.path = []
        self.action_seq = []
        self.step_in_charg = 0
        self.chargPath = None
        self.charg_action_seq = []

    def reset(self):
        self.pos = self.init_pos
        self.worktime = 0
        self.global_view = None
        self.local_view = None
        self.islanding = False
        self.isCharging = False
        del self.experience
        self.experience = ReplayBuffer(1e6)
        self.terminal = False
        self.step_in_path = 0
        self.path.clear()
        self.action_seq.clear()

    def takeAction(self, env, center, strategy='Adaptive_Move'):
        moved = False
        action = None
        self.get_local_view(env)
        self.get_global_view(center)
        self.old_pos = [self.pos[0], self.pos[1]]
        # print("Before take action:")
        # print(self.pos)
        # print(self.old_pos)


        # for key in self.local_view:
        #     print(key)
        #     if self.local_view[key]:
        #         print("Reachable: %s.\n" % self.local_view[key].reachable)
        #         print("Visited: %s.\n" % self.local_view[key].visited)
        #     else:
        #         print("None!")

        # Strategy
        from_p = self.pos[1]+self.pos[0]*env.scale[1]

        if strategy == 'Blind_Random':
            rand_action = random.choice(self.actions)
            self.pos[0] += rand_action[0]
            self.pos[1] += rand_action[1]
            moved = True
            action = rand_action
        elif strategy == 'Avoid_Random':
            try_times = 0
            try_actions = []
            if self.worktime <= self.workcap and not self.terminal:
                while True:
                    if self.pos[0]>0 and self.pos[0]<env.scale[0]-1 and self.pos[1]>0 and self.pos[1]<env.scale[1]-1:
                        allow_actions = self.actions[0:4]
                    elif self.pos[0] == 0 and self.pos[1]>0 and self.pos[1]<env.scale[1]-1:
                        allow_actions = [self.actions[0],self.actions[2],self.actions[3]]
                    elif self.pos[0] == env.scale[0]-1 and self.pos[1]>0 and self.pos[1]<env.scale[1]-1:
                        allow_actions = [self.actions[1],self.actions[2],self.actions[3]]
                    elif self.pos[1] == 0 and self.pos[0]>0 and self.pos[0]<env.scale[0]-1:
                        allow_actions = [self.actions[0], self.actions[1], self.actions[2]]
                    elif self.pos[1] == env.scale[1] - 1 and self.pos[0]>0 and self.pos[0]<env.scale[0]-1:
                        allow_actions = [self.actions[0], self.actions[1], self.actions[3]]
                    elif self.pos == [0,0]:
                        allow_actions = [self.actions[0],self.actions[2]]
                    elif self.pos == [0,env.scale[1]-1]:
                        allow_actions = [self.actions[0],self.actions[3]]
                    elif self.pos == [env.scale[0]-1,0]:
                        allow_actions = [self.actions[1],self.actions[2]]
                    elif self.pos == [env.scale[0]-1,env.scale[1]-1]:
                        allow_actions = [self.actions[1],self.actions[3]]
                    allow_actions.extend(self.actions[4:5])

                    rand_action = random.choice(allow_actions)
                    rand_act = (rand_action[0],rand_action[1])
                    # 判断action合不合法的时候应该是根据local_view判断是否可达，根据global_view判断是否已经访问
                    # env每个点的visited情况应该和global_view的完全一致，但是否可达不一定一致
                    try_p = self.local_view[rand_act]
                    from_p = self.pos[0]*env.scale[1]+self.pos[1]
                    if try_p.reachable:
                        if not (isinstance(try_p, ChargingPoint) and rand_action == self.actions[5] and try_p.stop_num == try_p.cap):
                            to_p = (self.pos[0]+rand_action[0])*env.scale[1]+(self.pos[1]+rand_action[1])
                            tmp_worktime = self.worktime + env.Edges[from_p][to_p].traveltime
                            if rand_action == self.actions[4]:
                                tmp_worktime += self.floor_interval
                            elif rand_action == self.actions[5] and isinstance(try_p,ChargingPoint):
                                tmp_worktime -= self.land_interval
                            if tmp_worktime <= self.workcap:
                                self.pos[0] += rand_action[0]
                                self.pos[1] += rand_action[1]
                                moved = True
                                self.worktime = tmp_worktime
                                action= rand_action
                                if action == self.actions[5]:
                                    self.islanding = True
                                else:
                                    self.islanding = False
                                break
                    try_times += 1
                    if rand_action not in try_actions:
                        try_actions.append(rand_action)

                    if try_times >= 6 and len(try_actions) < len(allow_actions):
                        for i in allow_actions:
                            if i not in try_actions:
                                allow_act = (i[0],i[1])
                                try_p = self.local_view[allow_act]
                                if try_p.reachable:
                                    if not (isinstance(try_p, ChargingPoint) and rand_action == self.actions[5] and try_p.stop_num == try_p.cap):
                                        to_p = (self.pos[0] + rand_action[0]) * env.scale[1] + (self.pos[1] + rand_action[1])
                                        tmp_worktime = self.worktime + env.Edges[from_p][to_p].traveltime
                                        if rand_action == self.actions[4]:
                                            tmp_worktime += self.floor_interval
                                        elif rand_action == self.actions[5] and isinstance(try_p, ChargingPoint):
                                            tmp_worktime -= self.land_interval
                                        if tmp_worktime <= self.workcap:
                                            self.pos[0] += rand_action[0]
                                            self.pos[1] += rand_action[1]
                                            moved = True
                                            self.worktime = tmp_worktime
                                            action = rand_action
                                            if action == self.actions[5]:
                                                self.islanding = True
                                            else:
                                                self.islanding = False
                                            break
                        break
                    elif try_times >= 6 and len(try_actions) >= len(allow_actions):
                        break
        elif strategy == 'Adaptive_Move':
            if self.worktime <= self.workcap and not self.terminal:
                if self.worktime <= self.config.ForbidTimeRatio * self.workcap:
                    old_try_p=(self.pos[0],self.pos[1])
                    if self.pos[0] > 0 and self.pos[0] < env.scale[0] - 1 and self.pos[1] > 0 and self.pos[1] < env.scale[
                        1] - 1:
                        allow_actions = [1,1,1,1]
                    elif self.pos[0] == 0 and self.pos[1] > 0 and self.pos[1] < env.scale[1] - 1:
                        allow_actions = [1,0,1,1]
                    elif self.pos[0] == env.scale[0] - 1 and self.pos[1] > 0 and self.pos[1] < env.scale[1] - 1:
                        allow_actions = [0,1,1,1]
                    elif self.pos[1] == 0 and self.pos[0] > 0 and self.pos[0] < env.scale[0] - 1:
                        allow_actions = [1,1,1,0]
                    elif self.pos[1] == env.scale[1] - 1 and self.pos[0] > 0 and self.pos[0] < env.scale[0] - 1:
                        allow_actions = [1,1,0,1]
                    elif self.pos == [0, 0]:
                        allow_actions = [1,0,1,0]
                    elif self.pos == [0, env.scale[1] - 1]:
                        allow_actions = [1,0,0,1]
                    elif self.pos == [env.scale[0] - 1, 0]:
                        allow_actions = [0,1,1,0]
                    elif self.pos == [env.scale[0] - 1, env.scale[1] - 1]:
                        allow_actions = [0,1,0,1]

                    tolerant_actions = allow_actions
                    # print("origin_try_p:")
                    # print(old_try_p)
                    for i in range(len(allow_actions)):
                        if allow_actions[i] == 1:
                            try_p = (old_try_p[0]+self.actions[i][0], old_try_p[1]+self.actions[i][1])
                            # print("try_p: %s, %s." % (try_p[0], try_p[1]))
                            if isinstance(self.local_view[self.sense_p[i]],ChargingPoint):
                                continue
                            else:
                                if self.local_view[self.sense_p[i]].reachable and not self.global_view.Points[try_p[0]][try_p[1]].visited:
                                    continue
                                elif self.local_view[self.sense_p[i]].reachable and self.global_view.Points[try_p[0]][try_p[1]].visited:
                                    allow_actions[i] = 0
                                elif not self.local_view[self.sense_p[i]].reachable:
                                    allow_actions[i] = 0
                                    tolerant_actions[i] = 0

                    if allow_actions[1]==1:
                        action = self.actions[1]
                    elif allow_actions[2]==1 and allow_actions[3]==1:
                        action = random.choice(self.actions[2:4])
                    elif allow_actions[2]==1 and allow_actions[3]==0:
                        action = self.actions[2]
                    elif allow_actions[2]==0 and allow_actions[3]==1:
                        action = self.actions[3]
                    elif allow_actions[0]==1:
                        action = self.actions[0]
                    else:
                        t = False
                        for i in range(len(tolerant_actions)):
                            t = t or bool(tolerant_actions[i])
                        if not t:
                            action = self.actions[5]# None?
                            self.terminal = True
                        else:
                            if tolerant_actions[1] == 1:
                                action = self.actions[1]
                            elif tolerant_actions[2] == 1 and tolerant_actions[3] == 1:
                                action = random.choice(self.actions[2:4])
                            elif tolerant_actions[2] == 1 and tolerant_actions[3] == 0:
                                action = self.actions[2]
                            elif tolerant_actions[2] == 0 and tolerant_actions[3] == 1:
                                action = self.actions[3]
                            elif tolerant_actions[0] == 1:
                                action = self.actions[0]
                    if self.worktime <= self.config.WarnTimeRatio * self.workcap:
                        self.chargPath = None
                        self.step_in_charg = 0
                        self.charg_action_seq = []
                    else:
                        if not self.chargPath:
                            choice = random.uniform(0,1)
                            if choice < 0.5:
                                if self.local_view[(0, 0)].ischargingp:
                                    tar_station = self.pos[0] * self.global_view.scale[1] + self.pos[1]
                                    self.step_in_charg = 0
                                    action = self.actions[5]
                                else:
                                    temp_path, find, tar_station = self.findBestStation(start_p=from_p)
                                    hasPath, self.chargPath, self.charg_action_seq = self.replanPath(start_p=from_p,
                                                                                            tar_station=tar_station, path = temp_path)
                                    self.step_in_charg = 0
                                    action = self.charg_action_seq[self.step_in_charg]
                                    if find:
                                        self.step_in_charg += 1
                                    else:
                                        self.chargPath = None
                                        self.charg_action_seq.clear()
                        else:
                            if self.step_in_charg == len(self.chargPath) and self.local_view[(0, 0)].ischargingp:
                                action = self.actions[5]
                                self.step_in_charg = 0
                                self.chargPath = None
                                self.charg_action_seq = []
                            else:
                                action = self.charg_action_seq[self.step_in_charg]
                                # 需要判断action是不是合法，是否需要replan path
                                try_p = (self.pos[0] + action[0], self.pos[1] + action[1])
                                if not isinstance(self.local_view[(action[0],action[1])], ChargingPoint):
                                    if not self.local_view[(action[0],action[1])].reachable:
                                        hasPath,last_chargPath, last_charg_action_seq = self.replanPath(start_p=from_p,
                                                                                                tar_station=self.chargPath[len(self.chargPath)-1])
                                        if hasPath:
                                            self.chargPath = self.chargPath[0:self.step_in_charg+1].extend(last_chargPath)
                                            self.charg_action_seq = self.charg_action_seq[0:self.step_in_charg].extend(last_charg_action_seq)
                                            action = self.charg_action_seq[self.step_in_charg]
                                        else:
                                            moved=False
                                            action = None
                                            self.terminal = True
                                if not action:
                                    self.step_in_charg += 1
                else:
                    # 目前是电量高于40%就继续搜索，低于40%就必须去充电（在充电或去充电途中），这样可能会出现震荡，
                    # 即一低于40%就就近充电，刚充电到高于40%就继续工作，走不远就又需要充电了
                    # 可改进为40%以上必须工作，20%-40%可以工作可以充电，20%以下必须充电
                    # 或者改进成任何时间都可以充电，但40%以下必须充电
                    if self.chargPath:
                        if self.step_in_charg == len(self.chargPath) and self.local_view[(0,0)].ischargingp:
                            action = self.actions[5]
                            self.step_in_charg = 0
                            self.chargPath = None
                            self.charg_action_seq = []
                        else:
                            action = self.charg_action_seq[self.step_in_charg]
                            # 需要判断action是不是合法，是否需要replan path
                            try_p = (self.pos[0] + action[0], self.pos[1] + action[1])
                            if not isinstance(self.local_view[(action[0], action[1])], ChargingPoint):
                                if not self.local_view[(action[0], action[1])].reachable:
                                    hasPath, last_chargPath, last_charg_action_seq = self.replanPath(start_p=from_p,
                                                                                            tar_station=self.chargPath[
                                                                                                len(
                                                                                                    self.chargPath) - 1])
                                    if hasPath:
                                        self.chargPath = self.chargPath[0:self.step_in_charg + 1].extend(last_chargPath)
                                        self.charg_action_seq = self.charg_action_seq[0:self.step_in_charg].extend(
                                            last_charg_action_seq)
                                        action = self.charg_action_seq[self.step_in_charg]
                                    else:
                                        moved=False
                                        action = None
                                        self.terminal = True
                            if not action:
                                self.step_in_charg += 1
                    else:
                        if self.local_view[(0,0)].ischargingp:
                            tar_station = self.pos[0]*self.global_view.scale[1]+self.pos[1]
                            self.step_in_charg = 0
                            action = self.actions[5]
                        else:
                            temp_path, find,tar_station = self.findBestStation(start_p=from_p)
                            hasPath, self.chargPath,self.charg_action_seq = self.replanPath(start_p=from_p,tar_station=tar_station, path=temp_path)
                            self.step_in_charg = 0
                            action = self.charg_action_seq[self.step_in_charg]
                            if find:
                                self.step_in_charg += 1
                            else:
                                self.chargPath = None
                                self.charg_action_seq.clear()

                to_p = (self.pos[0] + action[0]) * env.scale[1] + (self.pos[1] + action[1])
                tmp_worktime = self.worktime + self.global_view.Edges[from_p][to_p].traveltime
                if action == self.actions[5] and isinstance(try_p, ChargingPoint):
                    tmp_worktime -= self.land_interval
                if tmp_worktime <= self.workcap:
                    self.pos[0] += action[0]
                    self.pos[1] += action[1]
                    moved = True
                    self.worktime = tmp_worktime
                    if action == self.actions[5]:
                        self.islanding = True
                    else:
                        self.islanding = False
                else:
                    moved = False
                    action = None
                    self.terminal = True
        # elif strategy == 'Follow_Path':
        #     self.get_adaptive_path()
        #     if self.step_in_path < len(self.path):
        #         action = self.action_seq[self.step_in_path]
        #         moved = True
        #         self.step_in_path += 1
        #     else:
        #         self.terminal = True
        # elif strategy == 'Adaptive_Path':
        #     self.get_adaptive_path()#IF EXIST,NONE;ELSE NEW PATH
        #     past_path = self.path[0:self.step_in_path]
        #     past_action_seq = self.action_seq[0:self.step_in_path]
        #     if self.worktime > self.workcap*2/3:
        #         temp_path, find,tar_station = self.findBestStation()#IF EXIST,FOLLOW;ELSE NEW STATION
        #         start_p = self.path[self.step_in_path]
        #         hasPath,last_path,last_action_seq = self.replanPath(start_p, tar_station,path = temp_path)
        #         if hasPath:
        #             self.path = past_path.extend(last_path)
        #             self.action_seq = past_action_seq.extend(last_action_seq)
        #         else:
        #             print("Can't find safe path for charging.")
        #     if self.step_in_path < len(self.path):
        #         action = None
        #         while not action:
        #             action = self.action_seq[self.step_in_path]
        #             act = (action[0], action[1])
        #             # 判断action合不合法的时候应该是根据local_view判断是否可达，根据global_view判断是否已经访问
        #             # env每个点的visited情况应该和global_view的完全一致，但是否可达不一定一致
        #             try_p = self.local_view[act]
        #             from_p = self.pos[0] * env.scale[1] + self.pos[1]
        #             if try_p.reachable:
        #                 if not (isinstance(try_p, ChargingPoint) and action == self.actions[
        #                     5] and try_p.stop_num == try_p.cap):
        #                     to_p = (self.pos[0] + action[0]) * env.scale[1] + (self.pos[1] + action[1])
        #                     tmp_worktime = self.worktime + env.Edges[from_p][to_p].traveltime
        #                     if action == self.actions[4]:
        #                         tmp_worktime += self.floor_interval
        #                     elif action == self.actions[5] and isinstance(try_p, ChargingPoint):
        #                         tmp_worktime -= self.land_interval
        #                     if tmp_worktime <= self.workcap:
        #                         self.pos[0] += action[0]
        #                         self.pos[1] += action[1]
        #                         moved = True
        #                         self.worktime = tmp_worktime
        #                         if action == self.actions[5]:
        #                             self.islanding = True
        #                         else:
        #                             self.islanding = False
        #             if not moved:
        #                 action = None
        #                 temp_path = None
        #                 start_p = self.path[self.step_in_path]
        #                 if self.worktime > self.workcap * 2 / 3:
        #                     temp_path, find,tar_station = self.findBestStation(start_p)
        #                 else:
        #                     tar_station = None
        #                 center = self.put_local_view(env, center)
        #                 hasPath, last_path, last_action_seq = self.replanPath(start_p, tar_station, path=temp_path)
        #                 if hasPath:
        #                     self.path = past_path.extend(last_path)
        #                     self.action_seq = past_action_seq.extend(last_action_seq)
        #                 else:
        #                     print("Can't find safe path for avoiding obstacles and charging.")
        #             else:
        #                 self.step_in_path += 1
        #     if self.step_in_path == len(self.path):
        #         self.terminal = True

        if moved:
            if self.worktime >= self.workcap and not isinstance(env.Points[self.pos[0]][self.pos[1]],ChargingPoint):
                self.terminal = True
        else:
            self.terminal = True

        # print("After take action:")
        # print(self.pos)
        # print(self.old_pos)
        return moved, self.terminal, action, center

    def get_local_view(self, env):
        self.local_view = {}
        for i in range(len(self.sense_p)):
            sense_pos = (self.pos[0]+self.sense_p[i][0],self.pos[1]+self.sense_p[i][1])
            if sense_pos[0]>=0 and sense_pos[0]<env.scale[0] and sense_pos[1]>=0 and sense_pos[1]<env.scale[1]:
                self.local_view[self.sense_p[i]]= env.Points[sense_pos[0]][sense_pos[1]]
            else:
                self.local_view[self.sense_p[i]]= None

    def get_global_view(self, center):
        self.global_view = center.global_view

    def put_local_view(self, env, center, action = None):
        if action:
            self.local_view[(action[0],action[1])] = env.Points[self.pos[0]][self.pos[1]]
        # print("Before put:")
        # print(self.old_pos)
        center.put_view(self.local_view, self.old_pos, self.sense_p)
        return center

    # def generate_target_area(self):
    #     # left_line =[]
    #     # sep_obs = []
    #     # for i in range(self.global_view.scale[0]):
    #     #     for j in range(self.global_view_scale[1]):
    #     #         p = self.global_view.Points[i][j]
    #     #         if not p.reachable:
    #     #             temp_left = j
    #     #             left_line.append(temp_left)
    #     #             sep_obs.append(i)
    #     #             # if temp_left not in right_line and temp_left not in left_line:
    #     #             #     left_line.append(temp_left)
    #     #             # elif temp_left in right_line:
    #     #             #     right_line[right_line.index(x=temp_left)]=temp_right
    #     #             # if temp_right not in left_line and temp_right not in right_line:
    #     #             #     right_line.append(temp_right)
    #     #             # elif temp_right in left_line:
    #     #             #     left_line[left_line.index(x=temp_right)] = temp_left
    #     # unique_left_line = []
    #     # line_sep_obs = []
    #     # for i, k in enumerate(left_line):
    #     #     if k not in unique_left_line:
    #     #         unique_left_line.append(k)
    #     #         line_sep_obs.append([])
    #     #     line_sep_obs[unique_left_line.index(x=k)].append(sep_obs[i])
    #     num_uav = len(self.global_view.state)

    def get_adaptive_path(self):
        # based on currently global view(avoid known obstacles and satisfy the worktime limitation)
        self.path = []
        self.action_seq = []

    def findBestStation(self,start_p):
        #需要考虑到时间限制找到最优目标，如果没有时间内可达的目标则返回false,最近station标号
        find = False
        stats = []
        for i,row in enumerate(self.global_view.Points):
            for j,p in enumerate(row):
                if isinstance(p,ChargingPoint):
                    stats.append((i*self.global_view.scale[1]+j,p))
        '''
        暂时加在这里,通过遍历图中的点更新所有边上的权值,
        原本边的改变应该在点被更新的时候同时改变,对global_view而言就是调用put_local_view的时候,
        后面更新后可以直接利用global_view的Edges信息进行计算
        '''
        for i in range(len(self.global_view.Edges)):
            r = int(i/self.global_view.scale[1])
            c = i-r*self.global_view.scale[1]
            p = self.global_view.Points[r][c]
            if not p.reachable:
                if r>0 and r< self.global_view.scale[0]-1 and c>0 and c<self.global_view.scale[1]-1:
                    self.global_view.Edges[i][i+1].traveltime = 100000
                    self.global_view.Edges[i][i-1].traveltime = 100000
                    self.global_view.Edges[i][i-self.global_view.scale[1]].traveltime = 100000
                    self.global_view.Edges[i][i+self.global_view.scale[1]].traveltime = 100000
                    self.global_view.Edges[i+1][i].traveltime = 100000
                    self.global_view.Edges[i-1][i].traveltime = 100000
                    self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000
                    self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                elif r==0:
                    self.global_view.Edges[i][i+self.global_view.scale[1]].traveltime = 100000
                    self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                    if c>0 or c<self.global_view.scale[1]-1:
                        self.global_view.Edges[i][i + 1].traveltime = 100000
                        self.global_view.Edges[i][i - 1].traveltime = 100000
                        self.global_view.Edges[i+1][i].traveltime = 100000
                        self.global_view.Edges[i-1][i].traveltime = 100000
                    elif c==0:
                        self.global_view.Edges[i][i + 1].traveltime = 100000
                        self.global_view.Edges[i+1][i].traveltime = 100000
                    elif c==self.global_view.scale[1]-1:
                        self.global_view.Edges[i][i - 1].traveltime = 100000
                        self.global_view.Edges[i - 1][i].traveltime = 100000
                elif r==self.global_view.scale[0]-1:
                    self.global_view.Edges[i][i - self.global_view.scale[1]].traveltime = 100000
                    self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000
                    if c > 0 or c < self.global_view.scale[1] - 1:
                        self.global_view.Edges[i][i + 1].traveltime = 100000
                        self.global_view.Edges[i][i - 1].traveltime = 100000
                        self.global_view.Edges[i + 1][i].traveltime = 100000
                        self.global_view.Edges[i - 1][i].traveltime = 100000
                    elif c == 0:
                        self.global_view.Edges[i][i + 1].traveltime = 100000
                        self.global_view.Edges[i + 1][i].traveltime = 100000
                    elif c == self.global_view.scale[1].traveltime - 1:
                        self.global_view.Edges[i][i - 1].traveltime = 100000
                        self.global_view.Edges[i - 1][i].traveltime = 100000
                elif c == 0:
                    self.global_view.Edges[i][i+1].traveltime = 100000
                    self.global_view.Edges[i+1][i].traveltime = 100000
                    if r > 0 or r < self.global_view.scale[0] - 1:
                        self.global_view.Edges[i][i + self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i][i - self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                        self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000
                    elif r == 0:
                        self.global_view.Edges[i][i + self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                    elif r == self.global_view.scale[0] - 1:
                        self.global_view.Edges[i][i - self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000
                elif c == self.global_view.scale[1]-1:
                    self.global_view.Edges[i][i - 1].traveltime = 100000
                    self.global_view.Edges[i - 1][i].traveltime = 100000
                    if r > 0 or r < self.global_view.scale[0] - 1:
                        self.global_view.Edges[i][i + self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i][i - self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                        self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000
                    elif r == 0:
                        self.global_view.Edges[i][i + self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i + self.global_view.scale[1]][i].traveltime = 100000
                    elif r == self.global_view.scale[0] - 1:
                        self.global_view.Edges[i][i - self.global_view.scale[1]].traveltime = 100000
                        self.global_view.Edges[i - self.global_view.scale[1]][i].traveltime = 100000

        '''
        ############################################################################
        '''
        # Floyd algorithm
        A = []
        path = []
        n = len(self.global_view.Edges)
        for i in range(n):
            A.append([])
            path.append([])
            for j in range(n):
                A[i].append(self.global_view.Edges[i][j].traveltime)
                path[i].append(-1)
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if A[i][j] > (A[i][k] + A[k][j]):
                        A[i][j] = A[i][k] + A[k][j]
                        path[i][j] = k
        min = 100000
        tar = (-1,None)
        for stat_p in stats:
            if min > A[start_p][stat_p[0]]:
                min = A[start_p][stat_p[0]]
                tar = stat_p

        total_travel_time = 0
        t = tar[0]
        while t != start_p and t != -1:
            mid = path[start_p][t]
            if mid != -1:
                total_travel_time += self.global_view.Edges[mid][t].traveltime
                t = mid
            else:
                total_travel_time += self.global_view.Edges[start_p][t].traveltime
                break
        if total_travel_time <= self.workcap - self.worktime:
            find = True
        else:
            find = False
        return path, find, tar[0]

    def replanPath(self, start_p, tar_station = None, path = None):
        #需要考虑到时间限制，在时间限制内的可行路径才行
        hasPath = False
        if tar_station:
            if not self.chargPath or not path:
                if not path:
                    A = []
                    path = []
                    n = len(self.global_view.Edges)
                    for i in range(n):
                        A.append([])
                        path.append([])
                        for j in range(n):
                            A[i].append(self.global_view.Edges[i][j].traveltime)
                            path[i].append(-1)
                    for k in range(n):
                        for i in range(n):
                            for j in range(n):
                                if A[i][j] > (A[i][k] + A[k][j]):
                                    A[i][j] = A[i][k] + A[k][j]
                                    path[i][j] = k

                temp_path = [tar_station]
                t = path[start_p][tar_station]
                while t != -1:
                    temp_path.append(t)
                    t = path[start_p][t]
                temp_path.append(start_p)
                temp_path.reverse()
                temp_actions = []
                for i in range(1,len(temp_path)):
                    bias = temp_path[i]-temp_path[i-1]
                    if bias == 1:
                        temp_actions.append(self.actions[2])
                    elif bias == -1:
                        temp_actions.append(self.actions[3])
                    elif bias == self.global_view.scale[1]:
                        temp_actions.append(self.actions[0])
                    elif bias == - self.global_view.scale[1]:
                        temp_actions.append(self.actions[1])
                    else:
                        print("Illegal Action.")
                total_travel_time = 0
                for i in range(1,len(temp_path)):
                    total_travel_time += self.global_view.Edges[temp_path[i-1]][temp_path[i]].traveltime
                if total_travel_time <= self.workcap - self.worktime:
                    hasPath = True
                else:
                    hasPath = False
            else:
                temp_path = [start_p]
                temp_path.extend(self.chargPath[self.step_in_charg+1:len(self.chargPath)])
                temp_actions = self.charg_action_seq[self.step_in_charg:len*self.charg_action_seq]
                new_path = None
                s = (int(temp_path[0]/self.global_view.scale[1]), int(temp_path[0]%self.global_view.scale[1]))
                m = (int(temp_path[1]/self.global_view.scale[1]), int(temp_path[1]%self.global_view.scale[1]))
                l = (int(temp_path[2]/self.global_view.scale[1]), int(temp_path[2]%self.global_view.scale[1]))
                current_tar = 2
                _tar = temp_path[current_tar]
                search_area = [(max(0,min((s[0],m[0],l[0]))-1),min(self.global_view.scale[0]-1,max((s[0],m[0],l[0]))+1)),(max(0,min((s[1],m[1],l[1]))-1),min(self.global_view.scale[1]-1,max((s[1],m[1],l[1]))+1))]
                while True:
                    MAXN = 100000
                    v = -1
                    p_pos = []
                    for i in range(search_area[0][0],search_area[0][1]+1):
                        for j in range(search_area[1][0],search_area[1][1]+1):
                            p_pos.append(i*self.global_view.scale[1]+self.global_view.scale[0])
                    a = []
                    dist = []
                    for i,point in enumerate(p_pos):
                        if point == start_p:
                            v = i
                        if point == temp_path[current_tar]:
                            _tar = i
                        a.append([])
                        dist.append(MAXN)
                        for j in range(len(p_pos)):
                            a[i].append(self.global_view.Edges[point][p_pos[j]])
                    pre=[]
                    s=[]
                    n = (search_area[0][1]-search_area[0][0]+1)*(search_area[1][1]-search_area[1][0]+1)
                    m=0
                    for i in range(n):
                        pre.append(-1)
                        s.append(False)
                        if i != v:
                            dist[i] = a[v][i]
                        s[i] = False
                    s[v] = True
                    dist[v] = 0
                    pre[v]=v
                    for i in range(n):
                        minset = MAXN
                        u = v
                        for j in range(n):
                            if not s[j] and dist[j] < minset:
                                u = j
                                minset = dist[u]
                        s[u] = True
                        for j in range(n):
                            if not s[j] and a[u][j] < MAXN:
                                if dist[u] + a[u][j] < dist[j]:
                                    dist[j] = dist[u] + a[u][j]
                                    pre[j] = u

                    if dist[_tar]< MAXN:
                        new_path = [_tar]
                        _pre = pre[_tar]
                        while _pre != v:
                            new_path.append(_pre)
                            _pre = pre[_pre]
                        new_path.append(v)
                        new_path.reverse()

                    if not new_path:
                        if search_area[0][0]==0 and search_area[0][1] == self.global_view.scale[0]-1 and search_area[1][0]==0 and search_area[1][1]==self.global_view.scale[1]-1:
                            hasPath = False
                            current_tar = current_tar+1
                            if current_tar >= len(temp_path):
                                return False, None, None
                        else:
                            search_area = [(max(0, search_area[0][0] - 1),
                                            min(self.global_view.scale[0] - 1, search_area[0][1] + 1)),
                                           (max(0, search_area[1][0] - 1),
                                           min(self.global_view.scale[1] - 1, search_area[1][1] + 1))]
                        continue
                    else:
                        contain = False
                        for i in range(1,len(new_path)-1):
                            if p_pos[new_path[i]] in temp_path and temp_path.index(p_pos[new_path[i]])> current_tar:
                                mid_index = temp_path.index(p_pos[new_path[i]])
                                temp_path = temp_path[0:1].extend([p_pos[new_path[j]] for j in range(1,i) ]).extend(temp_path[temp_path.index(p_pos[new_path[i]]): len(temp_path)])
                                add_actions = []
                                for j in range(1,i+1):
                                    bias = temp_path[j]-temp_path[j-1]
                                    if bias == 1:
                                        add_actions.append(self.actions[2])
                                    elif bias == -1:
                                        add_actions.append(self.actions[3])
                                    elif bias == self.global_view.scale[1]:
                                        add_actions.append(self.actions[0])
                                    elif bias == - self.global_view.scale[1]:
                                        add_actions.append(self.actions[1])
                                    else:
                                        print("Illegal Action.")
                                temp_actions = add_actions.extend(temp_actions[mid_index:len(temp_actions)])
                                contain = True
                                break
                            else:
                                continue
                        if not contain:
                            temp_path = temp_path[0:1].extend([p_pos[new_path[j]] for j in range(1,len(new_path)-1)]).extend(temp_path[current_tar: len(temp_path)])
                            add_actions = []
                            for j in range(1, len(new_path)):
                                bias = temp_path[j] - temp_path[j - 1]
                                if bias == 1:
                                    add_actions.append(self.actions[2])
                                elif bias == -1:
                                    add_actions.append(self.actions[3])
                                elif bias == self.global_view.scale[1]:
                                    add_actions.append(self.actions[0])
                                elif bias == - self.global_view.scale[1]:
                                    add_actions.append(self.actions[1])
                                else:
                                    print("Illegal Action.")
                            temp_actions = add_actions.extend(temp_actions[current_tar:len(temp_actions)])

                        break

                total_travel_time = 0
                for i in range(1, len(temp_path)):
                    total_travel_time += self.global_view.Edges[temp_path[i - 1]][temp_path[i]].traveltime
                if total_travel_time <= self.workcap - self.worktime:
                    hasPath = True
                else:
                    hasPath = False
        else:
            start = start_p
            # TSP问题？哈密尔顿通路/回路问题？
            # hasPath, temp_path, temp_actions = Function(start_p)

        return hasPath, temp_path[1:len(temp_path)], temp_actions


if __name__ == '__main__':
    # done代表全局的是否结束
    # moved代表不违反动作选择约束(包括动作是否合法和动作是否安全)的情况下有动作可做
    # done returned from env.step代表没有撞墙，并且还没有完全覆盖
    # reward为大小为n的集合,表示当前步骤下每个agent获得的即时奖励
    episode = 1 # 迭代轮数
    scale = (20, 20) # 搜索范围
    uav_num = 2 # UAVs机群规模
    MAX_T = int(scale[0] * scale[1]/uav_num * 2) # 最大搜索步数

    env = Env(scale=scale, obstacles_num=40, stations_num=5)
    view = copy.deepcopy(env)

    for x in range(view.scale[0]):
        for y in range(view.scale[1]):
            if isinstance(view.Points[x][y], ChargingPoint):
                init_prob = view.Points[x][y].changeProb
                view.Points[x][y].changeProb = 0.2
                view.Points[x][y].update()
                view.Points[x][y].changeProb = init_prob
            else:
                view.Points[x][y].update()
    center = Center(view)

    fleet = []
    tmp_fleet_pos = []
    uav_count = 0
    while uav_count < uav_num:
        rand_x = int(random.randint(0, scale[0] - 1))
        rand_y = int(random.randint(0, scale[1] - 1))
        f_p = rand_x * scale[1]+rand_y
        t_P = env.Points[rand_x][rand_y]
        if t_P.reachable and f_p not in tmp_fleet_pos:
            if (not isinstance(t_P,ChargingPoint)) or (isinstance(t_P,ChargingPoint) and t_P.stop_num<t_P.cap):
                fleet.append(UAV(pos_x=rand_x, pos_y=rand_y))
                tmp_fleet_pos.append(f_p)
                uav_count += 1

    for i in range(episode):
        start_t = time.time()
        env.reset()
        del center.global_view
        center.global_view = copy.deepcopy(view)
        dones = False
        temp_dones = True

        for u in fleet:
            u.reset()
            env.state.append(np.array([u.pos[0], u.pos[1], 0, 0]))# x,y,isdone(结束或者坠毁),island(主动降落）
            env.Points[u.pos[0]][u.pos[1]].reachable = True
            env.Points[u.pos[0]][u.pos[1]].visited = True
            if isinstance(env.Points[u.pos[0]][u.pos[1]], ChargingPoint):
                env.Points[u.pos[0]][u.pos[1]].stop_num += 1
            center.global_view.Points[u.pos[0]][u.pos[1]].reachable = True
            center.global_view.Points[u.pos[0]][u.pos[1]].visited = True
            if isinstance(center.global_view.Points[u.pos[0]][u.pos[1]], ChargingPoint):
                center.global_view.Points[u.pos[0]][u.pos[1]].stop_num = env.Points[u.pos[0]][u.pos[1]].stop_num
        center.global_view.state = env.state

        step_count = 0
        while step_count < MAX_T and not dones:
            env.render()
            action = []
            obs_n = env.state
            has_moved = False
            for u in fleet:
                moved, done, ac, center = u.takeAction(env=env,center=center)
                if moved:
                    action.append(ac)
                else:
                    action.append(None)
                has_moved = has_moved or moved # 只要还有在moved的has_moved=True
            dones = not has_moved
            if not dones:
                new_state, reward, dones, info = env.step(action)
                center.global_view.state = new_state
                for i in range(len(fleet)):
                    temp_dones = temp_dones and fleet[i].terminal
                    # 可以在这里加上如果降落且不是必须降落的话，reward要减少，这不是环境直接反馈的reward，而是uav自检得出的惩罚
                    fleet[i].experience.add(obs_n[i], action[i], reward[i], new_state[i], float(dones))
                    center = fleet[i].put_local_view(env=env, center=center, action=action[i])
                dones = dones or temp_dones
                step_count += 1
            else:
                for i in range(len(fleet)):
                    center = fleet[i].put_local_view(env=env, center=center, action=action[i])
                break
            time.sleep(0.5)
        episode_t = time.time()-start_t
        print("episode %d: %s s."%(i , episode_t))

    env.close()




