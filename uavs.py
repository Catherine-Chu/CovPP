import random
from maddpg.trainer.replay_buffer import ReplayBuffer
from center import *
import time
import copy

class UAV:
    def __init__(self,pos_x, pos_y, workcap=40, sense_r = 1, global_view = True):
        self.pos = [pos_x, pos_y]
        self.old_pos = self.pos
        self.workcap = 40
        self.worktime = 0
        # TODO: Establish relation between sense_r and sense_p
        self.sense_r=sense_r
        self.sense_p = [(1,0),(-1,0),(0,1),(0,-1)]#下，上，右，左
        self.global_view = global_view
        self.local_view = None
        self.global_view = None
        self.island = True # island=False特指悬停, island=True特指降落充电
        self.isCharging = False
        self.actions=[(1,0),(-1,0),(0,1),(0,-1)]
        self.staytime = 0
        self.experience = ReplayBuffer(1e6)

    def takeAction(self, env, center, strategy='Avoid_Random'):
        moved = False
        action = [(0,0),0]
        self.get_local_view(env)
        self.get_global_view(center)
        self.old_pos = self.pos

        # Strategy
        from_p = self.pos[1]+self.pos[0]*env.scale[1]

        if strategy == 'Blind_Random':
            rand_action = random.choice(self.actions)
            self.pos[0] += rand_action[0]
            self.pos[1] += rand_action[1]
            moved = True
            action[0] = rand_action

        elif strategy == 'Avoid_Random':
            try_times = 0
            try_actions = []
            while True:
                if self.pos[0]>0 and self.pos[0]<env.scale[0]-1 and self.pos[1]>0 and self.pos[1]<env.scale[1]-1:
                    allow_actions = self.actions
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

                rand_action = random.choice(allow_actions)
                # TODO: 判断action合不合法的时候应该是根据local_view判断是否可达，根据global_view判断是否已经访问,此处有逻辑问题
                try_p = self.local_view[rand_action]
                if try_p.reachable and not try_p.visited:
                    self.pos[0] += rand_action[0]
                    self.pos[1] += rand_action[1]
                    moved = True
                    action[0] = rand_action
                    break
                try_times += 1
                print("Try_times: %d"% try_times)
                if rand_action not in try_actions:
                    try_actions.append(rand_action)

                if try_times >= 4 and len(try_actions)<len(allow_actions):
                    print("try")
                    for i in range(len(allow_actions)):
                        try_p = self.local_view[allow_actions[i]]
                        if try_p.reachable and not try_p.visited:
                            self.pos[0] += allow_actions[i][0]
                            self.pos[1] += allow_actions[i][1]
                            moved = True
                            action[0] = allow_actions[i]
                            break
                    print("Done")
                    break
                elif try_times>=4 and len(try_actions)>=len(allow_actions):
                    break
        if moved:
            obs_p = self.local_view[action[0]]
            print("x %d" % self.pos[0])
            print("y %d" % self.pos[1])
            if obs_p.ischargingp:
                if obs_p.stop_num < obs_p.cap:
                    staytime = random.randint(0, obs_p.timecost)
                else:
                    staytime = 0
            else:
                # TODO: 由于global_view的初始化也是完全随机的，和环境对于charging station的认知不同,有可能根据视觉判断此处不是充电站
                # 但是在global_view中这里是充电站，导致obs_p重新赋值后会得到一个很大的stay_time,此处有逻辑问题
                obs_p = self.global_view.Points[self.pos[0]][self.pos[1]]
                staytime = random.randint(0, obs_p.timecost-obs_p.timespent)

            to_p = self.pos[1]+self.pos[0]*env.scale[1]
            action[1] = staytime
            from_p = 0
            to_p = 20
            self.worktime += env.Edges[from_p][to_p].traveltime
            if obs_p.ischargingp:
                self.worktime -= min(self.worktime, staytime)

            # self evaluate first and update view by the strategy
            if self.worktime >= self.workcap and not obs_p.ischargingp:
                moved = False
        if not center:
            center = self.put_local_view(center, moved)

        print("WorkTime: %d"% self.worktime)
        return moved, action, center

    def get_local_view(self, env):
        self.local_view = {}
        for i in range(len(self.sense_p)):
            sense_pos = (self.pos[0]+self.sense_p[i][0],self.pos[1]+self.sense_p[i][1])
            if sense_pos[0]>=0 and sense_pos[0]<env.scale[0] and sense_pos[1]>=0 and sense_pos[1]<env.scale[1]:
                self.local_view[self.sense_p[i]]=env.Points[sense_pos[0]][sense_pos[1]]
            else:
                self.local_view[self.sense_p[i]]= None

    def get_global_view(self, center):
        self.global_view = center.global_view

    def put_local_view(self, center, moved = True):
        center.put_view(self.local_view, self.old_pos, self.sense_p, self.pos, self.staytime, moved)
        return center

if __name__ == '__main__':
    # done代表全局的是否结束
    # moved代表不违反动作选择约束(包括动作是否合法和动作是否安全)的情况下有动作可做
    # done returned from env.step代表没有撞墙，并且还没有完全覆盖
    # reward为大小为n的集合,表示当前步骤下每个agent获得的即时奖励
    MAX_T = 200
    episode = 1
    env = Env(scale=(20, 20), initMode='Random')
    view = copy.deepcopy(env)
    for x in range(view.scale[0]):
        for y in range(view.scale[1]):
            view.Points[x][y].update()
    center = Center(view)
    for i in range(episode):
        start_t=time.time()
        env.reset()
        del center.global_view
        center.global_view = copy.deepcopy(view)
        done = False
        fleet = [UAV(pos_x=0, pos_y=0), UAV(int(random.randint(0,19)),int(random.randint(0,19)))]
        for u in fleet:
            env.state.append(np.array([u.pos[0], u.pos[1]]))
            # TODO: 后续完善关于fleet在环境中的初始化
            env.Points[u.pos[0]][u.pos[1]].reachable = True
            env.Points[u.pos[0]][u.pos[1]].visited = True
            center.global_view.Points[u.pos[0]][u.pos[1]].reachable = True
            center.global_view.Points[u.pos[0]][u.pos[1]].visited = True

        terminal = 0
        while not done:
            env.render()
            action = []
            obs_n = env.state
            for u in fleet:
                moved, ac, center = u.takeAction(env=env,center=center)
                if moved:
                    action.append(ac)
                else:
                    done = True
                    break
            if not done:
                new_state, reward, done, info = env.step(action)
                for i in range(len(fleet)):
                    fleet[i].experience.add(obs_n[i], action[i], reward[i], new_state[i], float(done))
                terminal += 1
            if terminal >= MAX_T:
                break
            time.sleep(0.5)
        episode_t = time.time()-start_t
        print("episode %d: %s s."%(i , episode_t))

    env.close()




