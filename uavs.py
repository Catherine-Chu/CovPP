import random
class UAV:
    def __init__(self, pos_x, pos_y, workcap=40, sense_r = 1, global_view = True):
        self.pos = (pos_x, pos_y)
        self.workcap = 40
        self.worktime = 0
        # TODO: Establish relation between sense_r and sense_p
        self.sense_r=sense_r
        self.sense_p = [(1,0),(-1,0),(0,1),(0,-1)]
        self.global_view = global_view
        self.local_view = None
        self.global_view = None
        self.actions=[(1,0),(-1,0),(0,1),(0,-1)]

    def step(self, env, strategy='Avoid_Random'):
        moved = False
        if strategy == 'Blind_Random':
            rand_action = random.choice(self.actions)
            self.pos[0] += rand_action[0]
            self.pos[1] += rand_action[1]
            moved = True

        elif strategy == 'Avoid_Random':
            self.get_local_view(env)
            try_times = 0
            try_actions = []
            while True:
                rand_action = random.choice(self.actions)
                try_p = self.local_view[rand_action]
                if try_p.reachable and not try_p.visited and not try_p.ischargingp:
                    self.pos[0] += rand_action[0]
                    self.pos[1] += rand_action[1]
                    moved = True
                    break
                try_times += 1
                if rand_action not in try_actions:
                    try_actions.append(rand_action)
                if try_times >= 10 and len(try_actions)<4:
                    for i in range(len(self.actions)):
                        try_p = self.local_view[self.actions[i]]
                        if try_p.reachable and not try_p.visited and not try_p.ischargingp:
                            self.pos[0] += rand_action[0]
                            self.pos[1] += rand_action[1]
                            moved = True
                            break
                    break
        return moved

    def get_local_view(self, env):
        self.local_view = {}
        for i in range(len(self.sense_p)):
            sense_pos = (self.pos[0]+self.sense_p[i][0],self.pos[1]+self.sense_p[i][1])
            self.local_view[self.sense_p[i]]=env.Points[sense_pos[0]][sense_pos[1]]

    def get_global_view(self, center):
        self.global_view = center.global_view

    def put_local_view(self, center):
        center.put_view(self.local_view)