from environment import *

class Center:
    def __init__(self, view):
        self.global_view = view

    def put_view(self,local_view, old_pos, sense_p, visit_pos=None, visit_time=0, moved=True):
        for i,key in enumerate(local_view):
            new_p = local_view[key]
            pos_x = old_pos[0]+sense_p[i][0]
            pos_y = old_pos[1]+sense_p[i][1]
            if not new_p:
                self.global_view.Points[pos_x][pos_y] = new_p
        if moved:
            next_p = self.global_view.Points[visit_pos[0]][visit_pos[1]]
            if not next_p.ischargingp and next_p.reachable:
                if next_p.timespent + visit_time >= next_p.timecost:
                    next_p.visited = True
            if new_p.ischargingp and visit_time > 0:
                if next_p.stop_num < next_p.cap:
                    next_p.stop_num += 1
            self.global_view.Points[visit_pos[0]][visit_pos[1]] = next_p