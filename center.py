from environment import *

class Center:
    def __init__(self, view):
        self.global_view = view

    def put_view(self,local_view, old_pos, sense_p):
        for i, key in enumerate(local_view):
            new_p = local_view[key]
            pos_x = old_pos[0]+key[0]
            pos_y = old_pos[1]+key[1]
            # print(key)
            # print("new_p: %s." % new_p)
            if new_p:
                # print("pos_x: %s." % pos_x)
                # print("pos_y: %s." % pos_y)
                self.global_view.Points[pos_x][pos_y] = new_p
