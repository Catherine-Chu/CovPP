from environment import *
class Center:
    def __init__(self):
        self.global_view = Env(scale=(20,20), initMode='View')
    def put_view(self,local_view):
        self.global_view.Points = local_view.Points