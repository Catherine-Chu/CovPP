class Statistic:
    def __init__(self):
        self.timeLimit = 72
        self.timeCost = 0
    def calCoverageRatio(self, points):
        # 默认计算整个系统开始到当前时刻的覆盖率
        # 需要统计不同策略下系统到timeLimit时刻的覆盖率，以及到整个eposido结束的时候的覆盖率
        return 0
    def getTimeLimit(self):
        return self.timeLimit
    def setTimeCost(self,timecost):
        # 需要记录不用策略下从开始到eposido结束所花的时间（不一定是MAX_T）
        self.timeCost=timecost
    def calRedundancy(self, points):
        return 0