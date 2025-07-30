class Pillar:
    def __init__(self, area, dist, x, y, target): 
        self.area = area #pillar area
        self.dist = dist #pillar distance from bottom middle point of screen
        self.x = x #pillar x
        self.y = y #pillar y
        self.target = target #stores either target of green pillars or target of red pillarss
        self.w = 0
        self.h = 0
    
    def setDimentions(self, w, h):
        self.w = w
        self.h = h