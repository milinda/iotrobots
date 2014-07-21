from lib import libardrone

class Planning():
    def do(self, droneNavData, targets):
        if(len(targets) >= 1):
            if targets[0].found == True:
                x = targets[0].x
                y = targets[0].y
                kX = -(1.0/320*0.1)
                kY = -(-1.0/320*0.1)
                return DroneCommand(kX*x, kY*y, 0, 0)
            else:
                return DroneCommand(0,0,0,0)
        return DroneCommand(0,0,0,0)
    
class DroneCommand():
    def __init__(self,tiltx,tilty,spin,velocity_z):
        self.tiltx = tiltx
        self.tilty = tilty
        self.spin = spin
        self.velocity_z = velocity_z
        
    def do(self, drone):
        print "Tilt X and Y: ",self.tiltx, self.tilty
        drone.at(libardrone.at_pcmd, True, self.tiltx, self.tilty, 0, 0)