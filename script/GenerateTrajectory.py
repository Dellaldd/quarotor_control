import numpy as np

class Circle:
    def __init__(self, radius, velocity, savepath, yaw):
        self.radius = radius
        self.velocity = velocity
        self.savepath = savepath
        self.trajectory = []
        self.yaw_range = yaw
        self.f = open(savepath, 'w')
        print(self.radius)
    
    def generate(self):
        total_time = 2 * np.pi * self.radius / self.velocity
        print(" total time: ", total_time)
        frequency = 200
        iter_count = total_time * frequency
        
        for i in range(int(iter_count)):
            theta = 2 * np.pi / iter_count * i
            print(theta)
            current_time = total_time / iter_count * i
            position_x = self.radius - self.radius * np.cos(theta)
            position_y = -self.radius * np.sin(theta)
            position_z = 1
            psi = self.yaw_range / iter_count * i
            self.trajectory.append([str(current_time), str(position_x), str(position_y), str(position_z), str(psi)])
            
        self.writetitle()
        self.writedata()
        
    def writetitle(self):
        self.f.write("# timestamp tx ty tz psi")
        self.f.write('\r\n') 
    
    def writedata(self):
        for data in self.trajectory:
            self.f.write(' '.join(data))
            self.f.write('\r\n')  
        self.f.close()
        

def main():
    radius = 1
    velocity = 0.5
    yaw = np.pi/2
    savepath = "/home/coolpi/work/openvins/quarotor_control/src/quarotor_feedback_controller/library/circle_yaw.txt"
    circle = Circle(radius, velocity, savepath, yaw)
    circle.generate()

if __name__ == '__main__':

    main()
