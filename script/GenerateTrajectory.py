import numpy as np



class Circle:
    def __init__(self, height, radius_a, radius_b, oval_num, z_change, yaw, velocity, savepath):
        self.radius_a = radius_a
        self.radius_b = radius_b
        self.height = height
        
        self.velocity = velocity
        self.savepath = savepath
        self.trajectory = []
        self.yaw_range = yaw
        
        self.oval_num = oval_num
        self.z_change = z_change
        self.TWO_PI = 2 * np.pi
        
        self.f = open(savepath, 'w')
        print(self.radius_a, self.radius_b)
    
    def generate(self):
        frequency = 200
        per_iter_time = 1 / frequency
        
        per_oval_length = self.TWO_PI * self.radius_b + 4 * (self.radius_a - self.radius_b) 
        per_line_length = self.z_change
        
        total_length = (2 * self.oval_num - 1) * per_oval_length + 2 * (self.oval_num -1) * per_line_length
        
        per_oval_time = per_oval_length / self.velocity
        per_oval_num = int(per_oval_time / per_iter_time)
        
        per_line_time = per_line_length / self.velocity
        per_line_num = int(per_line_time / per_iter_time)
        
        iter_count = (2 * self.oval_num - 1) * per_oval_num + 2 * (self.oval_num -1) * per_line_num 
         
        total_time = total_length /self.velocity
        print(" total time: ", total_time)
        
        
        
        per_oval_iter = 0
        per_line_iter = 0
        traj_state = "CIRCLE"
        line_state = "ADD"
        psi_state = "ADD"
        
        height = self.height
        yaw = 0
        per_height_change = self.z_change / per_line_num
        max_height = self.height + (self.oval_num -1) * self.z_change
        
        for i in range(int(iter_count)):
                
            if(traj_state == "CIRCLE"):
                per_oval_iter += 1
                theta = self.TWO_PI / per_oval_num * per_oval_iter
                position_x = -self.radius_a * np.cos(theta)
                position_y = self.radius_b * np.sin(theta)
                position_z = height
                if(psi_state == "ADD"):
                    yaw = yaw + self.yaw_range / per_oval_num 
                    psi = yaw
                
                if(psi_state == "MINUS"):
                    yaw = yaw - self.yaw_range / per_oval_num
                    psi = yaw
                    
                if(per_oval_iter == per_oval_num):
                    # print("change state")
                    per_oval_iter = 0
                    traj_state = "LINE"
                    
                    if(psi_state == "ADD"):
                        psi_state = "MINUS"
                    else:
                        psi_state = "ADD"
                    
                    if(abs(height - max_height) < 0.1):
                        line_state = "MINUS"
                    continue
                        
            if(traj_state == "LINE"):
                position_x = -self.radius_a
                position_y = 0
                
                per_line_iter = per_line_iter + 1
                if(line_state == "MINUS"):
                    height = height - per_height_change
                    position_z = height
                
                if(line_state == "ADD"):
                    height = height + per_height_change
                    position_z = height
                
                if(per_line_iter == per_line_num):
                    per_line_iter = 0
                    traj_state = "CIRCLE"
                
            current_time = total_time / iter_count * i

            
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
    radius_a = 1.5
    radius_b = 1
    height = 1.2
    oval_num = 5
    z_change = 0.1
    
    velocity = 3
    yaw = np.pi/2
    savepath = "/home/ldd/quarotor_controller/src/quarotor_feedback_controller/library/line_3.txt"
    circle = Circle(height, radius_a, radius_b, oval_num, z_change, yaw, velocity, savepath)
    circle.generate()

if __name__ == '__main__':

    main()