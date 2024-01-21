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
        
        oval_length = self.TWO_PI * self.radius_b + 4 * (self.radius_a - self.radius_b)         
        line_length = self.z_change
        
        per_length = np.sqrt(oval_length * oval_length + line_length * line_length)
        per_length_time = per_length / self.velocity
        per_length_num = int(per_length_time / per_iter_time)
        per_height_change = self.z_change / per_length_num
        per_yaw_change = self.yaw_range / per_length_num
        
        total_length = 2 * (self.oval_num - 1) * per_length
        total_count = 2 * (self.oval_num - 1) * per_length_num 
         
        total_time = total_length /self.velocity
        print(" total time: ", total_time)

        per_iter = 0
        line_state = "ADD"
        psi_state = "ADD"
        
        height = self.height
        yaw = 0
        max_height = self.height + (self.oval_num -1) * self.z_change
        
        for i in range(int(total_count)):
            
            theta = self.TWO_PI / per_length_num * per_iter
            position_x = -self.radius_a * np.cos(theta)
            position_y = self.radius_b * np.sin(theta)
            position_z = height
            psi = yaw
            
            if(psi_state == "ADD"):
                yaw = yaw + per_yaw_change
                
            
            if(psi_state == "MINUS"):
                yaw = yaw - per_yaw_change
                
            
            if(line_state == "ADD"):
                height = height + per_height_change 
            
            if(line_state == "MINUS"):
                height = height - per_height_change

                
            if(per_iter == per_length_num):
                per_iter = 0
                if(psi_state == "ADD"):
                    psi_state = "MINUS"
                else:
                    psi_state = "ADD"
                
                if(abs(height - max_height) < 0.01):
                    line_state = "MINUS"
                continue

            per_iter += 1
                
            current_time = total_time / total_count * i     
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
    radius_a = 1.4
    radius_b = 0.9
    height = 1.2
    oval_num = 5
    z_change = 0.1
    
    velocity = 3
    yaw = 0
    savepath = "/home/ldd/quarotor_controller/src/quarotor_feedback_controller/library/small_traj_3.txt"
    circle = Circle(height, radius_a, radius_b, oval_num, z_change, yaw, velocity, savepath)
    circle.generate()

if __name__ == '__main__':

    main()