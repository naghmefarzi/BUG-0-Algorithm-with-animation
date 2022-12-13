import random
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd



show_animation=True
number_of_datasets=1
path_to_save_datas='/Users/sergeileito/Documents/data robot/'



class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y):
        self.head_angle=90
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.r_x = [start_x]
        self.r_y = [start_y]
        self.out_x = []
        self.out_y = []
        self.obstacle_points=[]
        for o_x, o_y in zip(obs_x, obs_y):
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                cand_x, cand_y = o_x+add_x, o_y+add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    self.out_x.append(cand_x), self.out_y.append(cand_y)
                   

    def mov_normal(self):
        return self.r_x[-1] + np.sign(self.goal_x - self.r_x[-1]), \
               self.r_y[-1] + np.sign(self.goal_y - self.r_y[-1])

    def mov_to_next_obs(self, visited_x, visited_y):
        for add_x, add_y in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            c_x, c_y = self.r_x[-1] + add_x, self.r_y[-1] + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                if c_x == _x and c_y == _y:
                    for v_x, v_y in zip(visited_x, visited_y):
                        if c_x == v_x and c_y == v_y:
                            use_pt = False
                            break
                    if use_pt:
                        return c_x, c_y, False
                if not use_pt:
                    break
        return self.r_x[-1], self.r_y[-1], True

    def draw_circle(self,robot_vec,theta_s):
        #print('im in draw crcl')
        #print('it turned by '+str(theta_s[-1]))
        a, b = self.r_x[-2], self.r_y[-2]
        exes=[]
        vays=[]
        laser_beam=[]
        obs_laser_det_pos=[]
        obs_laser_det_r_angle=[]
        r_alpha_matrix = 180*[30]
        #print(  r_alpha_matrix) 
        obs_laser_det_angles=[]
        
        delta=self.head_angle
        #print(delta)
        
        
        #if delta-90<=0:
            #f,t=int(theta_s[-1])-90
            #delta+=360
        #f,t=delta, -delta+90
        #print(delta)    

        #print('from'+str(delta-90)+' to' +str(delta+90))    
        for r in range(1,31):
                #print('im in first for')
                #print(  r_alpha_matrix) 
            for indx,angle in enumerate(range(min(delta-180, delta),max(delta-180, delta))):
                #print(angle)
                x = r * math.sin(math.radians(angle)) + a
                y = r * math.cos(math.radians(angle)) + b
                exes.append(int((x)))
                vays.append(int((y)))
                beam=[int((x)),int((y))]
                laser_beam.append(beam)
                if beam in self.obstacle_points:
                    if angle not in obs_laser_det_angles:
                        #obs_laser_det_pos.append([beam[0],beam[1]])
                        exes.append(beam[0])
                        vays.append(beam[1])
                        obs_laser_det_angles.append(angle)
                        obs_laser_det_r_angle.append([r,angle])
                        r_alpha_matrix[indx]=r
                            
        '''else:
            #r_alpha_matrix = 180*[30]
            delta=int(theta_s[-1])
            for r in range(1,31):
                for indx,angle in enumerate(range(delta, -delta+90)):
                    x = r * math.sin(math.radians(angle)) + a
                    y = r * math.cos(math.radians(angle)) + b
                    exes.append(int(round(x)))
                    vays.append(int(round(y)))
                    beam=[int(round(x)),int(round(y))]
                    laser_beam.append(beam)
                    if beam in self.obstacle_points:
                        if angle not in obs_laser_det_angles:
                                #obs_laser_det_pos.append([beam[0],beam[1]])
                            obs_laser_det_angles.append(angle)
                            obs_laser_det_r_angle.append([r,angle])
                            r_alpha_matrix[indx]=r'''
        #print(  r_alpha_matrix)
        r_alpha_matrix=np.array(r_alpha_matrix)/30
        #print(  r_alpha_matrix)
        return exes,vays,r,r_alpha_matrix

    def bug0(self):

        
        data=np.zeros(182)
        header=[]
        for i in range(180):
            header.append('angle'+str(i))
        header.append('theta P')
        header.append('theta S')
        df=pd.DataFrame(columns=header)
        
        laserbeam=[]
        start_point = [self.r_x[0], self.r_y[0]]
        target_point = [self.goal_x, self.goal_y]
        first_theta_p=cal_angle([1,0],[ self.goal_x-self.r_y[0] ,self.goal_y-self.r_y[0]])
        positions=[ start_point ]
        theta_s=[0]
        theta_p=[first_theta_p]
        inertial_vector=[self.r_x[0],self.r_y[0]+1]
        robot_vecs=[[0,1]]
        
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 0')

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x and \
                    self.r_y[-1] == self.goal_y:
                break
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                can_go_normal = True
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    if self.mov_normal()[0] == x_ob and \
                            self.mov_normal()[1] == y_ob:
                        can_go_normal = False
                        break
                if can_go_normal:
                    mov_dir = 'normal'
                else:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
                    visited_x.append(cand_x), visited_y.append(cand_y)

            cur_pos=[self.r_x[-1], self.r_y[-1]]        
            positions.append(cur_pos)
            
            robot_vec_x = self.r_x[-1]-self.r_x[-2]
            robot_vec_y = self.r_y[-1]-self.r_y[-2]
            robot_vec = [robot_vec_x,robot_vec_y]

                  
            robot_vecs.append(robot_vec)
            cur_angle=cal_angle(robot_vecs[-1],robot_vecs[-2])
            self.head_angle=self.head_angle+int(cur_angle)
            theta_s.append(cur_angle)
            
            cur_theta_p = cal_angle([1,0],[ self.goal_x-self.r_y[-1] ,self.goal_y-self.r_y[-1]])
            theta_p.append(cur_theta_p)
            exes, vays,r, obs_laser_det = self.draw_circle(robot_vec,theta_s)
            if show_animation:
                #plt.rc('figure', figsize=(3, 4))
                plt.plot(self.r_x, self.r_y, "-r")
                #plt.pause(0.00001)
                hemisphere,=plt.plot(exes,vays,'-b',alpha=0.1)
                plt.pause(10**-10)
                hemisphere.remove() 
                plt.draw()     
            #exes, vays,r, obs_laser_det = self.draw_circle(robot_vec,theta_s)
            #print(obs_laser_det)
            new_data=np.append(obs_laser_det,np.array([cur_theta_p,theta_s[-1]]))
            data=np.vstack([data,new_data])
            
            #plt.clf()
            #break
        if show_animation:
            plt.show()
            plt.close('all')
                
        df=pd.DataFrame(data,columns=header)
        df['theta S'] = df['theta S'].shift(-1)
        df=df[df['theta P']<0+360]
        df = df.dropna()
        return df

def cal_angle(vector_1,vector_2):
    x1,y1 = vector_1
    x2,y2 = vector_2
    angle=np.arctan2(x1*y2-y1*x2,x1*x2+y1*y2)
    mydegrees = math.degrees(angle)
    return mydegrees



def main():
    # set obstacle positions
    o_x, o_y = [], []



    obstacle_points=[]

    '''for i in range(20, 40):
        for j in range(20, 40):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(70, 80):
        for j in range(60, 100):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(120, 140):
        for j in range(90, 100):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(80, 120):
        for j in range(120, 140):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(0, 20):
        for j in range(80, 120):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(30, 50):
        for j in range(70, 90):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(120, 160):
        for j in range(50, 60):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])'''
    for i in range(20, 40):
        for j in range(20, 40):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(60, 100):
        for j in range(40, 80):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(120, 140):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(80, 140):
        for j in range(0, 20):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(0, 20):
        for j in range(60, 100):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(20, 40):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

    for i in range(120, 160):
        for j in range(40, 60):
            o_x.append(i)
            o_y.append(j)
            obstacle_points.append([i,j])

            
    for i in range(number_of_datasets):            
        s_x = random.randint(0,124)
        s_y = random.randint(0,124)
        while [s_x,s_y] in obstacle_points:
            s_x = random.randint(0,124)
            s_y = random.randint(0,124)
        g_x = random.randint(0,124)
        g_y = random.randint(0,124)            
        while [g_x,g_y] in obstacle_points or [g_x,g_y]==[s_x,s_y] :
            g_x = random.randint(0,124)
            g_y = random.randint(0,124)


    
        print('makeing data'+str(i))
        #try:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.obstacle_points=obstacle_points
        df = my_Bug.bug0()
        '''for indx,tup in enumerate(positions):
                tup.append(theta_s[indx])
                tup.append(theta_p[indx])
                #print(positions)
                return positions'''
        df.to_csv(path_to_save_datas+"data"+str(i)+".csv")
        #except:
            #pass



if __name__ == '__main__':
    main()
    
