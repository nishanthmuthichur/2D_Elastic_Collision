import numpy as np

import matplotlib.pyplot as plt

#*************************MACRO DEFINITIONS************************************

PI = np.pi

X_DIR = 0
Y_DIR = 1

TRUE = 1
FALSE = 0

#*************************DATA STRUCTURE DEFINITIONS***************************

class two_D_ball:
        
    def __init__(self,   ball_idx, \
                         ball_col, \
                        ball_mass, \
                         ball_rad, \
                     pos_vec_init, \
                     vel_vec_init, \
                         time_vec):

        N_time = len(time_vec)
        
        pos_vec_arry = np.zeros([2, N_time])
        vel_vec_arry = np.zeros([2, N_time])
        
        pos_vec_arry[:, 0] = pos_vec_init
        vel_vec_arry[:, 0] = vel_vec_init
    
        self.ball_mass = ball_mass
        self.ball_rad  = ball_rad
        self.pos_vec_arry = pos_vec_arry
        self.vel_vec_arry = vel_vec_arry
        self.time_vec  = time_vec
        self.ball_vis  = plt.Circle((pos_vec_init[0] , \
                                     pos_vec_init[1]), \
                                             ball_rad, \
                                        fc = ball_col)
        
    def __str__(self):
        
        return f"ball_idx = {self.ball_idx}\n"

#******************************************************************************
#**********************FUNCTION DEFINITIONS************************************

def two_D_elastic_collision_init(ball_vec, \
                                  N_balls, \
                                 ball_col, \
                                ball_mass, \
                                 ball_rad, \
                                  vel_max, \
                           two_D_box_leng, \
                                 time_vec):
    
    ball_dia = 2 * ball_rad
    
    init_ball_pos_list = list()
        
    for ball_idx in range(0, N_balls):

        list_len = len(init_ball_pos_list)
  
        pos_vec_init = np.zeros(2)            
        vel_vec_init = np.zeros(2)    
   
        while True:

            pos_x_init = np.random.rand() * (two_D_box_leng[X_DIR] - ball_dia) + ball_rad           
            pos_y_init = np.random.rand() * (two_D_box_leng[Y_DIR] - ball_dia) + ball_rad    
                        
            #vel_init   = np.random.rand() * vel_max
            vel_init   =  vel_max
            theta_init = np.random.rand() * (2 * PI)
            
            vel_x_init = vel_init * np.cos(theta_init)
            vel_y_init = vel_init * np.sin(theta_init)
            
            POS_FLAG = 1                            

            for list_idx in range(0, list_len):
                
                init_ball_pos_x = init_ball_pos_list[list_idx][X_DIR]
                init_ball_pos_y = init_ball_pos_list[list_idx][Y_DIR]
               
                dist = np.sqrt( (pos_x_init - init_ball_pos_x)**2 + \
                                (pos_y_init - init_ball_pos_y)**2 )
                
                if (dist < ball_dia):
            
                    POS_FLAG = 0
                
                    print(f'ball_idx = {ball_idx}:Initial position fix failed! Trying again.')
                    break
                
            if (POS_FLAG == 1):
    
                print(f'ball_idx = {ball_idx}:Initial position fix success!')
                break

        pos_vec_init[0] = pos_x_init
        pos_vec_init[1] = pos_y_init
        
        vel_vec_init[0] = vel_x_init
        vel_vec_init[1] = vel_y_init

        ball_vec.append(two_D_ball(ball_idx, \
                                   ball_col, \
                                  ball_mass, \
                                   ball_rad, \
                               pos_vec_init, \
                               vel_vec_init, \
                                   time_vec))
    
        init_ball_pos_list.append(pos_vec_init)

    return ball_vec
#******************************************************************************
#******************************************************************************
def pos_update_time_step(ball_vec, \
                         time_idx, \
                        time_step):
    
    N_balls = len(ball_vec)

    for ball_idx in range(0, N_balls):

        vel_x = ball_vec[ball_idx].vel_vec_arry[X_DIR, (time_idx - 1)]
        vel_y = ball_vec[ball_idx].vel_vec_arry[Y_DIR, (time_idx - 1)]
        
        pos_x = ball_vec[ball_idx].pos_vec_arry[X_DIR, (time_idx - 1)]
        pos_y = ball_vec[ball_idx].pos_vec_arry[Y_DIR, (time_idx - 1)]        
        
        up_pos_x = pos_x + vel_x * time_step
        up_pos_y = pos_y + vel_y * time_step
        
        up_vel_x = vel_x
        up_vel_y = vel_y
    
        ball_vec[ball_idx].pos_vec_arry[X_DIR, time_idx] = up_pos_x
        ball_vec[ball_idx].pos_vec_arry[Y_DIR, time_idx] = up_pos_y        
                
        ball_vec[ball_idx].vel_vec_arry[X_DIR, time_idx] = up_vel_x
        ball_vec[ball_idx].vel_vec_arry[Y_DIR, time_idx] = up_vel_y
    
    return ball_vec
#******************************************************************************

def vel_update_time_step(ball_vec, \
                   two_D_box_leng, \
                         time_idx):

    ball_vec = vel_update_wall_coll(ball_vec, \
                              two_D_box_leng, \
                                    time_idx)
        
    ball_vec = vel_update_post_coll(ball_vec, \
                                    time_idx)        

    return ball_vec

#******************************************************************************
def vel_update_wall_coll(ball_vec, \
                   two_D_box_leng, \
                         time_idx):
    
    N_balls = len(ball_vec)
    
    B_Lx = two_D_box_leng[X_DIR]
    B_Ly = two_D_box_leng[Y_DIR]
    
# Code to handle collision with the walls.
 
    for ball_idx in range(0, N_balls):
    
        pos_x = ball_vec[ball_idx].pos_vec_arry[X_DIR, time_idx]
        pos_y = ball_vec[ball_idx].pos_vec_arry[Y_DIR, time_idx]        
        
        vel_x = ball_vec[ball_idx].vel_vec_arry[X_DIR, time_idx]
        vel_y = ball_vec[ball_idx].vel_vec_arry[Y_DIR, time_idx]        
        
        ball_rad = ball_vec[ball_idx].ball_rad

        # Bottom wall
        if ( (pos_y < ball_rad)          and \
             (pos_x > ball_rad)          and \
             (pos_x < (B_Lx - ball_rad)) and \
             (vel_y < 0) ):
            
            up_vel_x =  vel_x
            up_vel_y = -vel_y

        # Top wall            
        elif ( (pos_y > (B_Ly - ball_rad)) and \
               (pos_x > ball_rad)          and \
               (pos_x < (B_Lx - ball_rad)) and \
               (vel_y > 0) ):
            
            up_vel_x =  vel_x
            up_vel_y = -vel_y        

        # Left wall
        elif ( (pos_x < ball_rad)          and \
               (pos_y > ball_rad)          and \
               (pos_y < (B_Ly - ball_rad)) and \
               (vel_x < 0) ):
            
            up_vel_x = -vel_x
            up_vel_y =  vel_y

        # Right wall
        elif ( (pos_x > (B_Lx - ball_rad)) and \
               (pos_y > ball_rad)          and \
               (pos_y < (B_Ly - ball_rad)) and \
               (vel_x > 0) ):
            
            up_vel_x = -vel_x
            up_vel_y =  vel_y

        # Bottom left corner
        elif ( (pos_x < ball_rad) and \
             (pos_y < ball_rad)   and \
             (vel_x < 0)          and \
             (vel_y < 0) ):
            
            up_vel_x = -vel_x
            up_vel_y = -vel_y

        # Bottom right corner
        elif ( (pos_x > (B_Lx - ball_rad)) and \
             (pos_y < ball_rad)            and \
             (vel_x > 0)                   and \
             (vel_y < 0) ):
            
            up_vel_x = -vel_x
            up_vel_y = -vel_y

        # Top left corner
        elif ( (pos_x < ball_rad)        and \
             (pos_y > (B_Ly - ball_rad)) and \
             (vel_x < 0)                 and \
             (vel_y > 0) ):
            
            up_vel_x = -vel_x
            up_vel_y = -vel_y

        # Top right corner
        elif ( (pos_x > (B_Lx - ball_rad)) and \
             (pos_y > (B_Ly - ball_rad))   and \
             (vel_x > 0)                   and \
             (vel_y > 0)):
                         
            up_vel_x = -vel_x
            up_vel_y = -vel_y

        else: 

            up_vel_x = vel_x
            up_vel_y = vel_y

        ball_vec[ball_idx].vel_vec_arry[X_DIR, time_idx] = up_vel_x
        ball_vec[ball_idx].vel_vec_arry[Y_DIR, time_idx] = up_vel_y

    return ball_vec
#******************************************************************************

def vel_update_post_coll(ball_vec, \
                         time_idx):  

    N_balls = len(ball_vec)
     
    coll_tab = cons_coll_tab(ball_vec, \
                             time_idx)

    for ball_i_idx in range(1, N_balls):    
      for ball_j_idx in range(0, ball_i_idx):
          
          if (coll_tab[ball_i_idx, ball_j_idx] == 1):
              
              pos_vec_i = ball_vec[ball_i_idx].pos_vec_arry[:, time_idx]
              pos_vec_j = ball_vec[ball_j_idx].pos_vec_arry[:, time_idx]              

              pos_vec_ij = pos_vec_j - pos_vec_i
              coll_vec_long = pos_vec_ij / (np.sqrt(np.inner(pos_vec_ij, pos_vec_ij)))
              
              deg_90_rot_mat = np.array([[0, -1], [1, 0]])
              coll_vec_lat  = deg_90_rot_mat @ coll_vec_long
              
              vel_vec_i = ball_vec[ball_i_idx].vel_vec_arry[:, time_idx]
              vel_vec_j = ball_vec[ball_j_idx].vel_vec_arry[:, time_idx]
  
              mass_i = ball_vec[ball_i_idx].ball_mass
              mass_j = ball_vec[ball_j_idx].ball_mass

              proj_mat = np.array([coll_vec_long, coll_vec_lat])  
              inv_proj_mat = proj_mat.transpose()
              
              proj_vel_vec_i = proj_mat @ vel_vec_i
              proj_vel_vec_j = proj_mat @ vel_vec_j

              long_vel_i = proj_vel_vec_i[0]
              long_vel_j = proj_vel_vec_j[0]
            
              up_long_vel_i = (2 * mass_j * long_vel_j + (mass_i - mass_j) * long_vel_i) \
                                                / \
                                         (mass_i + mass_j)
              
              up_long_vel_j = (2 * mass_i * long_vel_i + (mass_j - mass_i) * long_vel_j) \
                                                /  \
                                         (mass_i + mass_j)
    
              proj_vel_vec_i[0] = up_long_vel_i
              proj_vel_vec_j[0] = up_long_vel_j

              up_vel_vec_i = inv_proj_mat @ proj_vel_vec_i
              up_vel_vec_j = inv_proj_mat @ proj_vel_vec_j
    
              ball_vec[ball_i_idx].vel_vec_arry[:, time_idx] = up_vel_vec_i
              ball_vec[ball_j_idx].vel_vec_arry[:, time_idx] = up_vel_vec_j
    
    return ball_vec

#******************************************************************************

def cons_coll_tab(ball_vec, \
                  time_idx):

    N_balls = len(ball_vec)
    
    coll_tab = np.zeros([N_balls, N_balls])
    
    for ball_i_idx in range(1, N_balls):
      for ball_j_idx in range(0, ball_i_idx):
    
          pos_vec_i = ball_vec[ball_i_idx].pos_vec_arry[:, time_idx]
          pos_vec_j = ball_vec[ball_j_idx].pos_vec_arry[:, time_idx]
          
          vel_vec_i = ball_vec[ball_i_idx].vel_vec_arry[:, time_idx]
          vel_vec_j = ball_vec[ball_j_idx].vel_vec_arry[:, time_idx] 

          ball_rad_i = ball_vec[ball_i_idx].ball_rad
          ball_rad_j = ball_vec[ball_j_idx].ball_rad
 
          delta_dist = np.sqrt( (pos_vec_i[X_DIR] - pos_vec_j[X_DIR])**2 + \
                                (pos_vec_i[Y_DIR] - pos_vec_j[Y_DIR])**2 )
 
          coll_dist = (ball_rad_i + ball_rad_j)  

          pos_vec_ij = (pos_vec_j - pos_vec_i)
          
          rel_vel_vec_ij = (vel_vec_j - vel_vec_i)
            

          if ( (delta_dist < coll_dist) and \
               (np.inner(pos_vec_ij, rel_vel_vec_ij) < 0) ):

                coll_tab[ball_i_idx, ball_j_idx] = TRUE

    
    return coll_tab
