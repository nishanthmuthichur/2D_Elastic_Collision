# SCRIPT DESCRIPTION
#
# This script is used to simulate the 2D elastic collision of N balls
#
# Initialization
# time_step
#   Update_position_accordingly
#   Construct_coll_tables 
#   Update_velocities_accordingly

#**************************IMPORTING MODULES***********************************
import numpy as np

import two_D_elastic_collision_lib as two_D

import matplotlib.pyplot as plt
import matplotlib.animation as anim

#**************************MACRO DEFINITIONS***********************************

X_DIR = two_D.X_DIR
Y_DIR = two_D.Y_DIR
PI    = two_D.PI

#**************************INPUT PARAMETERS************************************

N_balls = 50

vel_max   = 20
ball_mass = 1
ball_rad  = 0.5
ball_dia  = 2 * ball_rad

time_start = 0
time_step  = (0.1 * ball_rad)/(vel_max)
N_time     = 2001

two_D_box_leng = (40, 20)

col_blue = (0, 0.25, 0.75)

#**************************COMPUTATION SECTION*********************************

time_end = (N_time - 1) * time_step
time_vec = np.linspace(time_start, time_end, N_time)

ball_vec = list()

ball_vec = two_D.two_D_elastic_collision_init(ball_vec, \
                                               N_balls, \
                                              col_blue, \
                                             ball_mass, \
                                              ball_rad, \
                                               vel_max, \
                                        two_D_box_leng, \
                                              time_vec)
        
for time_idx in range(1, N_time):
    
    ball_vec = two_D.pos_update_time_step(ball_vec, \
                                          time_idx, \
                                         time_step)
        
    ball_vec = two_D.vel_update_time_step(ball_vec, \
                                    two_D_box_leng, \
                                          time_idx)    

#***********************VISUALIZATION OF RESULTS*******************************

fig, ax = plt.subplots(1, 1) 

fig.set_dpi(300)

B_Lx = two_D_box_leng[X_DIR]
B_Ly = two_D_box_leng[Y_DIR]

ax.set_xlim(0, B_Lx)
ax.set_ylim(0, B_Ly)

ax.set_aspect(1)

vis_list = list()
    
def anim_init():
    
    N_balls = len(ball_vec)
    
    for ball_idx in range(0, N_balls):
        
        pos_x_init = ball_vec[ball_idx].pos_vec_arry[X_DIR, 0]
        pos_y_init = ball_vec[ball_idx].pos_vec_arry[Y_DIR, 0]        
        
        ball_vec[ball_idx].ball_vis.center = (pos_x_init, pos_y_init)
    
        ax.add_patch(ball_vec[ball_idx].ball_vis)
    
        vis_list.append(ball_vec[ball_idx].ball_vis)
    
    return vis_list

def animate(time_idx):
 
    N_balls = len(ball_vec)
    
    for ball_idx in range(0, N_balls):
        
        pos_x = ball_vec[ball_idx].pos_vec_arry[X_DIR, time_idx]
        pos_y = ball_vec[ball_idx].pos_vec_arry[Y_DIR, time_idx]    
    
        vis_list[ball_idx].center = (pos_x, pos_y)

        ax.add_patch(vis_list[ball_idx])

    return vis_list

anim_obj = anim.FuncAnimation(             fig, \
                                        animate, \
                          init_func = anim_init, \
                                frames = N_time, \
                                  interval = 10, \
                                    blit = True)

plt.show()
        
#************************SNAPSHOT VISUALIZATION OF THE DATA********************
        
# fig, ax = plt.subplots(1, 1)

# fig.set_dpi(300)

# ax.set_xlim(0, two_D_box_leng[0])
# ax.set_ylim(0, two_D_box_leng[1])

# ax.set_aspect(1)

# for ball_idx in range(0, N_balls):

#     ax.add_patch(ball_vec[ball_idx].ball_vis)
    
# plt.show()    




print('Execution complete\n')

    
    
    

