import control_cal as cc

import numpy as np
import matplotlib.pyplot as plt
def get_octagon_vertices(radius, init_angle = 0 ):
    vertices = []
    for i in range(8):
        angle = np.pi / 4 * i
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        vertices.append(np.array([x, y]))
    return np.array(vertices)

Outter_walls = get_octagon_vertices(8)
Inner_walls = get_octagon_vertices(5)

# octagon_vertices = get_octagon_vertices(1.5)
# octagon_vertices2 = get_octagon_vertices(2.5)


def plot_world(Inner_walls,Outter_walls, ax ):
    # Unzip the vertices for plotting
    x, y = zip(*Inner_walls)

    # Plot the octagon

    ax.plot(*zip(*Inner_walls, Inner_walls[0]), marker='o') # Connect the vertices and plot them
    ax.scatter(x, y, color='red') # Plot the vertices
    plt.axis('equal')
    
    x, y = zip(*Outter_walls)

    # Plot the octagon
    ax.plot(*zip(*Outter_walls, Outter_walls[0]), marker='o') # Connect the vertices and plot them
    ax.scatter(x, y, color='red') # Plot the vertices
    for i in range(len(Inner_walls)):
        x1, y1 = 0, 0  # Starting point (for example, the origin)
        x2, y2 = 1, 1  # Ending point

        # Plot the line
        ax.plot([Inner_walls[i][0], Outter_walls[i][0]], [Inner_walls[i][1], Outter_walls[i][1]], color= "gray", linestyle= ":")

   
# fig, ax = plt.subplots()
# plot_world(Inner_walls,Outter_walls, ax )
# fig.show()
# plt.show()
    



dt = 0.01

# A = np.zeros((2,2))
A = np.zeros((2,2))
# A = np.ones((2,2))*0.1
B = np.eye((2))

# CD = cc.Discrertized_Linear_Controller(A, B, dt)
# A_dis, B_dis = CD()

class cell ():
    def __init__(self,Barrier, exit_Vertices,vrt, landmark_ls ):
        self.bar = Barrier
        # self.wrd = world
        self.exit_vrt = exit_Vertices
        self.vrt = vrt
        self.landmark_ls = landmark_ls
        # self.other = other


cell_ls = []
num_cell = 8
for ic in range(num_cell-2):
    # cell_tmp = cell(Barrier= [[Outter_walls[ic], Outter_walls[ic+1]], [Inner_walls[ic], Inner_walls[ic+1]],[Outter_walls[ic+1], Outter_walls[ic+2]], [Inner_walls[ic+1], Inner_walls[ic+2]]], exit_Vertices= [Outter_walls[ic+1], Inner_walls[ic+1]], vrt = [Outter_walls[ic], Outter_walls[ic+1], Inner_walls[ic+1], Inner_walls[ic]], landmark_ls=[Outter_walls[ic], Outter_walls[ic+1], Inner_walls[ic+1], Inner_walls[ic]])
    cell_tmp = cell(Barrier= [[Outter_walls[ic], Outter_walls[ic+1]], [Inner_walls[ic], Inner_walls[ic+1]], [Outter_walls[ic+1], Outter_walls[ic+2]] ], exit_Vertices= [Outter_walls[ic+1], Inner_walls[ic+1]], vrt = [Outter_walls[ic], Outter_walls[ic+1], Inner_walls[ic+1], Inner_walls[ic]], landmark_ls=[Outter_walls[ic+1], Inner_walls[ic+1]])
    cell_ls.append(cell_tmp)


#####one landmark
# cell_tmp = cell(Barrier= [[Outter_walls[-2], Outter_walls[-1]], [Inner_walls[-2], Inner_walls[-1]], [Outter_walls[-1], Outter_walls[0]]], exit_Vertices= [Outter_walls[-1], Inner_walls[-1]], vrt = [Outter_walls[-2], Outter_walls[-1], Inner_walls[-1], Inner_walls[-2]], landmark_ls=[Outter_walls[-1]])
# cell_ls.append(cell_tmp)

# cell_tmp = cell(Barrier= [[Outter_walls[-1], Outter_walls[0]], [Inner_walls[-1], Inner_walls[0]], [Outter_walls[0], Outter_walls[1]]], exit_Vertices= [Outter_walls[0], Inner_walls[0]], vrt = [Outter_walls[-1], Outter_walls[0], Inner_walls[0], Inner_walls[-1]], landmark_ls=[Outter_walls[0]])
# cell_ls.append(cell_tmp)






###four landmarks
# cell_tmp = cell(Barrier= [[Outter_walls[-2], Outter_walls[-1]], [Inner_walls[-2], Inner_walls[-1]], [Outter_walls[-1], Outter_walls[0]]], exit_Vertices= [Outter_walls[-1], Inner_walls[-1]], vrt = [Outter_walls[-2], Outter_walls[-1], Inner_walls[-1], Inner_walls[-2]], landmark_ls=[Outter_walls[-1], Outter_walls[-2], Inner_walls[-1], Inner_walls[-2]])
# cell_ls.append(cell_tmp)

# cell_tmp = cell(Barrier= [[Outter_walls[-1], Outter_walls[0]], [Inner_walls[-1], Inner_walls[0]], [Outter_walls[0], Outter_walls[1]]], exit_Vertices= [Outter_walls[0], Inner_walls[0]], vrt = [Outter_walls[-1], Outter_walls[0], Inner_walls[0], Inner_walls[-1]], landmark_ls=[Outter_walls[-1], Outter_walls[0], Inner_walls[0], Inner_walls[-1]])
# cell_ls.append(cell_tmp)




####two landmarks
cell_tmp = cell(Barrier= [[Outter_walls[-2], Outter_walls[-1]], [Inner_walls[-2], Inner_walls[-1]], [Outter_walls[-1], Outter_walls[0]]], exit_Vertices= [Outter_walls[-1], Inner_walls[-1]], vrt = [Outter_walls[-2], Outter_walls[-1], Inner_walls[-1], Inner_walls[-2]], landmark_ls=[Outter_walls[-1], Inner_walls[-1]])
cell_ls.append(cell_tmp)

cell_tmp = cell(Barrier= [[Outter_walls[-1], Outter_walls[0]], [Inner_walls[-1], Inner_walls[0]], [Outter_walls[0], Outter_walls[1]]], exit_Vertices= [Outter_walls[0], Inner_walls[0]], vrt = [Outter_walls[-1], Outter_walls[0], Inner_walls[0], Inner_walls[-1]], landmark_ls=[Outter_walls[0], Inner_walls[0]])
cell_ls.append(cell_tmp)






ch_ls = [10 , 10, 10, 10, 10, 10, 10, 10]
cv_ls = [1 , 1, 1, 1, 1, 1, 1, 1]
i_cell = 2

# s0=cc.Control_cal(cell_ls[i_cell],A, B,dt,ch_ls[i_cell],cv_ls[i_cell], 1, 1.2 )
# # s0.plot_cell()
# # plt.show()
# s0.get_K() 
# # s0.control_gain_ls =  np.load('hexaton_control_gains\Mp_ls'+str(i_cell)+'.npy')
# # s0.Kb =np.load('hexaton_control_gains\Kb_ls'+str(i_cell)+'.npy')
# s0.vector_F(0)
# clf,clf_val, max_clf=s0.CLF_check()
# print(max_clf)
# np.save('hexaton_control_gains\Mp_ls'+str(i_cell)+'.npy',s0.control_gain_ls)
# np.save('hexaton_control_gains\Kb_ls'+str(i_cell)+'.npy', s0.Kb)

for i_cell in range(len(cell_ls)):

    s0=cc.Control_cal(cell_ls[i_cell],A, B,dt,ch_ls[i_cell],cv_ls[i_cell], 1,2 )
    s0.get_K() 
    np.save('hexaton_control_gains\Mp_ls'+str(i_cell)+'.npy',s0.control_gain_ls)
    np.save('hexaton_control_gains\Kb_ls'+str(i_cell)+'.npy', s0.Kb)
    print("**************finshed_cell"+str(i_cell)+"**************")
    s0.vector_F(0)


#     s0.plot_cell()
#     plt.show()
# # 
# i_cell = 0
# s0=cc.Control_cal(cell_ls[i_cell],A, B,dt,ch_ls[i_cell],cv_ls[i_cell], 1,2 )
# s0.get_K() 
# s0.vector_F(0)



    
# # np.save('Mp_ls'+str(i_cell)+'.npy',s0.control_gain_ls)
# # np.save('Kb_ls0'+str(i_cell)+'.npy', s0.Kb)
