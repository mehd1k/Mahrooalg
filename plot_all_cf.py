

import numpy as np
import matplotlib.pyplot as plt
import math
# from shapely.geometry import Point, Polygon
from scipy.spatial import Delaunay
import cvxpy as cp
import gurobipy as gp
from gurobipy import GRB
import mosek
import sys
from scipy.spatial import Delaunay
import  scipy

       

class Discrertized_Linear_Controller():
    def __init__(self, A, B, dt):
        self.A = A
        self.B = B
        self.dt = dt
        self.A_dis = np.zeros_like(A)
        self.B_dis = np.zeros_like(B)
    def A_discretized_calculator(self):
        A_dis = np.eye(len(self.A))
        A_n = A_dis
        for n in range(20):
            A_n = self.A@A_n
            new_trem = 1/math.factorial(n)*A_n*self.dt**n
            A_dis = A_dis + new_trem
        self.A_dis = A_dis
    
    
    def B_discretized_calculator(self):
        B_dis = self.B*self.dt
        B_n = B_dis
        for n in range(2,20):
            B_n = self.A@B_n
            new_trem = 1/math.factorial(n)*B_n*self.dt**n
            B_dis = B_dis + new_trem
        self.B_dis = B_dis

    def __call__(self) :
        self.A_discretized_calculator()
        self.B_discretized_calculator()
        return self.A_dis, self.B_dis



def vectorize_matrix(P):
    ny = np.shape(P)[0]
    nx = np.shape(P)[1]
    out = np.zeros((int(nx*ny), 1))
    for iy in range(ny):
        for ix in range(nx):
            out[int(iy*nx+ix)] = P[iy, ix]
   
    return out



def polygon(x_pos, Ax, bx):
    c = Ax@x_pos+np.reshape(bx, (-1,1) )
    if all(c<=0):
        return 1
    else:
        return 0







class Control_cal():
    def __init__(self, cell, A, B, dt,ch,cv):
        
    
        self.cell = cell
        self.nl = len(cell.landmark_ls)
       
        self.ch = ch
        self.cv = cv
        self.o = np.reshape(cell.exit_vrt[1],(2,1))

        
        
        self.vrt = np.array(cell.vrt)
        self.xmin = np.min(self.vrt[:,0])
        self.ymin = np.min(self.vrt[:,1])
        self.xmax = np.max(self.vrt[:,0])
        self.ymax = np.max(self.vrt[:,1])
        
        
        self.center = np.mean(self.vrt, axis=0)
        
        
        self.A = A
        self.B = B
        self.dt = dt
        lm = []
        for il in range(self.nl):
            lm.append(np.reshape(self.cell.landmark_ls[il], (2,1)))
        self.l = lm
        self.Ax, self.bx = self.get_Axbx()
        
        
        self.AH,  self.bH = self.get_Ahbh()
        self.get_LI()

        self.v = self.get_v()
        self.grid = []

    def get_LI(self):
        L_ls = np.zeros((2*self.nl,1))
        for il in range(self.nl):
            L_ls[2*il] = self.cell.landmark_ls[il][0]
            L_ls[2*il+1] = self.cell.landmark_ls[il][1]
        self.L_ls = L_ls
        self.Ic = np.eye(2)
        for il in range(self.nl-1):
            self.Ic= np.vstack((self.Ic,np.eye(2)))
        # print(self.Ic)
        # print(self.L_ls)
        
   

 

        
        
    def plot_cell(self):
        fig, ax = plt.subplots() 
            
        v0 = [-1889,-1738]
        v1 = [-2152, -3126	]
        v2 = [-1594, -4985]
        v3 = [834,	-5226]
        v4 = [2718,	-5093]
        v5 = [3347,	-3349]
        v6 = [2829,	-2003]
        v7 = [591,	-1611]
        v_ls = np.array([v0, v1, v2, v3, v4, v5, v6, v7])*10**-3
                
        for i in range(len(v_ls)-1):
            ax.plot([v_ls[i,0], v_ls[i+1,0]], [v_ls[i,1], v_ls[i+1,1]], color = 'black')
                
        ax.plot([v_ls[0,0], v_ls[-1,0]], [v_ls[0,1], v_ls[-1,1]], color = 'black')
            
        ax.plot([center[0], v_ls[5,0]], [center[1],v_ls[5,1]], color = 'blue')
        ax.plot([center[0], v_ls[7,0]], [center[1],v_ls[7,1]], color = 'blue')
        ax.plot([center[0], v_ls[1,0]], [center[1],v_ls[1,1]], color = 'blue')
        ax.plot([center[0], v_ls[3,0]], [center[1],v_ls[3,1]], color = 'blue')
                # ax.scatter(center[0], center[1])
        # fig, ax = plt.subplots()
        for i in range(len(self.vrt)-1):
            ax.plot([self.vrt[i,0], self.vrt[i+1,0]], [self.vrt[i,1], self.vrt[i+1,1]], color = 'black')
        
        ax.plot([self.vrt[0,0], self.vrt[-1,0]], [self.vrt[0,1], self.vrt[-1,1]], color = 'black')
        for wall in (self.cell.bar):
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax.plot([self.cell.exit_vrt[0][0], self.cell.exit_vrt[1][0]], [self.cell.exit_vrt[0][1], self.cell.exit_vrt[1][1]], color = 'green')

    def get_Axbx(self):
        Ax = []
        bx = [] 
      
        for iv in range(len(self.vrt)-1):
            Ai = np.array([-self.vrt[iv+1][1]+self.vrt[iv][1],  self.vrt[iv+1][0]-self.vrt[iv][0]])
            bi = self.vrt[iv][0]*self.vrt[iv+1][1] - self.vrt[iv][1]*self.vrt[iv+1][0]
            
            # check1 = Ai@np.array(self.vrt[iv+1])+bi
            # check2 = Ai@np.array(self.vrt[iv])+bi
            
            if Ai@self.center+bi > 0 :
                Ai = -Ai
                bi = -bi
            Ax.append(Ai)
            bx.append(bi)
        
        
        ###last and first vertices
        Ai = np.array([-self.vrt[-1][1]+self.vrt[0][1],  self.vrt[-1][0]-self.vrt[0][0]])
        bi = self.vrt[0][0]*self.vrt[-1][1] - self.vrt[0][1]*self.vrt[-1][0]
        
        # check1 = Ai@np.array(self.vrt[iv+1])+bi
        # check2 = Ai@np.array(self.vrt[iv])+bi
        
        if Ai@self.center+bi > 0 :
            Ai = -Ai
            bi = -bi
        Ax.append(Ai)
        bx.append(bi)
       
        return np.array(Ax),np.reshape(np.array(bx),(-1,1))
    

    def get_Ahbh(self):
        Ah = []
        bH = [] 
        for seg in self.cell.bar: 
            x1,y1,x2,y2 = seg[0][0],seg[0][1],seg[1][0],seg[1][1]
            Ai = np.array([y1-y2, -(x1-x2)])
            ln = np.linalg.norm(Ai)
            Ai = Ai/ln
            bi = y1*(x1-x2)-x1*(y1-y2)
            bi = bi/ln
            # check1 = Ai@np.array(self.vrt[iv+1])+bi
            # check2 = Ai@np.array(self.vrt[iv])+bi
            
            if Ai@self.center+bi< 0 :
                Ai = -Ai
                bi = -bi
            Ah.append(Ai)
            bH.append(bi)

                
       
  
       
        return np.array(Ah),np.reshape(np.array(bH),(-1,1))
    
    def u(self, x):
        u = self.K@(self.L_ls-self.Ic@x)
        return u

    
    
   
    def vector_F(self,ax):
        
        
        CD = Discrertized_Linear_Controller(self.A, self.B, self.dt)
        A_dis, B_dis = CD()
        X  = np.linspace(self.xmin,self.xmax,20)
        Y = np.linspace(self.ymin,self.ymax,20)
        ux_ls = []
        uy_ls = []
      
        xg =[]
        yg =[]
        for ix in range(len(X)):
            for iy in range(len(Y)):
                x =X[ix]
                y = Y[iy]
                x_old = np.array([[x],[y]])
                if all(self.Ax@x_old+self.bx <0):
                    u =self.u(x_old)
                
                    uc = np.copy(u)
                    # u = self.K @(self.l-x_old)
                    # lest = U@vectorize_matrix(Po) + x_old
                    # lest = np.array([ [np.sum(np.multiply(self.tx,Po))], [np.sum(np.multiply(self.ty,Po))] ]) + x_old
                    # self.cpox[iy,ix] = lest[0]
                    # self.cpoy[iy,ix] = lest[1]
                    
                    
                    ux_ls.append(uc[0]) 
                    uy_ls.append(uc[1])
                    xg.append(x)
                    yg.append(y)
        # fig, ax = plt.subplots()        
        for i in range(len(self.vrt)-1):
            ax.plot([self.vrt[i,0], self.vrt[i+1,0]], [self.vrt[i,1], self.vrt[i+1,1]], color = 'black')
        
        ax.plot([self.vrt[0,0], self.vrt[-1,0]], [self.vrt[0,1], self.vrt[-1,1]], color = 'black')
        for wall in (self.cell.bar):
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax.plot([self.cell.exit_vrt[0][0], self.cell.exit_vrt[1][0]], [self.cell.exit_vrt[0][1], self.cell.exit_vrt[1][1]], color = 'green')
        ax.quiver(xg,yg,ux_ls,uy_ls,angles='xy', scale_units='xy')
        # np.save('ux_ls.npy',ux_ls)
        # np.save('uy_ls.npy',uy_ls)
        # print("uy_min", np.min(np.abs(uy_ls)), "uxmin", np.min(np.abs(ux_ls)))
        # # fig.show()
        # fname = 'vec_field_mod'+'ch_'+str(self.ch)+'cv_'+str(self.cv)+'.png'
        # plt.show()
        # fig.savefig(fname, dpi = 600)
    
    
        
    def get_v(self):
        ###normal vector to the exit face
        t_exit = self.cell.exit_vrt[0]-self.cell.exit_vrt[1]
        n_exit = np.array([-t_exit[1], t_exit[0]])
        V = n_exit @(self.center- self.cell.exit_vrt[0])
        if V < 0 :
            n_exit = -n_exit
        return np.reshape(np.array(n_exit)/np.linalg.norm(n_exit), (1,2))
    
    
    def check_in_polygon(self, p):
        """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
        p = np.reshape(p,(1,2))
        from scipy.spatial import Delaunay
        if not isinstance(self.vrt,Delaunay):
            hull = Delaunay(self.vrt)

        return (hull.find_simplex(p)>=0)[0]


    def get_K(self, cbf_lb=-0.1, clf_lb = -0.01):
        ###calcualting gains with LP
      
      
        wh =1
        wv =1
        ncbf =  len(self.bH)
        m = gp.Model()
        # ###Defining the Optimization problem
        control_lim = 5
        K = m.addMVar((2,int(self.nl*2)), ub = control_lim, lb = -control_lim, name = 'K')     
        infval = 10**6
        dv = m.addVar(ub = clf_lb, lb = -infval, name = 'dv')
        dh = m.addVar(ub = cbf_lb, lb = -infval, name = 'dh')
        x_mid_ext_face = 1/3*(2*self.cell.exit_vrt[0]+self.cell.exit_vrt[1])
        x_mid_ext_face = np.reshape(x_mid_ext_face,(2,1))
        m.addConstr(self.A@x_mid_ext_face+self.B@K@(self.L_ls-self.Ic@x_mid_ext_face)==0)
        # ex_ = np.array([1,0])
        # x_ = np.array([[0.7], [-4.8]])
        # m.addConstr(ex_@(self.A@x_+self.B@K@(self.L_ls-self.Ic@x_))>=0.2)
        m.addConstr(dh<= dv)
       

          
        #### CLF related vars
        lx = m.addMVar((len(self.bx)), lb= 0, ub = infval, name='lambda_x_v')    
        rv= (self.cv*self.v@self.o)[0,0]-(self.v@self.B@K@self.L_ls)[0,0]
        
        Mxv = self.v@self.A+self.cv*self.v-self.v@self.B@K@self.Ic
        m.addConstr((-lx@self.bx)[0]<= dv+rv)
        m.addConstr(lx@self.Ax-Mxv[0] == 0 )



        # ###########CBF vars
        lxh = m.addMVar((ncbf,len(self.bx)), lb= 0, ub = infval, name='lambda_x_v')
        ########CBF constraints
        for ih in range(ncbf):
        # ih = 0

            rh = (self.ch*self.bH[ih]+self.AH[ih]@self.B@K@self.L_ls)[0]
            Mxh = -self.AH[ih]@self.A-self.ch*self.AH[ih]+self.AH[ih]@self.B@K@self.Ic
            m.addConstr((-lxh[ih]@self.bx)[0]<= dh+rh)
            m.addConstr(lxh[ih]@self.Ax-Mxh == 0 )

                
    
        m.update()
        m.setObjective(wv*dv+wh*dh,  GRB.MINIMIZE)
        m.update()
        # m.params.NonConvex = 2
        m.optimize()
        print("dh=", dh.X, "dv=", dv.X)

       
    
        
        print('k=', K.X)
        # self.Kb = Kb.X
        self.K = K.X

        







# CD = Discrertized_Linear_Controller(A, B, dt)
# A_dis, B_dis = CD()

class cell ():
    def __init__(self,Barrier, exit_Vertices,vrt, landmark_ls ):
        self.bar = Barrier
        # self.wrd = world
        self.exit_vrt = exit_Vertices
        self.vrt = vrt
        self.landmark_ls = landmark_ls
   



# 0.423	-3.53
# 0.1498	-1.9649
# 1.8546	-1.7083
# 3.1034	-2.2849
# 3.55	-3.453
# 3.0852	-4.653
# 1.574	-5.2199
# 0.475	-4.644

v0 = [-1889,-1738]
v1 = [-2110, -3110	]
v2 = [-1594, -4985]
v3 = [630,	-5116]
v4 = [2718,	-5093]
v5 = [3347,	-3349]
v6 = [2829,	-2003]
v7 = [570,	-1689]
v_ls = np.array([v0, v1, v2, v3, v4, v5, v6, v7])*10**-3
center = np.mean(v_ls, axis=0)
sc = 1
o0 = center+ 0.9*sc*(v_ls[1]-center)/np.linalg.norm((v_ls[1]-center))
o1 = center+ 0.5*sc*(v_ls[3]-center)/np.linalg.norm((v_ls[3]-center))
o2 = center+ 1*sc*(v_ls[5]-center)/np.linalg.norm((v_ls[5]-center))
o3 = center+ 1*sc*(v_ls[7]-center)/np.linalg.norm((v_ls[7]-center))
o_ls = np.array([o0, o1, o2, o3])

c0 = cell(Barrier=[[v_ls[7],v_ls[0]], [v_ls[1], v_ls[0]], [v_ls[7], center], [o_ls[0],o_ls[3]] ],exit_Vertices = [v_ls[1], o_ls[0]], vrt = [v_ls[7], v_ls[0], v_ls[1], o_ls[0], o_ls[3]], landmark_ls=[v_ls[0], v_ls[1],v_ls[2], v_ls[3],v_ls[4],v_ls[5],v_ls[6], v_ls[7]])
c1 = cell(Barrier=[[v_ls[1],v_ls[2]], [v_ls[2], v_ls[3]], [v_ls[1], center], [o_ls[0], o_ls[1]]],exit_Vertices = [v_ls[3], o_ls[1]], vrt = [v_ls[1], v_ls[2], v_ls[3], o_ls[1], o_ls[0]], landmark_ls=[v_ls[0], v_ls[1],v_ls[2], v_ls[3],v_ls[4],v_ls[5],v_ls[6], v_ls[7]])
c2 = cell(Barrier=[[v_ls[3],v_ls[4]], [v_ls[4], v_ls[5]], [v_ls[3], center],[o_ls[1], o_ls[2]]],exit_Vertices = [v_ls[5], o_ls[2]], vrt = [v_ls[3], v_ls[4], v_ls[5], o_ls[2], o_ls[1]], landmark_ls=[v_ls[0], v_ls[1],v_ls[2], v_ls[3],v_ls[4],v_ls[5],v_ls[6], v_ls[7]])
c3 = cell(Barrier=[[v_ls[5],v_ls[6]], [v_ls[6], v_ls[7]], [v_ls[5], center], [o_ls[2], o_ls[3]]],exit_Vertices = [v_ls[7], o_ls[3]], vrt = [v_ls[5], v_ls[6], v_ls[7], o_ls[3], o_ls[2]], landmark_ls=[v_ls[0], v_ls[1],v_ls[2], v_ls[3],v_ls[4],v_ls[5],v_ls[6], v_ls[7]])




cell_ls = [c0,c1,c2,c3]       

A = np.zeros((2,2))
B = np.eye((2))
dt = 0.01

fig, ax = plt.subplots()

        
for i in range(len(v_ls)-1):
    ax.plot([v_ls[i,0], v_ls[i+1,0]], [v_ls[i,1], v_ls[i+1,1]], color = 'black')
        
ax.plot([v_ls[0,0], v_ls[-1,0]], [v_ls[0,1], v_ls[-1,1]], color = 'black')
    
ax.plot([center[0], v_ls[5,0]], [center[1],v_ls[5,1]], color = 'blue')
ax.plot([center[0], v_ls[7,0]], [center[1],v_ls[7,1]], color = 'blue')
ax.plot([center[0], v_ls[1,0]], [center[1],v_ls[1,1]], color = 'blue')
ax.plot([center[0], v_ls[3,0]], [center[1],v_ls[3,1]], color = 'blue')
ch_ls = [1,1,1,1,1,10,10,10,10,10]
cv_ls = [1,1,1,1,1,1,1,1,1,1]
i_cell = 1
for i_cell in range(len(cell_ls)):

    s0=Control_cal(cell_ls[i_cell],A, B,dt,ch= 1, cv = 1)
    s0.K = np.load('K'+str(i_cell)+'.npy') 
    # s0.plot_cell()
    plt.axis('equal')
    # plt.show()
    s0.vector_F(ax)
# np.save('K'+str(i_cell)+'.npy',s0.K)

plt.show()

# # for i_cell in range(len(cell_ls)):
# #     s0=Control_cal(cell_ls[i_cell],A, B,dt,ch_ls[i_cell],cv_ls[i_cell] )
# #     s0.get_K_dualAllV() 
# #     Mp_ls.append(s0.Mp)
# #     Kb_ls.append(s0.Kb)

    



# Mp =np.load('Mp_ls'+str(i_cell)+'.npy')
# Kb =np.load('Kb_ls0'+str(i_cell)+'.npy')
# s0.Mp = Mp
# s0.Kb = Kb

# s0.vector_F(0)

# cbfb, cbfval, cbfmax = s0.CBF_check()

# print('maxcbf', cbfmax)
# print(cbfb)
# # print(cbfval)
# x = np.array([[79],[61]])
# ih=0
# szp = int(s0.gs[0]*s0.gs[1])
# # s0.CBF_primal_givenX(s0.Kb, s0.Mp, x, ih)


# z0 = s0.txT-(s0.l-x)[0,0]*np.ones(szp)
# z1 =  s0.tyT-(s0.l-x)[1,0]*np.ones(szp)
# z = np.array([-z0,-z1])
# # s0.primial1(x,z,s0.Mp,s0.Kb, ih)
# # s0.dual1(x,z, ih)
# # s0.cbf_dual(s0.Kb,s0.Mp,ih)




# szp = int(s0.gs[0]*s0.gs[1])
# z0 = s0.txT-(s0.l-x)[0,0]*np.ones(szp)
# z1 =  s0.tyT-(s0.l-x)[1,0]*np.ones(szp)
# z = np.array([-z0,-z1])

# s0.primial1(x,z,s0.Mp,s0.Kb,ih)
# s0.dual1(x,z, ih)
# s0.cbf_dual(s0.Kb, s0.Mp, ih)