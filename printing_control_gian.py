import numpy as np
K_all =np.array([])
for clid in range(4):

    K= np.load('K'+str(clid)+'.npy')
    if clid == 0:


        K_all =K
    else:
        K_all = np.vstack((K_all,K))
    print(K)
    np.savetxt("K"+str(clid)+".csv", K, delimiter=",",fmt="%.4f")

np.savetxt("Kall.csv", np.array(K_all), delimiter=",",fmt="%.4f")