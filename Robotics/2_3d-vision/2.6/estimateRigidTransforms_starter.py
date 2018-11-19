import numpy as np
import math3d as m3d

P = np.array([[ 0.85715536,  0.19169091,  0.37547468],
            [ 0.36011534,  0.51080372,  0.15273612],
            [ 0.76553819,  0.96956225,  0.07534131],
            [ 0.99562617,  0.51500125,  0.66700672],
            [ 0.75873823,  0.57802293,  0.68742226],
            [ 0.40803516,  0.86300463,  0.84899731],
            [ 0.3681771 ,  0.11123281,  0.56888489],
            [ 0.42113594,  0.01455699,  0.02298089],
            [ 0.41690264,  0.49362725,  0.12992772],
            [ 0.7648861 ,  0.99328892,  0.131986  ],
            [ 0.24313837,  0.49029503,  0.45695072],
            [ 0.52637675,  0.97293938,  0.55972334],
            [ 0.84064045,  0.1832065 ,  0.82720688],
            [ 0.82169952,  0.66711466,  0.32593019],
            [ 0.24323087,  0.1992479 ,  0.1700658 ],
            [ 0.6131605 ,  0.51652155,  0.30193314],
            [ 0.1025926 ,  0.87449354,  0.2651951 ],
            [ 0.8714256 ,  0.46842985,  0.84525766],
            [ 0.83809381,  0.28522291,  0.06991397],
            [ 0.76453123,  0.97233898,  0.93372074]])

Q = np.array([[ 0.40901504,  0.83198029,  0.87697249],
            [ 0.24356397,  0.24470076,  0.71503754],
            [-0.18577546,  0.30922866,  1.15358153],
            [ 0.43260626,  0.75349715,  1.3263927 ],
            [ 0.49888288,  0.52382659,  1.26839902],
            [ 0.60616086,  0.07094228,  1.38545005],
            [ 0.7710987 ,  0.47837833,  0.71313771],
            [ 0.3663448 ,  0.58408716,  0.34549392],
            [ 0.21444126,  0.30117119,  0.71677242],
            [-0.15225962,  0.29424925,  1.20281878],
            [ 0.53620743,  0.15779029,  0.83104238],
            [ 0.28042234,  0.10685548,  1.33672152],
            [ 0.77861652,  0.81812   ,  1.13699047],
            [ 0.14504543,  0.52761744,  1.13663154],
            [ 0.45264174,  0.33027575,  0.4701034 ],
            [ 0.27256995,  0.44559883,  0.9274591 ],
            [ 0.24052756, -0.17745923,  0.89671581],
            [ 0.6404062 ,  0.67733176,  1.34604188],
            [ 0.12568749,  0.76576281,  0.74348215],
            [ 0.4966933 ,  0.29664203,  1.67406394]])

def findTransform(P,Q):
    '''
    Finds best rigid body transform between 2 sets of points
    '''

    # Here will implement the rigid body transform algorithm as described here: 
    # http://manipulation.csail.mit.edu/pset2/pset2_ICP.html
    # This is a modified part of the ICP algorithm from a pset I completed earlier in the semester

    # First calulate means
    mu_q = np.mean(Q, axis=0)
    mu_p = np.mean(P, axis=0)

    # Next center the points
    v1 = np.array([np.array(Q[i]-mu_q) for i in range(len(Q))])
    v2 = np.array([np.array(P[i]-mu_p) for i in range(len(P))])
    
    # Multiply and sum to get matrix W
    v1 = v1[:,:,np.newaxis]
    v2 = v2[:,:,np.newaxis]
    v3 = [v1[i,]*v2[i,].T for i in range(len(v1))]
    W = np.sum(v3, axis=0)
    W = W.reshape((3,3))

    # Compute SVD for calculation of R
    U, S, V = np.linalg.svd(W)
    R = np.matmul(U,V)
    # Make sure that R does not give householder reflection matrix
    if np.linalg.det(R) == -1:
        V[2] = V[2]*-1
        R = np.matmul(U,V)

    # Make sure they are the right shape and calculate t    
    mu_q = mu_q.reshape((3,1))
    mu_p = mu_p.reshape((3,1))
    t = (mu_q-np.matmul(R,mu_p)).reshape((3))


    return m3d.Transform(R,t)



if __name__ == "__main__":
    T = findTransform(P,Q)
    # Uncomment to see if T actually performs the correct transform
    # Should print values extremely close to 0
    # print((T*P)-Q)
