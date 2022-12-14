from numbers import Real
from re import T
import numpy as np
import math
import cmath
# bộ thông số hình học robot
a1 = 45.0237
a2 = -26.7644
b1 = -7.2984
c1 = 76.5092
c2 = 95.5003
c3 = 155.748
c4 = 22.6872


def ik(x, y, z):
    iP = np.array([x, y, z])  # input position
    DofO = np.array([0, 0, -1])  # directionOfOperation
    if DofO[0] == 0 and DofO[1] == 0 and DofO[2] == 0:
        print("Error: Direction of Operation is Zero")
        return
    elif DofO[0] == 0 and DofO[1] == 0:
        EE = np.array([[1, 0, 0, 0],
                      [iP[0], DofO[2], 0, 0],
                      [iP[1], 0, 1, 0],
                      [iP[2], 0, 0, DofO[2]]])
    else:
        DofO = DofO/np.linalg.norm(DofO)
        dn = math.sqrt(DofO[0]**2 + DofO[1]**2)
        EE = np.array([[1, 0, 0, 0, ],
                       [iP[0], (DofO[0]*DofO[2])/dn, (-DofO[1])/dn, DofO[0]],
                       [iP[1], (DofO[1]*DofO[2])/dn, (DofO[0])/dn, DofO[1]],
                       [iP[2], (-dn), 0, DofO[2]]])

    # tọa độ định hướng điểm C
    x = (EE[1:4, 1:4])
    y = (EE[1:4, 0])
    C = np.array([[1, 0, 0, 0],
                  [(EE[1, 0]-EE[1, 3]*c4), EE[1, 1], EE[1, 2], EE[1, 3]],
                  [(EE[2, 0]-EE[2, 3]*c4), EE[2, 1], EE[2, 2], EE[2, 3]],
                  [(EE[3, 0]-EE[3, 3]*c4), EE[3, 1], EE[3, 2], EE[3, 3]]])
    Cx0 = C[1, 0]
    Cy0 = C[2, 0]
    Cz0 = C[3, 0]
    nx1 = math.sqrt(Cx0**2 + Cy0**2-b1**2)-a1
    s12 = nx1**2 + (Cz0-c1)**2  # s12 = s1**2
    s22 = (nx1+2*a1)**2 + (Cz0-c1)**2  # s22 = s2**2
    a = math.sqrt(a2**2 + c3**2)

    theta = np.ones([3, 1])

    if np.isreal(nx1):
        theta[1, 0] = -math.acos((s12-a**2+c2**2) /
                                 (2*s12**0.5*c2))+math.atan2(nx1, Cz0-c1)
        # theta[1, 1] = cmath.acos(
        #     (s12-a**2+c2**2)/(2*s12**0.5*c2))+math.atan2(nx1, Cz0-c1)

        theta[2, 0] = math.acos(
            (s12-a**2-c2**2)/(2*c2*a)) - math.atan2(a2, c3)
        # theta[2, 1] = -cmath.acos((s12-a**2-c2**2) /
        #                           (2*c2*a)) - math.atan2(a2, c3)
        # theta[2, 2] = cmath.acos(
        #     (s22-a**2-c2**2)/(2*c2*a)) - math.atan2(a2, c3)
        # theta[2, 3] = -cmath.acos((s22-a**2-c2**2) /
        #                           (2*c2*a)) - math.atan2(a2, c3)
        if np.isreal(nx1+a1):
            theta[0, 0] = math.atan2(
                Cy0, Cx0)-math.atan2(b1, nx1+a1)
            # theta[0, 2] = theta[0, 3] = math.atan2(
            #     Cy0, Cx0)+math.atan2(b1, nx1+a1)-math.pi

        # if np.isreal(nx1+2*a1):
        #     theta[1, 2] = -cmath.acos((s22+c2**2-a**2) /
        #                               (2*(s22**0.5)*c2))-math.atan2(nx1+2*a1, Cz0-c1)
        #     theta[1, 3] = cmath.acos(
        #         (s22-a**2+c2**2)/(2*(s22**0.5)*c2))-math.atan2(nx1+2*a1, Cz0-c1)
    # FOR Multiple solutions use:
    # ind = []
    # c23 = []
    # s23 = []
    # temp = []
    # for i in range(4):
    #     if np.isreal(theta[:, i]).all() == True:
    #         ind.append(i)
    # for i in theta[1, ind] + theta[2, ind]:
    #     c23.append(cmath.cos(i))
    #     s23.append(cmath.sin(i))
    c23 = math.cos(theta[1, 0]+theta[2, 0])
    s23 = math.sin(theta[1, 0]+theta[2, 0])
    r11 = EE[1, 1]
    r12 = EE[1, 2]
    r13 = EE[1, 3]

    r21 = EE[2, 1]
    r22 = EE[2, 2]
    r23 = EE[2, 3]

    r31 = EE[3, 1]
    r32 = EE[3, 2]
    r33 = EE[3, 3]

    B1 = np.ones([3, 1])
    M2 = np.ones([3, 4])
    # theta123 = np.zeros([6, 1])
    B1[0, 0] = math.atan2(-r13*math.sin(theta[0, 0]) + r23*math.cos(theta[0, 0]), np.multiply(
        r13*math.cos(theta[0, 0]), c23) + np.multiply(r23*math.sin(theta[0, 0]), c23) - r33*s23)
    B1[1, 0] = math.atan2(math.sqrt(1 - np.power((np.multiply((r13*math.cos(theta[0, 0])), s23) + np.multiply(r23*math.sin(theta[0, 0]),
                          s23) + r33*c23), 2)), np.multiply(r13*math.cos(theta[0, 0]), s23) + np.multiply(r23*math.sin(theta[0, 0]), s23) + r33*c23)
    B1[2, 0] = math.atan2(np.multiply(r12*math.cos(theta[0, 0]), s23) + np.multiply(r22*math.sin(theta[0, 0]), s23) +
                          r32*c23, - np.multiply(r11*math.cos(theta[0, 0]), s23) - np.multiply(r21*math.sin(theta[0, 0]), s23) - r31*c23)

    theta123 = np.array(
        [[theta[0, 0], theta[1, 0], theta[2, 0], B1[0, 0], B1[1, 0], B1[2, 0]]])

    Q1 = theta123[0, 0]
    Q2 = theta123[0, 1]
    Q3 = theta123[0, 2] - math.pi/2
    Q4 = theta123[0, 3]
    Q5 = theta123[0, 4]
    Q6 = theta123[0, 5]
    print(Q1, Q2, Q3, Q4, Q5, Q6)


    # for testing
ik(200, 0, 0)
