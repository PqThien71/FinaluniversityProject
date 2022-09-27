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

    theta = np.ones([3, 4])*1j

    if np.isreal(nx1):
        theta[1, 0] = -cmath.acos((s12-a**2+c2**2) /
                                  (2*s12**0.5*c2))+math.atan2(nx1, Cz0-c1)
        theta[1, 1] = cmath.acos(
            (s12-a**2+c2**2)/(2*s12**0.5*c2))+math.atan2(nx1, Cz0-c1)

        theta[2, 0] = cmath.acos(
            (s12-a**2-c2**2)/(2*c2*a)) - math.atan2(a2, c3)
        theta[2, 1] = -cmath.acos((s12-a**2-c2**2) /
                                  (2*c2*a)) - math.atan2(a2, c3)
        theta[2, 2] = cmath.acos(
            (s22-a**2-c2**2)/(2*c2*a)) - math.atan2(a2, c3)
        theta[2, 3] = -cmath.acos((s22-a**2-c2**2) /
                                  (2*c2*a)) - math.atan2(a2, c3)
        if np.isreal(nx1+a1):
            theta[0, 0] = theta[0, 1] = math.atan2(
                Cy0, Cx0)-math.atan2(b1, nx1+a1)
            theta[0, 2] = theta[0, 3] = math.atan2(
                Cy0, Cx0)+math.atan2(b1, nx1+a1)-math.pi

        if np.isreal(nx1+2*a1):
            theta[1, 2] = -cmath.acos((s22+c2**2-a**2) /
                                      (2*(s22**0.5)*c2))-math.atan2(nx1+2*a1, Cz0-c1)
            theta[1, 3] = cmath.acos(
                (s22-a**2+c2**2)/(2*(s22**0.5)*c2))-math.atan2(nx1+2*a1, Cz0-c1)

    ind = []
    c23 = []
    s23 = []
    temp = []
    for i in range(4):
        if np.isreal(theta[:, i]).all() == True:
            ind.append(i)
    for i in theta[1, ind] + theta[2, ind]:
        c23.append(cmath.cos(i))
        s23.append(cmath.sin(i))

    r11 = EE[1, 1]
    r12 = EE[1, 2]
    r13 = EE[1, 3]

    r21 = EE[2, 1]
    r22 = EE[2, 2]
    r23 = EE[2, 3]

    r31 = EE[3, 1]
    r32 = EE[3, 2]
    r33 = EE[3, 3]

    B1 = np.ones([3, 4])*1j
    M2 = np.ones([3, 4])*1j
    if len(ind) != 0:
        B1_1_temp1 = []
        B1_1_temp2 = []
        B1_1_temp3 = []
        B1_2_temp1 = []
        B1_2_temp2 = []
        B1_2_temp3, B1_3_temp5 = [], []
        B1_3_temp1, B1_3_temp2, B1_3_temp3, B1_3_temp4, B1_3_temp6, B1_3_temp7 = [], [], [], [], [], []
        Y1 = []
        for i in range(len(ind)):
            Y1.append(-r13*math.sin(theta[0, ind[i]]
                                    ) + r23*math.cos(theta[0, ind[i]]) + 0)
            B1_1_temp1.append(r13*cmath.cos(theta[0, ind[i]]))
            B1_1_temp2.append(r23*cmath.sin(theta[0, ind[i]]))
            B1_1_temp3.append(r33*s23[i])
            B1_2_temp1.append(r13*cmath.cos(theta[0, ind[i]]))
            B1_2_temp2.append(r23*cmath.sin(theta[0, ind[i]]))
            B1_2_temp3.append(r33*c23[i])
            B1_3_temp5.append(r33*c23[i])
            B1_3_temp1.append(r12*cmath.cos(theta[0, ind[i]]))
            B1_3_temp2.append(r22*cmath.sin(theta[0, ind[i]]))
            B1_3_temp3.append(r32*c23[i])
            B1_3_temp4.append(r11*cmath.cos(theta[0, ind[i]]))
            B1_3_temp6.append(r21*cmath.sin(theta[0, ind[i]]))
            B1_3_temp7.append(r31*c23[i])

        X1 = np.multiply(B1_1_temp1, c23) + \
            np.multiply(B1_1_temp2, c23)-B1_1_temp3+0
        Y2 = np.multiply(B1_2_temp1, s23) + \
            np.multiply(B1_2_temp2, s23)+B1_2_temp3+0
        Y2 = np.power(Y2, 2)+0
        X2 = - np.multiply(B1_1_temp1, s23) - r21 + \
            np.multiply(B1_1_temp2, s23) + B1_3_temp5+0

        Y3 = np.multiply(B1_3_temp1, s23) + \
            np.multiply(B1_3_temp2, s23) + B1_3_temp3+0
        X3 = - np.multiply(B1_3_temp4, s23) - \
            np.multiply(B1_3_temp6, s23) - B1_3_temp7+0
        for i in range(len(ind)):
            B1[0, ind[i]] = math.atan2(Y1[i], X1[i])
            B1[1, ind[i]] = math.atan2(math.sqrt(1 - Y2[i]), X2[i])
            B1[2, ind[i]] = math.atan2(Y3[i], X3[i])

    for i in range(np.shape(B1)[1]):
        M2[0, i] = B1[0, i]+math.pi
        M2[1, i] = - B1[1, i]
        M2[2, i] = B1[2, i]-math.pi

    theta123456 = np.empty([6, 8])
    theta123456_real = np.empty([6, 8])
    theta123456_imag = np.empty([6, 8])
    theta123456 = [[theta[0, 0], theta[0, 0], theta[0, 1], theta[0, 1], theta[0, 2], theta[0, 2], theta[0, 3], theta[0, 3]],
                   [theta[1, 0], theta[1, 0], theta[1, 1], theta[1, 1],
                    theta[1, 2], theta[1, 2], theta[1, 3], theta[1, 3]],
                   [theta[2, 0], theta[2, 0], theta[2, 1], theta[2, 1],
                    theta[2, 2], theta[2, 2], theta[2, 3], theta[2, 3]],
                   [B1[0, 0], M2[0, 0], B1[0, 1], M2[0, 1],
                       B1[0, 2], M2[0, 2], B1[0, 3], M2[0, 3]],
                   [B1[1, 0], M2[1, 0], B1[1, 1], M2[1, 1],
                       B1[1, 2], M2[1, 2], B1[1, 3], M2[1, 3]],
                   [B1[2, 0], M2[2, 0], B1[2, 1], M2[2, 1], B1[2, 2], M2[2, 2], B1[2, 3], M2[2, 3]]]

    for i in range(6):   # 6 rows
        for j in range(8):  # 8 columns
            theta123456[i][j] = theta123456[i][j]+(math.pi+0j)
            theta123456_real[i][j] = theta123456[i][j].real
            theta123456_imag[i][j] = theta123456[i][j].imag
            theta123456[i][j] = theta123456_real[i][j] % (
                2*math.pi) + 1j * theta123456_imag[i][j] - (math.pi+0j)
            theta123456[i][j] = theta123456[i][j] * 180 / math.pi
    Q1 = (theta123456[0][0], theta123456[1][0], theta123456[2][0],
          theta123456[3][0], theta123456[4][0], theta123456[5][0])
    Q2 = (theta123456[0][1], theta123456[1][1], theta123456[2][1],
          theta123456[3][1], theta123456[4][1], theta123456[5][1])
    Q3 = (theta123456[0][2], theta123456[1][2], theta123456[2][2],
          theta123456[3][2], theta123456[4][2], theta123456[5][2])
    Q4 = (theta123456[0][3], theta123456[1][3], theta123456[2][3],
          theta123456[3][3], theta123456[4][3], theta123456[5][3])
    Q5 = (theta123456[0][4], theta123456[1][4], theta123456[2][4],
          theta123456[3][4], theta123456[4][4], theta123456[5][4])
    Q6 = (theta123456[0][5], theta123456[1][5], theta123456[2][5],
          theta123456[3][5], theta123456[4][5], theta123456[5][5])
    Q7 = (theta123456[0][6], theta123456[1][6], theta123456[2][6],
          theta123456[3][6], theta123456[4][6], theta123456[5][6])
    Q8 = (theta123456[0][7], theta123456[1][7], theta123456[2][7],
          theta123456[3][7], theta123456[4][7], theta123456[5][7])
    return Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8

    # for testing
ik(200, 100, 50)
