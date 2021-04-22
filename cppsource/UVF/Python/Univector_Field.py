import numpy as np
import random
import copy
import pandas as pd
import matplotlib.pyplot as plt


def wraptopi(angle):
    mask = 0
    xwrap = np.remainder(angle, 2*np.pi)
    if np.abs(xwrap) > np.pi :
        if xwrap > 0:
            mask = 1
        elif xwrap < 0 :
            mask = -1
    xwrap -= 2*np.pi * mask
    return xwrap

def gauss(r, delta):
    g = np.exp(-np.power(r, 2) / (2 * np.power(delta, 2)))
    return g

#função que gera espirais (campo) -> angulo
def HS(point, tx, ty, de, kr, ccw):
    x = point[0]
    y = point[1]
    o = point[2]
    hs_point = [x, y, o]
    #ajusta o sinal dependendo da orientação desejada
    signal = 1
    if ccw == 1:
        signal = -1

    #tx e ty são parâmetros de ajuste
    theta = np.arctan2(hs_point[1] - ty, hs_point[0] - tx)
    ro = np.sqrt(np.power(hs_point[0] - tx, 2) + np.power(hs_point[1] - ty, 2))

    if ro > de :
        phiH = theta + signal * (np.pi/2) * (2 - ((de + kr)/(ro + kr)))
        hs_point[2] = phiH
    else :
        phiH = theta + signal * (np.pi/2) * np.sqrt(ro/de)
        hs_point[2] = phiH
    return hs_point[2]

#campo para desviar de obstáculos -> [cosseno, seno]
def AUF(point, obstacles_vector):
    obstacle_parameters = [] #lista com elementos da forma <vector<pair<<cos,sin>, distância>>>
    min_obstacle = None
    min_dist = 99999999
    for obs in obstacles_vector:
        dist = np.sqrt(np.power((point[0] - obs[0]), 2) + np.power((point[1] - obs[1]), 2))
        if dist <= min_dist :
            min_obstacle = obs
            min_dist = dist
        if dist == 0:
            pass
        else:
            cos = (point[0] - obs[0]) / dist
            sin = (point[1] - obs[1]) / dist
            obstacle_parameters.append([(cos,sin), dist])
    result_cos = 0
    result_sin = 0
    dist_prod = 1
    #realiza a multiplicação, atribuindo maior peso ao obstáculo mais próximo do nó
    for i in obstacle_parameters:
        factor = 1
        for j in obstacle_parameters:
            if i == j:
                pass
            else:
                factor = factor*j[1]
        result_cos = result_cos + i[0][0] * factor
        result_sin = result_sin + i[0][1] * factor
        dist_prod = dist_prod * i[1]
    result_cos = result_cos/dist_prod
    result_sin = result_sin/dist_prod
    k = np.sqrt(np.power(result_cos, 2) + np.power(result_sin, 2))
    if k == 0:
        k = 1
    return [result_cos / k,result_sin / k], min_obstacle, min_dist

#função que gera uma angulação, recebendo um ponto
def TUF(point,target, k_de, k_kr):
    # obtém a orientação do target e define uma nova orientação relativa a orientação do target
    target[2] = wraptopi(target[2])
    ot = target[2]
    rot = -ot

    # rotate UVF to target reference
    tmppoint = point.copy()
    apoint = point.copy()
    tmptarget = target.copy()
    atarget = target.copy()

    apoint[0] = tmppoint[0] * np.cos(rot) - tmppoint[1] * np.sin(rot)
    apoint[1] = tmppoint[0] * np.sin(rot) + tmppoint[1] * np.cos(rot)

    atarget[0] = tmptarget[0] * np.cos(rot) - tmptarget[1] * np.sin(rot)
    atarget[1] = tmptarget[0] * np.sin(rot) + tmptarget[1] * np.cos(rot)
    # traslate target as the new origin

    apoint[0] = apoint[0] - atarget[0]
    apoint[1] = apoint[1] - atarget[1]

    #agora todos os pontos estão transladados
    #criando as variáveis auxiliares
    yl = apoint[1] + k_de
    yr = apoint[1] - k_de
    pl = [apoint[0], yl, 0]
    pr = [apoint[0], yr, 0]
    if apoint[1] <= - k_de :
        TUF_angle = HS(pl, 0, 0, k_de, k_kr, 1)
    elif apoint[1] >= k_de :
        TUF_angle = HS(pr, 0, 0, k_de, k_kr, 0)
    else :
        phiHCCW = HS(pr, 0, 0, k_de, k_kr, 0)
        phiHCW  = HS(pl, 0, 0, k_de, k_kr, 1)

        NhCW  = [np.cos(phiHCW), np.sin(phiHCW)]
        NhCCW = [np.cos(phiHCCW), np.sin(phiHCCW)]

        phiP_cos = ((abs(yl) * NhCCW[0]) + (abs(yr) * NhCW[0])) / (2. * k_de)
        phiP_sin = ((abs(yl) * NhCCW[1]) + (abs(yr) * NhCW[1])) / (2. * k_de)
        TUF_angle = np.arctan2(phiP_sin, phiP_cos)

    return TUF_angle - rot

#função que junta os efeitos dos dois campos AUF e TUF
def UVF(point, target, obstacles_vector,k_de, k_kr, k_dmin, k_delta):
    auf_vector = AUF(point,obstacles_vector)                      #devolve um vetor [cos, sen], obstacle, distância para o ponto
    auf_dist   = auf_vector[2]
    auf_angle  = wraptopi(np.arctan2(auf_vector[0][1], auf_vector[0][0]))   #gera um ângulo entre pi e -pi
    tuf_angle   = wraptopi(TUF(point,target,k_de, k_kr))         #devolve um valor (ângulo)

    uvf_angle = 0

    if auf_dist < k_dmin:
        uvf_angle = auf_angle

    else :
        if abs(tuf_angle - auf_angle) >= (2*np.pi - abs(tuf_angle - auf_angle)) :
           if auf_angle <= 0:
                uvf_angle = wraptopi((2*np.pi + auf_angle) * gauss(auf_dist - k_dmin, k_delta) + (tuf_angle) * (1 - gauss(auf_dist - k_dmin, k_delta)))
           if tuf_angle <= 0:
                uvf_angle = wraptopi((auf_angle) * gauss(auf_dist - k_dmin, k_delta) + (2 * np.pi + tuf_angle) * (1 - gauss(auf_dist - k_dmin, k_delta)))
        else :
            uvf_angle = wraptopi((auf_angle) * gauss(auf_dist - k_dmin, k_delta) + (tuf_angle) * (1 - gauss(auf_dist - k_dmin, k_delta)))

    return uvf_angle

if __name__ == '__main__' :
    x_index = [i for i in np.arange(-29, 30, 2)]
    y_index = [i for i in np.arange(29, -30, -2)]
    Field = pd.DataFrame([], y_index, x_index)

    # create the base-matrix for UVF
    #for x in x_index:
    #    for y in y_index:
    #        Field[x][y] = [x, y, 0.]

    #fig1, [[ax, bx], [cx, dx]] = plt.subplots(2,2)
    fig1, bx = plt.subplots()
    x = [-30, -30, 29, 29, -30]
    y = [29, -30, -30, 29, 29]
    xx = [0, 0]
    yy = [29, -30]
    xxx = [-30, 29]
    yyy = [0, 0]
    #ax.plot(x, y, color='y', linestyle = ':')
    #ax.plot(xx, yy, color='y', linestyle = ':')
    #ax.plot(xxx, yyy, color='y', linestyle = ':')
    bx.plot(x, y, color='y', linestyle = ':')
    bx.plot(xx, yy, color='y', linestyle = ':')
    bx.plot(xxx, yyy, color='y', linestyle = ':')
    #cx.plot(x, y, color='y', linestyle=':')
    #cx.plot(xx, yy, color='y', linestyle=':')
    #cx.plot(xxx, yyy, color='y', linestyle=':')
    #dx.plot(x, y, color='y', linestyle=':')
    #dx.plot(xx, yy, color='y', linestyle=':')
    #dx.plot(xxx, yyy, color='y', linestyle=':')

    de = 3
    kr = 5
    delta = 3
    d_min = 5
    obstacle_vector = [[-7.5, 5], [7.5, -5]]
    target = [10, 10, -np.pi/2]


    for row in Field.index:
        for column in Field.columns:
            #bx.quiver(column, row, np.cos(HS([column, row, 0], 0, 0, de, kr, 1)), np.sin(HS([column, row, 0], 0, 0, de, kr, 1)), scale=50)
            #ax.quiver(column, row, np.cos(TUF([column, row, 0], [0 , 0, np.pi/4], 5, 5)), np.sin(TUF([column, row, 0], [0 , 0, np.pi/4], 5, 5)), scale = 75)
            #bx.quiver(column, row, np.cos(TUF([column, row, 0], target, 3, 5)),  np.sin(TUF([column, row, 0], target, 3, 5)), scale=50)
            #ax.quiver(column, row, np.cos(HS([column, row, 0], 0, 0, 5, 5, 1)), np.sin(HS([column, row, 0], 0, 0, 5, 5, 1)), scale=75)
            #bx.quiver(column, row, AUF([column, row, 0], obstacle_vector)[0][0], AUF([column, row, 0], obstacle_vector)[0][1], scale = 65)
            #ax.quiver(column, row, np.cos(UVF([column, row, 0],target, obstacle_vector, de, 1, d_min, delta)), np.sin(UVF([column, row, 0],target, obstacle_vector, de, 1, d_min, delta)), scale=50)
            bx.quiver(column, row, np.cos(UVF([column, row, 0],target, obstacle_vector, de, 3, d_min, delta)), np.sin(UVF([column, row, 0],target, obstacle_vector, de, 3, d_min, delta)), scale=50)
            #bx.quiver(column, row, np.cos(UVF([column, row, 0], target, obstacle_vector, de, 5, d_min, delta)), np.sin(UVF([column, row, 0], target, obstacle_vector, de, 5, d_min, delta)), scale=50)
            #dx.quiver(column, row, np.cos(UVF([column, row, 0], target, obstacle_vector, de, 7, d_min, delta)), np.sin(UVF([column, row, 0], target, obstacle_vector, de, 7, d_min, delta)), scale=50)

    #UVF([-5, -5, 0], [0, 0, np.pi / 4], obstacle_vector, 5, 5, 5, 2)
    #for obs in obstacle_vector:
    #    obs_circle = plt.Circle((obs[0], obs[1]), 5, color = 'r')
    #    ax.add_patch(obs_circle)
    for obs in obstacle_vector:
        obs_circle = plt.Circle((obs[0], obs[1]), 5, color = 'r')
        bx.add_patch(obs_circle)
   # target_circle = plt.Circle((target[0], target[1]), 1, color = 'g')
   # bx.add_patch(target_circle)
    #for obs in obstacle_vector:
    #for obs in obstacle_vector:
    #    obs_circle = plt.Circle((obs[0], obs[1]), 5, color='r')
    #    cx.add_patch(obs_circle)
    #for obs in obstacle_vector:
        #obs_circle = plt.Circle((obs[0], obs[1]), 5, color='r')
        #dx.add_patch(obs_circle)

    #ax.set_xlabel('kr = 1')
    bx.set_xlabel('TUF')
    #cx.set_xlabel('kr = 5')
    #dx.set_xlabel('kr = 7')
    plt.show()

    current = [1,1,1]
    print(HS(current, 0, 0, 5, 5, 0))