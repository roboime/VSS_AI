from Univector_Field import UVF, wraptopi, TUF, AUF
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def run(initial, target, obstacle_vector, MAX_IT):
    path = [initial]
    x = initial[0]
    y = initial[1]
    o = initial[2]
    v = 0
    w = 0
    t = 0.2
    #distância para iniciar a redução da velocidade, velocidade máxima do carro e constante de ajuste
    d_min = 5
    v_max = 2
    K_w = 0.25
    for i in range(MAX_IT):
        #calculating the distance between point and target
        rho = np.sqrt(np.power(target[0] - x, 2) + np.power(target[1] - y, 2))
        if rho <= 0.5:
            break
        # defining the velocities
        x_l = np.floor(x)
        y_l = np.floor(y)
        x_u = x_l + 1
        y_u = y_l + 1



        grad_x_minus = (UVF([x_u, y, o], target, obstacle_vector, 3, 5 , 5, 4)) - (UVF([x_l, y, o], target, obstacle_vector, 3, 5 , 5, 4))
        grad_x_plus  = (UVF([x_u, y, o], target, obstacle_vector, 3, 5 , 5, 4)) + (UVF([x_l, y, o], target, obstacle_vector, 3, 5 , 5, 4))
        grad_y_minus = (UVF([x, y_u, o], target, obstacle_vector, 3, 5 , 5, 4)) - (UVF([x, y_l, o], target, obstacle_vector, 3, 5 , 5, 4))
        grad_y_plus = (UVF([x, y_u, o], target, obstacle_vector, 3, 5 , 5, 4)) + (UVF([x, y_l, o], target, obstacle_vector, 3, 5 , 5, 4))

        if abs(grad_x_minus) >= abs(grad_x_plus) :
            grad_x  = grad_x_plus
        else :
            grad_x  = grad_x_minus

        if abs(grad_y_minus) >= abs(grad_y_plus) :
            grad_y  = grad_y_plus
        else :
            grad_y  = grad_y_minus

        if rho > d_min:
            v = v_max

            UVF_angle = UVF([x, y, o], target, obstacle_vector, 3, 5 , 5, 4)
            error_angle = wraptopi(o - UVF_angle)
            w = (grad_x * np.cos(o)  + grad_y * np.sin(o))* v - K_w * np.sign(error_angle) * np.sqrt(abs(error_angle))
            x = x + v * np.cos(o) * t
            y = y + v * np.sin(o) * t
            o = o + w * t


        if rho <= d_min:
            v = v_max * (rho / d_min)

            UVF_angle = UVF([x, y, o], target, obstacle_vector,  3, 5 , 5, 4)
            error_angle = wraptopi(o - UVF_angle)
            w = (grad_x * np.cos(o)  + grad_y * np.sin(o))* v - K_w * np.sign(error_angle) * np.sqrt(abs(error_angle))

            x = x + v * np.cos(o) * t
            y = y + v * np.sin(o) * t
            o = o + w * t

        path.append([x, y, o])
    return path



if __name__ == '__main__' :
    obstacle_vector = [[0,0]]
    initial = [-20, -10, 0]
    target = [15, 0, 1.71]
    x_index = [i for i in np.arange(-29,30 , 2)]
    y_index = [i for i in np.arange(29, -30, -2)]
    Field = pd.DataFrame([], y_index, x_index)
    # create the base-matrix for UVF
    for x in x_index:
        for y in y_index:
            Field[x][y] = [x, y, 0.]

    fig1, bx = plt.subplots()
    x = [-30, -30, 29, 29, -30]
    y = [29, -30, -30, 29, 29]
    xx = [0, 0]
    yy = [29, -30]
    xxx = [-30, 29]
    yyy = [0, 0]

    bx.plot(x, y, color='y', linestyle=':')
    bx.plot(xx, yy, color='y', linestyle=':')
    bx.plot(xxx, yyy, color='y', linestyle=':')
    bx.quiver(initial[0], initial[1], np.cos(initial[2]), np.sin(initial[2]), color='b', scale=20)
    bx.quiver(target[0], target[1], np.cos(target[2]), np.sin(target[2]), color='g', scale=20)
    #path = run(initial, target, obstacle_vector, 1000)
    #for node in path:
    #    bx.quiver(node[0], node[1], np.cos(node[2]), np.sin(node[2]),color = 'y' ,scale=40)



    for obs in obstacle_vector:
        obs_circle = plt.Circle((obs[0], obs[1]), 5, color = 'r')
        bx.add_patch(obs_circle)

    for row in Field.index:
        for column in Field.columns:
            # ax.quiver(column, row, np.cos(TUF([column, row, 0], [0 , 0, np.pi/4], 5, 5)), np.sin(TUF([column, row, 0], [0 , 0, np.pi/4], 5, 5)), scale = 75)
            #bx.quiver(column, row, np.cos(TUF([column, row, 0],target, 5, 5)),  np.sin(TUF([column, row, 0],target, 5, 5)), scale=75)
            # ax.quiver(column, row, np.cos(HS([column, row, 0], 0, 0, 5, 5, 1)), np.sin(HS([column, row, 0], 0, 0, 5, 5, 1)), scale=75)
            #bx.quiver(column, row, AUF([column, row, 0], obstacle_vector)[0][0], AUF([column, row, 0], obstacle_vector)[0][1], scale = 75)
            # ax.quiver(column, row, np.cos(UVF([column, row, 0],target, obstacle_vector, de, 1, d_min, delta)), np.sin(UVF([column, row, 0],target, obstacle_vector, de, 1, d_min, delta)), scale=50)
            # bx.quiver(column, row, np.cos(UVF([column, row, 0],target, obstacle_vector, de, 3, d_min, delta)), np.sin(UVF([column, row, 0],target, obstacle_vector, de, 3, d_min, delta)), scale=50)
            bx.quiver(column, row, np.cos(UVF([column, row, 0], target, obstacle_vector, 5, 5 , 5, 4)), np.sin(UVF([column, row, 0], target, obstacle_vector, 5, 5 , 5, 4)), scale=50)

    bx.set_xlabel('Control Test')
    #print(run(initial, target, obstacle_vector, 1))
    plt.show()

