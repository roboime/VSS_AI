#include "pch.h"
#include "framework.h"


using namespace std;
//definindo a fun��o sign, que retorna +1 se o sinal for positivo, -1 caso negativo e 0 caso 0
int sign(float x) {
    return (x > 0) - (x < 0);
}
//controle do ve�culo dado sua orienta��o e posi��o no campo. As constantes de,kr,d_min e delta devem ser ajustadas emp�ricamente e depois fixadas no c�digo
tuple<float, float> control(node point, node target, vector<tuple<float, float>> obstacle_vector,
    float de, float kr, float d_min, float delta, float rho_min, float  v_max, float k_error) {
    //velocidades
    float  v;
    float  w;
    //calculo da dist�ncia at� o objetivo
    float rho = sqrt(pow((get<0>(point) - get<0>(target)), 2) + pow(get<1>(point) - get<1>(target), 2));
    //defini��o de pontos auxiliares para c�lculo do gradiente linearizado em torno do ponto
    node p_lower_x = make_tuple(get<0>(point) - 0.5, get<1>(point), get<2>(point));
    node p_upper_x = make_tuple(get<0>(point) + 0.5, get<1>(point), get<2>(point));
    node p_lower_y = make_tuple(get<0>(point), get<1>(point) - 0.5, get<2>(point));
    node p_upper_y = make_tuple(get<0>(point), get<1>(point) + 0.5, get<2>(point));

    //c�lculo dos 4 gradientes
    float grad_x;
    float grad_y;
    float grad_x_minus = ((UVF(p_upper_x, target, obstacle_vector, de, kr, d_min, delta)) - (UVF(p_lower_x, target, obstacle_vector, de, kr, d_min, delta)));
    float grad_x_plus = ((UVF(p_upper_x, target, obstacle_vector, de, kr, d_min, delta)) + (UVF(p_lower_x, target, obstacle_vector, de, kr, d_min, delta)));
    float grad_y_minus = ((UVF(p_upper_y, target, obstacle_vector, de, kr, d_min, delta)) - (UVF(p_lower_y, target, obstacle_vector, de, kr, d_min, delta)));
    float grad_y_plus = ((UVF(p_upper_y, target, obstacle_vector, de, kr, d_min, delta)) + (UVF(p_lower_y, target, obstacle_vector, de, kr, d_min, delta)));

    //testagem de qual gradiente n�o apresenta descontinuidades
    if (abs(grad_x_minus) >= abs(grad_x_plus)) {
        grad_x = grad_x_plus;
    }
    else {
        grad_x = grad_x_minus;
    }
    if (abs(grad_y_minus) >= abs(grad_y_plus)) {
        grad_y = grad_y_plus;
    }
    else {
        grad_y = grad_y_minus;
    }

    //determina��o da velocidade radial e angular
    if (rho >= rho_min) {
        v = v_max;
        float uvf_angle = UVF(point, target, obstacle_vector, de, kr, d_min, delta);
        float error_angle = remainder(get<2>(point) - uvf_angle, 2 * M_PI);

        w = (grad_x * cos(get<2>(point)) + grad_y * sin(get<2>(point))) * v - k_error * sign(error_angle) * sqrt(abs(error_angle));

    }
    else {
        v = v_max * (rho / rho_min);
        float uvf_angle = UVF(point, target, obstacle_vector, de, kr, d_min, delta);
        float error_angle = remainder(get<2>(point) - uvf_angle, 2 * M_PI);

        w = (grad_x * cos(get<2>(point)) + grad_y * sin(get<2>(point))) * v - k_error * sign(error_angle) * sqrt(abs(error_angle));
    }
    return make_tuple(v, w);
};
