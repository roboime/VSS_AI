#include "framework.h"
#include "pch.h"

using namespace std;
//Univector Field code test
//Developed by RoboIME Team 2021
//função gaussiana que será usada na construção do UVF pela composição do TUF e do AUF
float gauss(float r, float delta) {
    float g = exp(-pow(r, 2) / (2 * pow(delta, 2)));
    return g;
};

//Espiral hiperbólica (HS), onde tx e ty são parâmetros de ajuste, de é o raio mínimo e kr é a constante de suavização da curva 
float HS(node point, float tx, float ty, float de, float kr, float ccw) {
    float PhiH;
    int signal = 1;
    //ajusta o sinal dependendo da orientação desejada
    if (ccw == 1) {
        signal = -1;
    };

    //definição de variáveis auxiliares
    float theta = atan2(get<1>(point) - ty, get<0>(point) - tx);
    float ro = sqrt(pow(get<0>(point) - tx, 2) + pow(get<1>(point) - ty, 2));

    //casos dependendo do raio mínimo
    if (ro > de) {
        PhiH = theta + signal * (M_PI / 2) * (2 - (de + kr) / (ro + kr));
    };
    if (ro <= de) {
        PhiH = theta + signal * (M_PI / 2) * sqrt(ro / de);
    };
    //ajusta o ângulo para o intervalo [-pi, pi]
    return remainder(PhiH, 2 * M_PI);
};

//Campo que gera um caminho simples até o objetivo (TUF) na orientação desejada. Os parâmetros de e kr serão utilizados
//para gerar as HS. Essa função retorna um ângulo entre [-pi, pi]
float TUF(node point, node target, float de, float kr) {
    //obtém a orientação do target, que será utilizada para fazer a rotação do sistema coordenado
    float rot = -remainder(get<2>(target), 2 * M_PI);
    float tuf_angle;
    //cria um ponto auxiliar para realizar a rotação e translação
    node tmppoint;

    get<0>(tmppoint) = (get<0>(point) * cos(rot) - get<1>(point) * sin(rot)) - (get<0>(target) * cos(rot) - get<1>(target) * sin(rot));
    get<1>(tmppoint) = (get<0>(point) * sin(rot) + get<1>(point) * cos(rot)) - (get<0>(target) * sin(rot) + get<1>(target) * cos(rot));

    //cria os pontos auxiliares para a determinação das espirais hiperbólicas
    node p_l = make_tuple(get<0>(tmppoint), get<1>(tmppoint) + de, 0);
    node p_r = make_tuple(get<0>(tmppoint), get<1>(tmppoint) - de, 0);

    if (get<1>(tmppoint) <= -de) {
        tuf_angle = HS(p_l, 0, 0, de, kr, 1);
    }
    else if (get<1>(tmppoint) >= de) {
        tuf_angle = HS(p_r, 0, 0, de, kr, 0);
    }
    else {
        //definição das duas espirais hiperbólicas
        float phiCCW = HS(p_r, 0, 0, de, kr, 0);
        float phiCW = HS(p_l, 0, 0, de, kr, 1);

        tuple<float, float> NhCW = make_tuple(cos(phiCW), sin(phiCW));
        tuple<float, float> NhCCW = make_tuple(cos(phiCCW), sin(phiCCW));

        //definição das componentes do campo resultante
        float tuf_cos = (abs(get<1>(p_l)) * get<0>(NhCCW) + abs(get<1>(p_r)) * get<0>(NhCW)) / (2.0 * de);
        float tuf_sin = (abs(get<1>(p_l)) * get<1>(NhCCW) + abs(get<1>(p_r)) * get<1>(NhCW)) / (2.0 * de);

        tuf_angle = atan2(tuf_sin, tuf_cos);
    };
    //retorna a orientação do campo original
    return remainder(tuf_angle - rot, 2 * M_PI);
};

//Campo para desviar de obstáculos (AUF), onde obstacle vector são os obstáculos no campo passados na forma de tuplas <x, y>.
tuple<float, tuple<float, float>, float> AUF(node point, vector<tuple<float, float>> obstacle_vector) {
    //parâmetros para controlar qual o obstáculo mais próximo ao ponto
    float min_dist = 99999;
    tuple<float, float> min_obst;
    //vetor para inserir todos os cossenos, senos e distâncias calculadas
    vector<node> obstacle_p;
    int i;
    for (i = 0; i < obstacle_vector.size(); i++) {
        //determinação da distância para cada obstáculo no campo
        float dist = sqrt(pow((get<0>(point) - get<0>(obstacle_vector[i])), 2) + pow((get<1>(point) - get<1>(obstacle_vector[i])), 2));
        //guarda as menores distâncias e o obstáculo mais próximo
        if (dist <= min_dist) {
            min_dist = dist;
            min_obst = obstacle_vector[i];
        };
        if (dist == 0) {
            continue;
        }
        else {
            //cálculo do cosseno e seno relativo a cada obstáculo no campo
            float cos = (get<0>(point) - get<0>(obstacle_vector[i])) / dist;
            float sin = (get<1>(point) - get<1>(obstacle_vector[i])) / dist;
            node obs_p = make_tuple(cos, sin, dist);
            obstacle_p.push_back(obs_p);
        }
    };
    //cosseno e seno resultantes do AUF
    float cos_r = 0;
    float sin_r = 0;
    for (i = 0; i < obstacle_p.size(); i++) {
        cos_r += get<0>(obstacle_p[i]) / get<2>(obstacle_p[i]);
        sin_r += get<1>(obstacle_p[i]) / get<2>(obstacle_p[i]);
    };
    //normalização dos resultados 
    float rho = sqrt(pow(cos_r, 2) + pow(sin_r, 2));
    cos_r = cos_r / rho;
    sin_r = sin_r / rho;

    //retorna um ângulo no intervalo [-pi, pi], além das coordenadas do obstáculo mais próximo e a distância classificada como mínima
    return make_tuple(remainder(atan2(sin_r, cos_r), 2 * M_PI), min_obst, min_dist);
};

//Campo resultante. retorna o ângulo final para cálculo das velocidades
float UVF(node point, node target, vector<tuple<float, float>> obstacle_vector,
    float de, float kr, float d_min, float delta) {
    //vetor com as informações do AUF
    tuple<float, tuple<float, float>, float> auf_vector = AUF(point, obstacle_vector);
    float tuf_angle = TUF(point, target, 5, 5);
    float uvf_angle;
    //caso a distância seja menor que um mínimo, ele deverá considerar apenas a angulação do AUF. Caso contrário, deverá ser feita a composição
    if (get<2>(auf_vector) < d_min) {
        uvf_angle = get<0>(auf_vector);
        cout << "Case 0: " << tuf_angle << ' ' << get<0>(auf_vector) << ' ' << uvf_angle << endl;
    }
    else {
        if (abs(tuf_angle - get<0>(auf_vector)) >= (2 * M_PI - abs(tuf_angle - get<0>(auf_vector)))) {
            if (get<0>(auf_vector) < 0) {
                uvf_angle = remainder(((2 * M_PI + get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta))), 2 * M_PI);
                cout << "Case 1: " << tuf_angle << ' ' << get<0>(auf_vector) << ' ' << uvf_angle << endl;
            }
            else {
                if (tuf_angle < 0) {
                    uvf_angle = remainder(((get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (2 * M_PI + tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta))), 2 * M_PI);
                    cout << "Case 2: " << tuf_angle << ' ' << get<0>(auf_vector) << ' ' << uvf_angle << endl;
                }
                if (abs(tuf_angle - get<0>(auf_vector)) == (2 * M_PI - abs(tuf_angle - get<0>(auf_vector)))) {
                    uvf_angle = (tuf_angle);
                }
            }
        }
        else {
            uvf_angle = remainder((get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta)), 2 * M_PI);
            cout << "Case 3: " << tuf_angle << ' ' << get<0>(auf_vector) << ' ' << uvf_angle << endl;
        }

    }
    cout << "Resultado do UVF_Angle " << tuf_angle << ' ' << get<0>(auf_vector) << ' ' << uvf_angle << endl;
    return uvf_angle;
};