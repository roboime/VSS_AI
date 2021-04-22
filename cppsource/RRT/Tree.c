#include <stdio.h>
#include<ctime>
#include <math.h>
#include <float.h>

#include "pch.h"
#include "Tree.h"
#include "Environment.h"

Node::Node(float x = 0, float y = 0, Node* parent = nullptr) : _x(x), _y(y), _parent(parent) {
    _vec.push_back(_x);
    _vec.push_back(_y);
}

Node Node::operator + (const Node& obj) {
    Node operation;
    operation._x = this->_x + obj._x;
    operation._y = this->_y + obj._y;
    return Node(operation._x, operation._y, nullptr);
}

Node Node::operator - (const Node& obj) {
    Node operation;
    operation._x = this->_x - obj._x;
    operation._y = this->_y - obj._y;
    return Node(operation._x, operation._y, nullptr);
}

float Node::operator * (const Node& obj) {
    return ((this->_x * obj._x) + (this->_y * obj._y));
}

bool Node::operator == (const Node& obj) {
    if (this->_x == obj._x && this->_y == obj._y && this->_parent == obj._parent) return true;
    else return false;
}

Node Node::multiplyByConstant(float c) const {
    Node operation;
    operation._x = c * this->_x;
    operation._y = c * this->_y;
    return Node(operation._x, operation._y, nullptr);
}

float Node::modulus() const {
    return(sqrt(pow(this->_x, 2) + pow(this->_y, 2)));
}

Node Node::makeUnitary() {
    if (this->modulus() == 0) return Node(0, 0, this->_parent);
    else return Node(_x / this->modulus(), _y / this->modulus(), this->_parent);
}

Node::~Node() {
    _vec.clear();
}


Tree::Tree(Node& root, Node& goal, float probGoal, float probWaypoint, Environment& env, ObstacleGrid& obs, vector<Node>& waypointCache) :
    _root(root), _goal(goal), _env(env), _obs(obs), _waypointCache(waypointCache), _kdtree(flann::KDTreeSingleIndexParams()) {
    //iniciar kdtree com o primeiro Node
    _kdtree.buildIndex(flann::Matrix<float>(root._vec.data(), 1, 2));
    //adicionar primeiro node na fila
    _nodemap[root._vec] = root;
    //verificar se probGoal é válido
    if (probGoal >= 0 && probGoal <= 1) _probGoal = probGoal;
    else _probGoal = 1;
    //verificar se probWaypoint é válido
    if (probWaypoint >= 0 && probWaypoint <= 1) _probWaypoint = probWaypoint;
    else _probWaypoint = 0;
}

Node Tree::chooseTarget() {
    //gera um valor aleatório para comparar com _probGoal
    float p = (float(rand() % 100) / (100));
    int i = rand() % ((_waypointCache).size());
    //target sendo o objetivo
    if (p >= 0 && p < _probGoal) return _goal;
    //target sendo waypoint da iteração passada
    else if (p >= _probGoal && p < _probGoal + _probWaypoint) return (_waypointCache)[i];
    //target aleatório
    else return _env.randomState();
}

Node Tree::calcNN(Node& target) {
    //querry da pesquisa
    flann::Matrix<float> query(target._vec.data(), 1, 2);
    //indices em ordem de menor distância
    vector<int> i(query.rows);
    flann::Matrix<int> indices(i.data(), query.rows, 2);
    //distâncias(não usado ainda)
    vector<float> d(query.rows);
    flann::Matrix<float> dists(d.data(), query.rows, 2);
    //fazer pesquisa
    _kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());
    //retornar Node com coordenadas correspondentes
    float* point = _kdtree.getPoint(indices[0][0]);
    vector<float> temp(point, point + 2);
    return _nodemap[temp];
}

bool Tree::extend(float step, Node* last) {
    Node target = chooseTarget();
    Node nearest = calcNN(target);
    //calcular proximo node
    Node temp = (nearest + ((target - nearest).makeUnitary()).multiplyByConstant(step));
    //verificar colisão com parede ou obstáculo
    if (_env.checkWallColision(temp) || _obs.checkObstacleColision(nearest, temp)) return false;
    else {
        //verificar se ponto n existe no nodemap
        if (_nodemap.find(temp._vec) != _nodemap.end()) return true;
        //setar o parent do ponto
        temp._parent = &_nodemap[nearest._vec];
        addPoints(temp);
        *last = temp;
        return true;
    }
}

bool Tree::grow(float step, float threshold) {
    Node* last = new Node(_root._x, _root._y, nullptr);
    bool valid = false;
    //quando o contador ultrapassar 1000 itr, interromper e passar simple path
    int cont = 0;
    //setar threshold para parar
    int newThreshold = 0;
    if (threshold - _env.distance(_root, _goal) < 20) newThreshold = threshold;
    else newThreshold = 5;
    do {

        if (extend(step, last)) valid = _env.distance(*last, _goal) > newThreshold;
        else valid = true;
        //retorna false se não foi possivel calcular em menos de 1000 itr
        if (cont > 1000) {
            _goal._parent = &_nodemap[last->_vec];
            delete last;
            return false;
        }

        cont++;
    } while (valid);
    //adicionar goal no _nodemap com parent setado
    _goal._parent = &_nodemap[last->_vec];
    addPoints(_goal);
    //setar _parent do _root como nullptr(muito importante)
    _nodemap[_root._vec]._parent = nullptr;
    //retorna true se foi calculado em menos de 1000 itr
    delete last;
    return true;
}

void Tree::addPoints(Node& current) {
    //Adicionar no nodemap
    _nodemap[current._vec] = current;
    //Adicionar na kdtree
    _kdtree.addPoints(flann::Matrix<float>(_nodemap[current._vec]._vec.data(), 1, 2));
}

vector<Node> Tree::backtrack() {
    Node& temp = _goal;
    vector<Node> path = { _goal };
    //repetir até encontrar a origem
    while (!(temp == _root)) {
        //encontrar parent do vetor
        temp = _nodemap[(temp._parent)->_vec];
        //adicionar no vetor
        path.push_back(Node(temp._x, temp._y, nullptr));
    }
    return path;
}

Tree::~Tree() {
    _waypointCache.clear();
    _nodemap.clear();
}