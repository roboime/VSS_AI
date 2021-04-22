// dllmain.cpp : Defines the entry point for the DLL application.
//Using VS2019

#include "pch.h"
#include "Environment.h"
#include <chrono>
#include <ctime>

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

// remove stupid MSVC min/max macro definitions
#undef min
#undef max

#ifdef _DEBUG
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
#define DBG_NEW new
#endif


using namespace std;

extern "C" __declspec(dllexport) int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug);

BOOL APIENTRY DllMain(HMODULE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved
)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}
__declspec(dllexport) int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug) {

    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

    //INICIAR SEED
    srand(*seed);

    auto st = std::chrono::high_resolution_clock::now();

    //Setar variaveis
    //Criar Environment
    Environment env(field_length, field_width);
    //Start/end
    Node start(*(startAndEnd), *(startAndEnd + 1), nullptr);
    Node end(*(startAndEnd + 2), *(startAndEnd + 3), nullptr);
    //ObstacleList
    vector<pair<float, Node>> obsV;
    for (int i = 0; i < obsLen; i++) {
        obsV.push_back(make_pair(*(obsList + 3*i), Node(*(obsList + 3*i + 1), *(obsList +3*i +2), nullptr)));
    }
    ObstacleGrid obs(obsV);
    //lastTree
    vector<Node> lst {Node(0,0,nullptr)};
  
    for (int i = 0; i < lastTreeLen; i++) {
        lst.push_back(Node(*(lastTree + 2*i), *(lastTree + 2*i + 1), nullptr));
    }
    
    //Generate Tree
     Tree tr(start, end, probGoal, probWaypoint, env, obs, lst);
    //se nao for possivel calcular, usar SimplePath
      *useSimplePath = !tr.grow(step, threshold);
    if (*useSimplePath) {
        obsV.clear();
        lst.clear();
        *pathsize = 0;
        *debug = 0;
        return 0; 
    }
    else {
        
        vector<Node> path = tr.backtrack();
        int pathLen = path.size();

        for (int i = 0; i < pathLen; i++) {
           
            *(newTree + 2*i) = path[pathLen -1 - i]._x;
            *(newTree + 2*i + 1) = path[pathLen - 1 - i]._y;
        }
        obsV.clear();
        lst.clear();
        path.clear();
        *pathsize = pathLen;
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - st;
        double fps = 1 / elapsed.count();
        *debug = fps;
        return 0;
    }
}
