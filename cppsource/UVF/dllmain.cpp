// dllmain.cpp : Defines the entry point for the DLL application.
//Using VS2019

#include "pch.h"
#include <chrono>


#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>
#include <iostream>

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

extern "C" __declspec(dllexport) int __cdecl SetEnv(float* p, float* t, float* obsList, int obsLen, float*v, float*w, float de,
    float kr, float d_min, float delta, float v_max, float rho_min,
    float k_error, float* debug);

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
__declspec(dllexport) int __cdecl SetEnv(float* p, float* t, float* obsList, int obsLen, float* v, float* w, float de,
    float kr, float d_min, float delta, float v_max, float rho_min,
    float k_error, float* debug) {

    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

#define ENABLE_CONSOLE_PRINT
    //Habilitar funções cout, cin e cerr. Não usar na versão final
#ifdef ENABLE_CONSOLE_PRINT
    AllocConsole();
    FILE* fDummy;
    freopen_s(&fDummy, "CONIN$", "r", stdin);
    freopen_s(&fDummy, "CONOUT$", "w", stderr);
    freopen_s(&fDummy, "CONOUT$", "w", stdout);
#endif // CONSOLE_PRINT
    //Desabilitar Console
#ifdef DISABLE_CONSOLE_PRINT
    FreeConsole();
#endif // DISABLE_CONSOLE_PRINT

    auto st = std::chrono::high_resolution_clock::now();

    //construção dos operadores
    node point  = make_tuple(*(p), *(p + 1), *(p + 2));
    node target = make_tuple(*(t), *(t + 1), *(t + 2));
    vector<tuple<float, float>> obstacle_vector;
    for (int i = 0; i < obsLen; i++) {
        obstacle_vector.push_back(make_tuple(*(obsList + 2 * i), *(obsList + 2 * i + 1)));
    };

    tuple<float, float> vel = control(point, target, obstacle_vector, de, kr, d_min, delta, rho_min, v_max, k_error);
    *v = get<0>(vel);
    *w = get<1>(vel);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = finish - st;
    float fps = 1 / elapsed.count();
    *debug = fps;
    return 0;
    //end
}
