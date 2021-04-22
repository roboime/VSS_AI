// dllmain.cpp : Defines the entry point for the DLL application.
//Using VS2019

#include "pch.h"

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

extern "C" __declspec(dllexport) int __cdecl SetEnv(bool dealloc, int a, int b, int* c);

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
__declspec(dllexport) int __cdecl SetEnv(bool dealloc, int a, int b, int* c) {

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
    if (dealloc) {
        FreeConsole();
        fclose(fDummy);
    }



    //Add code here
        cout << "111";
    *c = b - a;
    return b + a;

    //end
}
