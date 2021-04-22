// Template.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "Template.h"


// This is an example of an exported variable
TEMPLATE_API int nTemplate=0;

// This is an example of an exported function.
TEMPLATE_API int fnTemplate(void)
{
    return 0;
}

// This is the constructor of a class that has been exported.
CTemplate::CTemplate()
{
    return;
}
