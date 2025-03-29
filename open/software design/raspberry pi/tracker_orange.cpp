#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();
    handleThreads(true, false, false);
    shutdown();
}
