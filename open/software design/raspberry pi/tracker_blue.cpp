#include "common.h"

int main()
{
    show_debug_windows = true;
    startup();
    handleThreads(false, false, true);
    shutdown();
}
