#include "Display.h"
#include <string>
#include "thread"
#include "API.h"

#ifndef _WIN32

#include <unistd.h>

#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

int main(int argc, char *argv[]) {
    auto display = std::make_unique<Display>(argv[1]);

    std::thread t1(&Display::init, display.get());

    auto api = std::make_unique<API>(display.get());

    std::thread t2(&API::run, api.get());

//    sleep(5);

//    Display::walksat(display.get(), api.get());
//    display->solve();

    t1.join();
    t2.join();
//    t3.join();
//    t4.join();
//    t5.join();

    return EXIT_SUCCESS;
}