#include "Display.h"
#include <zmq.hpp>
#include "msgpack.hpp"
#include <string>
#include <iostream>
#include "thread"
#include "API.h"

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

using namespace std;

int main(int argc, char *argv[]) {

    std::thread t1(Display::init, argv[1]);

//    using namespace std::chrono_literals;

    auto api = new API();
    std::thread t2(&API::run_add_socket, api);
    std::thread t3(&API::run_remove_socket, api);

    t1.join();
    t2.join();
    t3.join();

    return EXIT_SUCCESS;
}
