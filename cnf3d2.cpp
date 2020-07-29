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

using namespace std;

int main(int argc, char *argv[]) {

    std::thread t1(Display::init, argv[1]);

    auto api = new API();
    std::thread t2(&API::run_add_clause_socket, api);
    std::thread t3(&API::run_remove_clause_socket, api);
    std::thread t4(&API::run_increase_variable_activity_socket, api);
    std::thread t5(&API::run_assign_variable_truth_value, api);

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();

    return EXIT_SUCCESS;
}
