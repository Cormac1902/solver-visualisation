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
    auto display = std::make_unique<Display>();

    std::thread t1(&Display::init, display.get(), argv[1]);

    auto api = std::make_unique<API>(display.get());

//    display->getInteraction().setAPI(api.get());
//    api->run();
    std::thread t2(&API::run, api.get());

//    sleep(5);

//    std::thread walksatThread(Display::walksat, display.get(), api.get());

//    api->send_start_interactor();

//    display->startDisplayInteractor();

//    std::thread t3(&Display::displayInteractor, display.get(), interaction.get());
//    std::thread t3(&API::run_remove_clause_socket, api.get());
//    std::thread t4(&API::run_increase_variable_activity_socket, api.get());
//    std::thread t5(&API::run_assign_variable_truth_value, api.get());

    t1.join();
    t2.join();
//    walksatThread.join();
//    t3.join();
//    t4.join();
//    t5.join();

    return EXIT_SUCCESS;
}