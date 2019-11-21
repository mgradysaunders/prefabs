#include <iostream>
#include <thread>
#include <preform/bash_format.hpp>

int main()
{
    for (int k = 0; k <= 500; k++) {
        std::cout << '\r';
        std::cout << pr::terminal_progress_bar{k / 500.0};
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << '\n';
    std::cout.flush();
    return EXIT_SUCCESS;
}
