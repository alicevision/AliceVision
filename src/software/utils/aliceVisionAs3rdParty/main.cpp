#include <iostream>
#include <aliceVision/system/cpu.hpp>

int main()
{
    std::cout << "Total cpus " << aliceVision::system::get_total_cpus() << std::endl;
    return EXIT_SUCCESS;
}