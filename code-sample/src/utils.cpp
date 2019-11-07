#include "utils.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace std::chrono;

void writeStr(std::string path, const std::string &str)
{
    ofstream file(path);
    file << str;
    file.close();
}

size_t unix_time() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch())
        .count();
}

double rand01() {
    return rand() / (double)RAND_MAX;
}