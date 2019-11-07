#include <iostream>
#include <string>

// fix me
void print(const char *str) {
    std::cout << "str: ";
    while (str) {
        std::cout << *str;
        ++str;
    }
    std::cout << std::endl;
}

int main() {
    const char *str1 = "one two three";
    std::string str2 = "four five six";
    print(str1);
    print(str2.c_str());
    return 0;
}
