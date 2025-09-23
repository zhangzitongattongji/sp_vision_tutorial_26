#include <fmt/core.h>
#include <string>
using namespace fmt;
using namespace std;
int main() {
    print("Hello, {}!\n", "fmt");
    string s = format("Pi ~= {:.3f}", 3.14159);
    print("{}\n", s);
    return 0;
}
