#include "svg.hpp"
#include "utils.hpp"

#include <iostream>
#include <sstream>

using namespace std;

int main(void)
{
    SVG svg(40, 30);
    svg.lines.push_back(
        SVG::Line(SVG::Point(0, 0), SVG::Point(10, 20), SVG::Color::RED));
    svg.lines.push_back(SVG::Line(SVG::Point(5, 6), SVG::Point(23.34, 23.2),
                                  SVG::Color::GREEN));
    svg.circles.push_back(SVG::Circle(SVG::Point(10, 10), 4, SVG::Color::BLACK,
                                      SVG::Color::GREEN));
    svg.texts.push_back(
        SVG::Text(SVG::Point(8, 6), "some text", SVG::Color::RED, 8));
    svg.polylines.push_back(SVG::Polyline({
        {1.1, 2.2}, {5.1, 3.2}, {8.1, 9.2},
    }));

    stringstream ss;
    ss << svg << endl;
    cout << svg << endl;

    size_t epoch = unix_time();
    string path = to_string(epoch) + ".svg";
    svg.save(path);
    cout << "wrote to '" << path << "'" << endl;
    return 0;
}