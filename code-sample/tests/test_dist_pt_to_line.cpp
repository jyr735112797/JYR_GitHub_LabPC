#include "svg.hpp"
#include "geometry.hpp"
#include "utils.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace std;
using namespace std::chrono;

double r(double left, double right)
{
    double rr = rand01();
    return left + (right - left) * rr;
}

int main()
{
    srand(time(nullptr));

    int width = 800;
    int height = 600;

    Point o;
    Point a(r(width / 4, width / 2), 100);
    Point b(100, r(height / 4, height / 2));
    Line ab(a, b);
    cout << "a: " << a << endl;
    cout << "b: " << b << endl;
    cout << "ab: " << ab << endl;
    cout << a << " to " << ab << ": " << ab.distTo(a) << endl;
    cout << b << " to " << ab << ": " << ab.distTo(b) << endl;
    cout << o << " to " << ab << ": " << ab.distTo(o) << endl;
    Point c = a + (a - b);
    cout << "c: " << c << endl;
    cout << c << " to " << ab << ": " << ab.distTo(c) << endl;

    Point p(r(width / 4, width / 4 * 3), r(height / 4, height / 4 * 3));
    Point pp;
    cout << p << " to " << ab << ": " << ab.distTo(p, &pp) << endl;

    double stroke_width = 3, radius = 5;
    SVG svg(width, height);
    svg.lines.push_back(
        SVG::Line({a.x, a.y}, {b.x, b.y}, SVG::Color::GREEN, stroke_width));
    svg.lines.push_back(SVG::Line({pp.x, pp.y}, {p.x, p.y},
                                  SVG::Color(255, 165, 0), stroke_width));
    svg.circles.push_back(SVG::Circle({a.x, a.y}, radius, SVG::Color::RED));
    svg.circles.push_back(SVG::Circle({b.x, b.y}, radius, SVG::Color::BLUE));
    svg.circles.push_back(SVG::Circle({p.x, p.y}, radius, SVG::Color::BLACK));
    svg.texts.push_back(SVG::Text({a.x, a.y}, a()));
    svg.texts.push_back(SVG::Text({b.x, b.y}, b()));
    svg.texts.push_back(SVG::Text({p.x, p.y}, p()));

    Point m = (a + b) / 2;
    svg.texts.push_back(
        SVG::Text({m.x, m.y}, "len(line):" + to_string(ab.norm())));
    Point pm = (p + pp) / 2;
    svg.texts.push_back(
        SVG::Text({pm.x, pm.y}, "dist(pt,line):" + to_string(ab.distTo(p))));

    size_t epoch = unix_time();
    string path = to_string(epoch) + ".svg";
    SVG::GRID_STEP = 10;
    SVG::BACKGROUND_COLOR = SVG::Color::WHITE;
    svg.save(path);
    cout << "wrote to '" << path << "'" << endl;

    return 0;
}