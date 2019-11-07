#include "svg.hpp"
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

double SVG::GRID_STEP = -1;
SVG::Color SVG::BACKGROUND_COLOR = SVG::Color(-1);

const SVG::Color SVG::Color::RED = SVG::Color(255, 0, 0);
const SVG::Color SVG::Color::GREEN = SVG::Color(0, 255, 0);
const SVG::Color SVG::Color::BLUE = SVG::Color(0, 0, 255);
const SVG::Color SVG::Color::BLACK = SVG::Color(0, 0, 0);
const SVG::Color SVG::Color::WHITE = SVG::Color(255, 255, 255);

void SVG::save(std::string path) const
{
    ofstream file(path);
    file << *this;
    file.close();
}

std::ostream &operator<<(std::ostream &out, const SVG::Color &c)
{
    out << "rgb(" << c.r << "," << c.g << "," << c.b << ")";
    return out;
}

std::ostream &operator<<(std::ostream &out, const SVG::Circle &c)
{
    out << "<circle r='" << c.r << "'"                  //
        << " cx='" << c.p.x << "' cy='" << c.p.y << "'" //
        << " style='stroke:" << c.stroke                //
        << ";stroke-width:" << c.stroke_width           //
        << ";fill:" << c.fill << "'"                    //
        << " />";
    return out;
}

std::ostream &operator<<(std::ostream &out, const SVG::Line &l)
{
    out << "<line"                                        //
        << " x1='" << l.p1.x << "' y1='" << l.p1.y << "'" //
        << " x2='" << l.p2.x << "' y2='" << l.p2.y << "'" //
        << " style='stroke:" << l.stroke                  //
        << ";stroke-width:" << l.stroke_width             //
        << "' />";
    return out;
}

std::ostream &operator<<(std::ostream &out, const SVG::Text &t)
{
    out << "<text"                                    //
        << " x='" << t.p.x << "' y='" << t.p.y << "'" //
        << " fill='" << t.fill << "'"                 //
        << " font-size='" << t.fontsize << "'"        //
        << ">" << t.text << "</text>";
    return out;
}

std::ostream &operator<<(std::ostream &out, const SVG::Polyline &p)
{
    out << "<polyline"                        //
        << " style='stroke:" << p.stroke      //
        << ";stroke-width:" << p.stroke_width //
        << ";fill:none'";
    out << " points='";
    for (auto &pt : p.points) {
        out << pt.x << "," << pt.y << " ";
    }
    out << "'";
    out << " />";
    return out;
}

std::ostream &operator<<(std::ostream &out, const SVG &s)
{
    out << "<svg width='" << s.width << "' height='" << s.height
        << "' xmlns='http://www.w3.org/2000/svg'>";
    if (!SVG::BACKGROUND_COLOR.invalid()) {
        out << "\n\t<rect width='100%' height='100%' fill='" //
            << SVG::BACKGROUND_COLOR                         //
            << "'/>";
    }
    if (SVG::GRID_STEP > 0) {
        SVG::Color grid_color(155, 155, 155);
        for (double i = 0; i < s.height; i += SVG::GRID_STEP) {
            out << "\n\t" << SVG::Line({0, i}, {s.width, i}, grid_color);
        }
        for (double j = 0; j < s.width; j += SVG::GRID_STEP) {
            out << "\n\t" << SVG::Line({j, 0}, {j, s.height}, grid_color);
        }
    }

    for (auto &l : s.lines) {
        out << "\n\t" << l;
    }
    for (auto &p : s.polylines) {
        out << "\n\t" << p;
    }
    for (auto &c : s.circles) {
        out << "\n\t" << c;
    }
    for (auto &t : s.texts) {
        out << "\n\t" << t;
    }
    out << "\n</svg>";
    return out;
}