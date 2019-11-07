#include "svg.hpp"
#include "geometry.hpp"
#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

using namespace std;

void update_svg(SVG &svg, const vector<Point> &points, SVG::Color line_color,
                double line_width, SVG::Color pt_color, double point_radius)
{
    vector<SVG::Point> polyline_points;
    transform(points.begin(), points.end(), back_inserter(polyline_points),
              [](const Point &pt) { return SVG::Point(pt.x, pt.y); });
    for (auto &pt : polyline_points) {
        svg.circles.push_back(
            SVG::Circle({pt.x, pt.y}, point_radius, pt_color));
        svg.width = max(svg.width, pt.x + 20);
        svg.height = max(svg.height, pt.y + 20);
    }
    svg.polylines.push_back(
        SVG::Polyline(polyline_points, line_color, line_width));
}

Point r(Point p0, Point p1, double step = 50)
{
    Point p01 = p1 - p0;
    double rad = rand01() * 360 / 180.0 * 3.14;
    double radius = ((rand01() / 4.0) + 0.4) * p01.norm();
    Point delta = p01 + Point(cos(rad) * radius, sin(rad) * radius);
    double scale = max(step / delta.norm(), 1.0);
    return p1 + delta * scale;
}

vector<Point> douglas(const vector<Point> &points, double thresh = 5)
{
    // TODO
    vector<Point> outputs;          //输出点集
    auto begin = points.begin();
    auto end = points.end() - 1;
    auto maxindex = begin;          //最大距离所对应点的索引
    outputs.push_back(*begin);
    outputs.push_back(*end);
    if(begin == end){
        return outputs;
    }
    Line BeginToEnd(*begin, *end);  //输入点集首尾相连线段
    double max = 0;
    //寻找输入点集距线段距离最大的点*maxindex
    for(auto it = (points.begin() + 1); it != (points.end() - 1); it++)
    {
        if(BeginToEnd.distTo(*it) > max)
        {
            max = BeginToEnd.distTo(*it);
            maxindex = it;
        }
    }
    //output_left,output_right为输入点集被maxindex点分割为左右两个子点集所进行递归的输出结果
    //stamps1,stamps2为其对应的递归输入
    vector<Point> output_left;
    vector<Point> output_right;
    vector<Point> stamps1;
    vector<Point> stamps2;
    if(max >= thresh){
        stamps1.insert(stamps1.end(), begin, maxindex);
        stamps2.insert(stamps2.end(), maxindex, end);
        output_left = douglas(stamps1, thresh);
        output_right = douglas(stamps2, thresh);
    }
    else{
        return outputs;
    }
    //若存在maxindex，则依次将其左子点集的结果、maxindex、右子点集的结果合并返回
    if(!output_left.empty()){
        outputs.insert(outputs.end()-1, output_left.begin()+1, output_left.end()-1);
    }
    outputs.insert(outputs.end()-1, *maxindex);
    if (!output_right.empty()){
        outputs.insert(outputs.end()-1, output_right.begin()+1, output_right.end()-1);
    }
    return outputs;
}

int main()
{
    srand(time(nullptr));

    int width = 800;
    int height = 600;

    Point p0(1000, 1000);
    Point p1(1020, 1020);
    vector<Point> points;
    points.push_back(p0);
    points.push_back(p1);
    for (int i = 0; i < 30; i++) {
        points.push_back(r(points[points.size() - 2], points.back()));
    }
    SVG svg(width, height);
    update_svg(svg, points, SVG::Color::BLACK, 1, SVG::Color::RED, 3);
    vector<Point> points_doug = douglas(points, 50);

    update_svg(svg, points_doug, SVG::Color::BLACK, 3, SVG::Color::GREEN, 5);

    stringstream ss;
    ss << "#points: " << points.size() << " -> " << points_doug.size();
    cout << ss.str() << endl;
    svg.texts.push_back(SVG::Text({50, 50}, ss.str(), SVG::Color::RED, 36));

    size_t epoch = unix_time();
    string path = to_string(epoch) + ".svg";
    SVG::GRID_STEP = 10;
    SVG::BACKGROUND_COLOR = SVG::Color::WHITE;
    svg.save(path);
    cout << "wrote to '" << path << "'" << endl;

    return 0;
}
