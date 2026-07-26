#include <iostream>
#include <cmath>
#include <iomanip>
const double PI = 3.14159265358979;
const double GRAVITY = 9.81;

struct Vector2 {
    double x, y;
};

Vector2 positionAt(double t, Vector2 v0, double g) {
    Vector2 pos;
    pos.x = v0.x * t;
    pos.y = v0.y * t - 0.5 * g * t * t;
    return pos;
}

int main(){
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "Projectile simulator starting..." << std::endl;
    double v0 = 20;
    double angle = 45;
    double t = 0.0;
    double dt = 0.1;
    double converted_to_radian =  angle * PI/180;
    Vector2 initialVelocity = {v0 * cos(converted_to_radian), v0 * sin(converted_to_radian)};
    std::cout << initialVelocity.x << " \n" << initialVelocity.y << std::endl;

    Vector2 pos = {0, 0};
    while(true){
        pos = positionAt(t, initialVelocity, GRAVITY);
        if(pos.y < 0) break;
        std::cout <<"t= " << t << "  x= " << pos.x << "  y= " << pos.y << std::endl;
        t+=dt;
    };

    double t_flight = 2 * initialVelocity.y / GRAVITY;
    double h_max = (initialVelocity.y * initialVelocity.y) / (2 * GRAVITY);
    double range = initialVelocity.x * t_flight;
    std::cout << "t_flight= " << t_flight<< "  h_max= " << h_max<< "  range= "<< range << std::endl;

    return 0;
}