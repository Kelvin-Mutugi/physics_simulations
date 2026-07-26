#include <iostream>
#include <cmath>
#include <iomanip>
const double PI = 3.14159265358979;
const double GRAVITY = 9.81;

struct Vector2{
    double x, y;
};

class Projectile{
    public:
        Projectile(double v0, double angleDeg, double gravity){
            double radians = angleDeg * PI / 180;
            initialVelocity.x = v0 * cos(radians);
            initialVelocity.y = v0 * sin(radians);
            g = gravity;
        }

        Vector2 positionAt(double t) const {
            Vector2 pos;
            pos.x = initialVelocity.x * t;
            pos.y = initialVelocity.y * t - 0.5 * g * t * t;
            return pos; 
        }

        double t_flight() const{
            return 2 * initialVelocity.y/g;
        }
        double h_max() const{
            return (initialVelocity.y * initialVelocity.y) / (g*2);
        }
        double range() const{
            return initialVelocity.x * t_flight();
        }

    private:
        Vector2 initialVelocity;
        double g;

};


int main(){
    std::cout << std::fixed << std::setprecision(2);

    Projectile p(20, 45, GRAVITY);
    Vector2 pos = p.positionAt(1.0);
    std::cout << pos.x << " " << pos.y << std::endl;

    std::cout <<"t_flight= " << p.t_flight() 
              <<"  h_max= " << p.h_max()
              <<"  range= " << p.range() << std::endl;
    
    double t = 0.0;
    double dt = 0.1;

    while(true){
        Vector2 pos = p.positionAt(t);
        if (pos.y < 0) break;
        std::cout << "t=" << t << "  x=" << pos.x << "  y=" << pos.y << std::endl;
        t += dt;
    }

    return 0;
}