#include <iostream>
#include <cmath>

double cubicEquation(double x, double a, double b, double c, double d) {
    return a * pow(x, 3) + b * pow(x, 2) + c * x + d;
}

double cubicEquationDerivative(double x, double a, double b, double c) {
    return 3 * a * pow(x, 2) + 2 * b * x + c;
}

double findClosestPoint(double x_p, double y_p, double a, double b, double c, double d) {
    double x = x_p; // Initial guess for x using the point's x-coordinate

    for (int i = 0; i < 100; ++i) { // Perform up to 100 iterations
        double y = cubicEquation(x, a, b, c, d);
        double dy_dx = cubicEquationDerivative(x, a, b, c);
        x = x - (y - y_p) / dy_dx; // Newton's method update rule
    }

    return x;
}

double findMinimumDistance(double a, double b, double c, double d, double x_p, double y_p) {
    double minDistance = std::numeric_limits<double>::max();
    double minX = 0.0; // Initialize with an arbitrary value
    double stepSize = 0.001; // Adjust the step size as needed

    for (double x = -10.0; x <= 10.0; x += stepSize) { // Adjust the range as needed
        double distance = distanceFunction(a, b, c, d, x_p, y_p, x);
        if (distance < minDistance) {
            minDistance = distance;
            minX = x;
        }
    }
    return minX;
}


int main() {
    double a, b, c, d;
    double x_p, y_p;

    // Input coefficients of the cubic equation and point's coordinates
    std::cout << "Enter coefficients a, b, c, d: ";
    std::cin >> a >> b >> c >> d;

    std::cout << "Enter point's coordinates x_p, y_p: ";
    std::cin >> x_p >> y_p;

    double closest_x = findClosestPoint(x_p, y_p, a, b, c, d);
    double closest_y = cubicEquation(closest_x, a, b, c, d);

    double distance = std::sqrt(pow(x_p - closest_x, 2) + pow(y_p - closest_y, 2));
    std::cout << "Distance between the point and the cubic equation: " << distance << std::endl;

    return 0;
}
