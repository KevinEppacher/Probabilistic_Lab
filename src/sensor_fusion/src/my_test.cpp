#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>

// Funktion zum Generieren einer Zufallsprobe basierend auf einer Gaußschen Verteilung
double sample(double stddev) 
{
    double u1 = (std::rand() + 1.0) / (RAND_MAX + 1.0);
    double u2 = (std::rand() + 1.0) / (RAND_MAX + 1.0);
    return stddev * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

// Funktion zur Berechnung von v_hat
double calculate_v_hat(double v, double omega, double alpha1, double alpha2) 
{
    double noise = sample(alpha1 * std::abs(v) + alpha2 * std::abs(omega));
    return v + noise;
}

int main() {
    std::srand(std::time(0)); // Initialisiere den Zufallsgenerator

    double v = 1.0; // Beispielwert für v
    double omega = 0.5; // Beispielwert für omega
    double alpha1 = 0.1; // Beispielwert für alpha1
    double alpha2 = 0.1; // Beispielwert für alpha2

    double v_hat = calculate_v_hat(v, omega, alpha1, alpha2);

    std::cout << "v_hat: " << v_hat << std::endl;

    return 0;
}
