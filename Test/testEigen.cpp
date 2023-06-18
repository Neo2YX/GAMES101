#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<random>

float RNG()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f,1.f);

    return dist(rng);
}

int main()
{
    for(int i = 0; i < 10; ++i) std::cout << RNG() << std::endl;
    
    return 0;
}