//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <opencv2/opencv.hpp>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.0001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 8;
    std::cout << "SPP: " << spp << "\n";
    
    for (uint32_t j = 0; j < scene.height; ++j) {
    #pragma omp parallel for schedule(dynamic, 1)
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[j*scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    cv::Mat image(scene.height, scene.width, CV_32FC3, framebuffer.data());
    image.convertTo(image, CV_8UC3, 255);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imwrite("PathTracing.png",image);
}
