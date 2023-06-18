//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

inline Eigen::Vector3f lerpVec(float rate, Eigen::Vector3f a, Eigen::Vector3f b)
{
    return (1-rate) * a + rate * b;
}
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        if(image_data.empty()){
            std::cout << "¼ÓÔØÎÆÀíÊ§°Ü£¡" << std::endl;
        }
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float u_near = std::floor(u_img+0.5f);
        float v_near = std::floor(v_img+0.5f);
        auto color_u00 = image_data.at<cv::Vec3b>(v_near-0.5f, u_near-0.5f);
        auto color_u10 = image_data.at<cv::Vec3b>(v_near-0.5f, u_near+0.5f);
        auto color_u01 = image_data.at<cv::Vec3b>(v_near+0.5f, u_near-0.5f);
        auto color_u11 = image_data.at<cv::Vec3b>(v_near+0.5f, u_near+0.5f);
        Eigen::Vector3f color_v0 = lerpVec(u_img - (u_near-0.5f), Eigen::Vector3f(color_u00[0],color_u00[1],color_u00[2]),Eigen::Vector3f(color_u10[0],color_u10[1],color_u10[2]));
        Eigen::Vector3f color_v1 = lerpVec(u_img - (u_near-0.5f), Eigen::Vector3f(color_u01[0],color_u01[1],color_u01[2]),Eigen::Vector3f(color_u11[0],color_u11[1],color_u11[2]));
        Eigen::Vector3f return_color = lerpVec(v_img - (v_near - 0.5f), color_v0, color_v1);
        return return_color;

    }

};
#endif //RASTERIZER_TEXTURE_H
