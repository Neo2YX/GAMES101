// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

float cross(Vector2f v1, Vector2f v2)
{
    return v1.x()*v2.y() - v1.y()*v2.x();
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f e01 = (_v[1]-_v[0]).head(2);
    Vector2f e12 = (_v[2] - _v[1]).head(2);
    Vector2f e20 = (_v[0] - _v[2]).head(2);
    Vector2f p((float)x+0.5,(float)y+0.5);
    Vector2f _0p = p-_v[0].head(2);
    Vector2f _1p = p-_v[1].head(2);
    Vector2f _2p = p-_v[2].head(2);
    float val0 = cross(e01, _0p);
    float val1 = cross(e12, _1p);
    float val2 = cross(e20, _2p);

    if((val0>=0 && val1>=0 && val2 >=0) || (val0<0 && val1<0 && val2 <0)) return true;
    else return false; 

}

static bool finsideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f e01 = (_v[1]-_v[0]).head(2);
    Vector2f e12 = (_v[2] - _v[1]).head(2);
    Vector2f e20 = (_v[0] - _v[2]).head(2);
    Vector2f p(x,y);
    Vector2f _0p = p-_v[0].head(2);
    Vector2f _1p = p-_v[1].head(2);
    Vector2f _2p = p-_v[2].head(2);
    float val0 = cross(e01, _0p);
    float val1 = cross(e12, _1p);
    float val2 = cross(e20, _2p);

    if((val0>=0 && val1>=0 && val2 >=0) || (val0<0 && val1<0 && val2 <0)) return true;
    else return false; 

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

void clampInt(int& x, int a, int b){
    x = std::min(b,std::max(a,x));
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float x_min = std::numeric_limits<float>::max(), y_min = std::numeric_limits<float>::max(), x_max = -std::numeric_limits<float>::max(), y_max = -std::numeric_limits<float>::max();
    for(int i = 0; i<3;++i)
    {
        if(x_min > v[i].x()) x_min = v[i].x();
        if(x_max < v[i].x()) x_max = v[i].x();
        if(y_min > v[i].y()) y_min = v[i].y();
        if(y_max < v[i].y()) y_max = v[i].y();
    }
    int l = std::floor(x_min), r = std::ceil(x_max), b = std::floor(y_min), top = std::ceil(y_max);
    clampInt(l, 0, width-1);
    clampInt(r,0,width-1);
    clampInt(b, 0, height-1);
    clampInt(top,0,height-1);

    for(int i = l; i<=r; ++i)
    {
        for(int j = b; j <= top; ++j)
        {
            int index = get_index(i,j);
            bool flag = false;
            float x = i + 0.25f, y = j+0.25f;
            if(finsideTriangle(x,y,t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(depth_buf[index * 4] > z_interpolated)
                {
                    depth_buf[index * 4] = z_interpolated;
                    frame_color_buf[index * 4] = t.getColor();
                    flag = true;
                }
            }
            x = i + 0.25f;
            y = j+0.75f;
            if(finsideTriangle(x,y,t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(depth_buf[index * 4+1] > z_interpolated)
                {
                    depth_buf[index* 4 +1] = z_interpolated;
                    frame_color_buf[index * 4 + 1] = t.getColor();
                    flag = true;
                }
            }

            x = i + 0.75f;
            y = j+0.25f;
            if(finsideTriangle(x,y,t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(depth_buf[index * 4+2] > z_interpolated)
                {
                    depth_buf[index * 4+2] = z_interpolated;
                    frame_color_buf[index * 4 + 2] = t.getColor();
                    flag = true;
                }
            }


            x = i + 0.75f;
            y = j+0.75f;
            if(finsideTriangle(x,y,t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(depth_buf[index * 4+3] > z_interpolated)
                {
                    depth_buf[index * 4+3] = z_interpolated;
                    frame_color_buf[index * 4 + 3] = t.getColor();
                    flag = true;
                }
            }

            if(flag)
            {
                frame_buf[index] = (frame_color_buf[index * 4] + frame_color_buf[index * 4 +1] + frame_color_buf[index * 4 + 2] + frame_color_buf[index * 4 + 3])/4.f;
            }
            
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(frame_color_buf.begin(), frame_color_buf.end(), Eigen::Vector3f{0,0,0});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h * 4);
    frame_color_buf.resize(w*h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on