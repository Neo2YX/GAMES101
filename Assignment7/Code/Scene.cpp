//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
const Vector3f BLACK(0.f,0.f,0.f);


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //if(depth > maxDepth) return BLACK;

    Intersection isec = intersect(ray);
    if(isec.happened)
    {
        if(isec.m->hasEmission()) return isec.m->getEmission();
        Vector3f directLight, indirectLight;

        //calculate directLight
        Intersection lightPosIsec;
        float lightSamplePdf;
        sampleLight(lightPosIsec, lightSamplePdf);
        Vector3f light2obj = isec.coords - lightPosIsec.coords;
        Vector3f lightRay = normalize(light2obj);
        float distance = light2obj.norm();
        float dist2 = distance*distance;
        Intersection lightRayIsec = intersect(Ray(isec.coords,-lightRay));
        //check if lightRay be blocked
        if(std::fabs(distance-lightRayIsec.distance) < EPSILON)
        {
            //calculate the rendering function
            Vector3f brdf = isec.m->eval(lightRay, -ray.direction, isec.normal);
            float geoFact = dotProduct(-lightRay, isec.normal) * dotProduct(lightRay,lightPosIsec.normal) / dist2;
            directLight = brdf * lightRayIsec.m->getEmission() * geoFact / lightSamplePdf;
        }else directLight = BLACK;
        return directLight;
        //calculate indirectLight
        Vector3f brdfLight = (isec.m->sample(ray.direction, isec.normal)).normalized();
        Intersection brdfRayIsec = intersect(Ray(isec.coords, brdfLight));
        float RR = get_random_float();
        if(RR<RussianRoulette && brdfRayIsec.happened && !brdfRayIsec.m->hasEmission())
        {
            //calculate the rendering function
            Vector3f brdf = isec.m->eval(-brdfLight, -ray.direction, isec.normal);
            float brdfSamplePdf = isec.m->pdf(ray.direction, brdfLight, isec.normal);  
            indirectLight = castRay(Ray(isec.coords,brdfLight), depth+1);
            indirectLight = indirectLight * brdf * dotProduct(brdfLight, isec.normal) / brdfSamplePdf / RussianRoulette;
        }else indirectLight = BLACK;
        
        return directLight + indirectLight;

    }

    return BLACK;
}