//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <cassert>


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
	float&                      tNear,
	uint32_t&                   index,
	Object**                    hitObject) const
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
    Vector3f L_direct   = {};
	Vector3f L_indirect = {};

    // find shading point at first
	Intersection itsct = intersect(ray);
    if (!itsct.happened)
    {
        // return black if we didn't hit any object.
		return Vector3f();
    }

    // if we shot at light source
    if (itsct.m->hasEmission())
    {
		return itsct.m->getEmission();
    }

    auto point = itsct.coords;

	// sample the light
	Intersection lightPos = {};
	float        lightPdf = 0.0;
    sampleLight(lightPos, lightPdf);

    // shot a ray from p to light sample point
	auto lightDir = (lightPos.coords - point).normalized();
	Ray  lightRay(point, lightDir);

    // test if we are blocked by some object
	auto blockItsct    = intersect(lightRay);
	auto lightDistance = distance(point, lightPos.coords);
	bool blocked       = blockItsct.happened && (blockItsct.distance + 0.01 < lightDistance);

    // calculate direct lighting
	if (!blocked)
    {
		auto wi        = lightDir;
		auto wo        = -ray.direction;
		auto brdf      = itsct.m->eval(wi, wo, itsct.normal);
		auto cosTheta  = dotProduct(itsct.normal.normalized(), wi);
		auto cosThetaP = dotProduct(lightPos.normal.normalized(), -wi);
		auto distance2 = std::powf(lightDistance, 2);
		L_direct       = lightPos.emit * brdf * cosTheta * cosThetaP / distance2 / (lightPdf + EPSILON);
    }

    // test Russian Roulette
	float r = get_random_float();
	if (r < RussianRoulette)
    {
        // sample an outgoing ray from shading point
		auto wo = -ray.direction;
		auto wi = itsct.m->sample(wo, itsct.normal);

		Ray sampleRay(point, wi);
        // test if we hit a non-emission object
		auto objectItsct = intersect(sampleRay);
		bool hitNonEmit  = objectItsct.happened && !objectItsct.m->hasEmission();
        if (hitNonEmit)
        {
            // calculate indirect lighting
			auto inRadiance = castRay(sampleRay, depth + 1);
			auto brdf       = itsct.m->eval(wi, wo, itsct.normal);
			auto cosTheta   = dotProduct(itsct.normal.normalized(), wi.normalized());
			const float hemiPdf    = itsct.m->pdf(wi, wo, itsct.normal);
			L_indirect             = inRadiance * brdf * cosTheta / (hemiPdf + EPSILON) / RussianRoulette;
        }
    }

    auto radiance = L_direct + L_indirect;
	return radiance;
}