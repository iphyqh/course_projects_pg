/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*<h2>atmosphere/demo/demo.glsl</h2>

<p>This GLSL fragment shader is used to render our demo scene, which consists of
a sphere S on a purely spherical planet P. It is rendered by "ray tracing", i.e.
the vertex shader outputs the view ray direction, and the fragment shader
computes the intersection of this ray with the spheres S and P to produce the
final pixels. The fragment shader also computes the intersection of the light
rays with the sphere S, to compute shadows, as well as the intersections of the
view ray with the shadow volume of S, in order to compute light shafts.

<p>Our fragment shader has the following inputs and outputs:
*/
#version 330
const float kLengthUnitInMeters = 1000.000000;
uniform vec3 camera;
uniform float exposure;
uniform vec3 white_point;
uniform vec3 earth_center;
uniform vec3 sun_direction;
uniform vec2 sun_size;
uniform float f_threshold;
uniform float u_time;
in vec3 view_ray;
layout(location = 0) out vec4 color;

/* 
Note by @HE Qihao for COMP5411: With "Additional", it means codes that we added to Bruneton's
implementation, which are used to mainly deal with cloud rendering, and is our main task
for this project.

Reference list: 
WangYiYun zhihu.com/people/di-si-zhang-pai-77 on Cloud Rendering
Scratchapixel scratchapixel.com/lessons/3d-basic-rendering/volume-rendering-for-developers/ on Cloud Rendering Fundamentals
Sebastian Lague @ Code Adventure on Cloud Rendering
Inigo Quilez https://iquilezles.org/articles/derivative/ on Cloud Rendering
DanielScherzer https://github.com/danielscherzer/SHADER/blob/master/raymarching/raymarching%20clouds.glsl on noise generation
*/

/*
<p>It uses the following constants, as well as the following atmosphere
rendering functions, defined externally (by the <code>Model</code>'s
<code>GetShader()</code> shader). The <code>USE_LUMINANCE</code> option is used
to select either the functions returning radiance values, or those returning
luminance values (see <a href="../model.h.html">model.h</a>).
*/
const float INF = 9999.0;
const float PI = 3.14159265;
const int RAY_MARCHING_STEPS = 128;
const int LIGHT_MARCH_NUM = 16; // May be raised later but be careful of the cost
const int OCTAVES = 4; // Number of different frequencies of noise we sample for the noise, bigger means more detailed noise, but not necessarily better clouds
const vec3 kSphereCenter = vec3(0.0, 0.0, 1000.0) / kLengthUnitInMeters;
const float kSphereRadius = 0.0 / kLengthUnitInMeters;
const vec3 kSphereAlbedo = vec3(0.8);
const vec3 kGroundAlbedo = vec3(0.0, 0.0, 0.04);
const vec3 cloud_color = vec3(1,0.5,0.5);
const vec3 box_scale = vec3(50000.0, 50000.0, 1000.0) / kLengthUnitInMeters;
const vec3 box_position = vec3(6000.0, 6000.0, 10000.0) / kLengthUnitInMeters;
// -------------------------------------------------------

// Additional: Ray, box and ray-box intersection (RayHit) structures
struct Ray
{
    vec3 origin;
    vec3 direction;
};

Ray CreateRay(vec3 origin, vec3 direction)
{
    Ray ray;
    ray.origin = origin;
    ray.direction = direction;
    return ray;
}

struct RayHit{
	vec3 position;
	float hitDist;
	float alpha;
	float entryPoint;
	float exitPoint;
};

RayHit CreateRayHit()
{
	RayHit hit;
	hit.position = vec3(0.0,0.0,0.0);
	hit.hitDist = INF;
	hit.alpha = 0.0;
	hit.entryPoint = 0.0;
	hit.exitPoint = INF;
	return hit;
}

struct Box
{
	vec3 position;
	vec3 scale;
};

Box CreateBox(vec3 position, vec3 scale)
{
	Box box;
	box.position = position;
	box.scale = scale;
	return box;
}
//

// Additional: Noise Generation
float hash(vec3 p)
{
    p  = fract( p*0.3183099 + 0.1 );
	  p *= 17.0;
    return fract( p.x*p.y*p.z*(p.x+p.y+p.z) );
}

float Remap(float v, float l0, float h0, float ln, float hn)
{
	return ln + ((v - l0) * (hn - ln)) / (h0 - l0);
}

// unit saturation function
float saturation(float v)
{
	if(v > 1.0) return 1.0;
	if(v < 0.0) return 0.0;
	return v;
}

float noise( in vec3 x )
{
	x *= 2.0;
  vec3 p = floor(x);
  vec3 f = fract(x);
  f = f*f*(3.0-2.0*f);

  return mix(mix(mix( hash(p+vec3(0,0,0)), 
                      hash(p+vec3(1,0,0)),f.x),
                  mix( hash(p+vec3(0,1,0)), 
                      hash(p+vec3(1,1,0)),f.x),f.y),
              mix(mix( hash(p+vec3(0,0,1)), 
                      hash(p+vec3(1,0,1)),f.x),
                  mix( hash(p+vec3(0,1,1)), 
                      hash(p+vec3(1,1,1)),f.x),f.y),f.z);
}

float fBm(vec3 p, const int octaves )
{
	float f = 0.0;
	float weight = 0.5;
	for(int i = 0; i < octaves; ++i)
	{
		f += weight * noise( p );
		weight *= 0.5;
		p *= 2.0;
	}
	return f;
}

float densityFunc(const vec3 p, Box box)
{
	vec3 q = p;

	float boxBottom = box.position.y - box.scale.y;
	float heightPercent = (p.y - boxBottom) / (box.scale.y);
  // Two linear remapping functions that maps heightPercent to two values and clamp it between 0.0 and 1.0,
  // multiplied as a quadratic function on height percentage to further erode the cloud
	float heightErosion = saturation(Remap(heightPercent, 0.0, 0.2, 0.0, 1.0)) * saturation(Remap(heightPercent, 1.0, 0.7, 0.0, 1.0));

	q += vec3(0.0, 0.10, 1.0) * (1.0 + u_time); // Time varying density to create
	float f = fBm(q, OCTAVES)*heightErosion; // Further erode the cloud
  if (f > f_threshold) // Important for the cloud box be adjusted with more gaps or less gaps
	return 2.0*f;
    else
  return -1.0;
}
//

// Check for intersection between a ray (camera or light ray) and a box, 
///and return the Hit in-point (and corresponding out-point) information
RayHit RayIntersectBox(Ray ray, Box box)
{
  RayHit bestHit = CreateRayHit();
	vec3 minBound = box.position - box.scale;
	vec3 maxBound = box.position + box.scale;

	vec3 t0 = (minBound - ray.origin) / ray.direction;
	vec3 t1 = (maxBound - ray.origin) / ray.direction;

	vec3 tsmaller = min(t0, t1);
	vec3 tbigger = max(t0, t1);

	float tmin = max(tsmaller[0], max(tsmaller[1], tsmaller[2]));
  float tmax = min(tbigger[0], min(tbigger[1], tbigger[2]));

  // Box Miss
	if(tmin > tmax) return bestHit;

	// Box Hit
  if(tmax > 0.0 && tmin < bestHit.hitDist)
	{
		if(tmin < 0.0) tmin = 0.0;
		bestHit.hitDist = tmin;
		bestHit.position = ray.origin + bestHit.hitDist * ray.direction;
		bestHit.alpha = 1.0;
		bestHit.entryPoint = tmin;
		bestHit.exitPoint = tmax;
	}
  return bestHit;
}

// Additional: Volume rendering 
// Light ray marching through the cloud bounding box
float lightMarch(vec3 origin, Box box)
{
	float totalDensity = 0.0; 
	vec3 light_direction = normalize(sun_direction);
	Ray lightRay = CreateRay(origin, light_direction);
	RayHit lightHit = RayIntersectBox(lightRay, box);

	// Distance inside the cloud box
	float stepSize = abs(lightHit.exitPoint - lightHit.entryPoint) / float(LIGHT_MARCH_NUM);
  
	for(int i = 0; i < RAY_MARCHING_STEPS; i++)
	{
		origin += light_direction*stepSize;
		totalDensity += 1.0; // Instead of sampling different densities along the ray, we decided to treat it as constant along the light ray path
	}

	float transmittance = exp(-totalDensity*2.0);
	float darknessThreshold = 0.3; // To avoid the volume from getting too dark
	return darknessThreshold + transmittance*(1.0-darknessThreshold);
}

// Render on a given path in the volume
void RenderVolume(Ray ray, RayHit hit, inout vec4 color, Box box)
{
  vec3 start = hit.position;
	vec3 end = hit.position + ray.direction*abs(hit.exitPoint-hit.entryPoint);
	float len = distance(start,end);
	float stepSize = len / float(RAY_MARCHING_STEPS);

	vec3 eachStep =  stepSize * normalize(end - start);
	vec3 currentPos = start;

	// exp for lighting, Beer's law
	// How much light will reflect by the cloud.
	float lightEnergy = 0.0;
	// if transmittance = 1, total transmission, which means sky's color.
	float transmittance = 1.0;
	for(int i = 0; i < RAY_MARCHING_STEPS; i++)
	{
    float density = densityFunc(currentPos, box);
    if(density > 0.0)
    {
      // Sample the cloud internal point received light following the path from the direction of the sun
      float lightTransmittance = lightMarch(currentPos, box);
      lightEnergy += density*stepSize*transmittance*lightTransmittance; // Riemann Sum
      // larger density, smaller transmittance
      transmittance *= exp(-density*stepSize*0.643);

      if(transmittance < 0.01 || lightEnergy > 2.0)
      {
      	break; // Early break
      }

    }
    currentPos += eachStep;	 	
	}

	 vec3 cloudColor = lightEnergy * cloud_color; // cloud_color can be manipulated to reflect different colors
	 color.rgb += color.rgb*transmittance + cloudColor;
}
//

#ifdef USE_LUMINANCE
#define GetSolarRadiance GetSolarLuminance
#define GetSkyRadiance GetSkyLuminance
#define GetSkyRadianceToPoint GetSkyLuminanceToPoint
#define GetSunAndSkyIrradiance GetSunAndSkyIlluminance
#endif

vec3 GetSolarRadiance();
vec3 GetSkyRadiance(vec3 camera, vec3 view_ray, float shadow_length,
    vec3 sun_direction, out vec3 transmittance);
vec3 GetSkyRadianceToPoint(vec3 camera, vec3 point, float shadow_length,
    vec3 sun_direction, out vec3 transmittance);
vec3 GetSunAndSkyIrradiance(
    vec3 p, vec3 normal, vec3 sun_direction, out vec3 sky_irradiance);

/*<h3>Shadows and light shafts</h3>

<p>The functions to compute shadows and light shafts must be defined before we
can use them in the main shader function, so we define them first. Testing if
a point is in the shadow of the sphere S is equivalent to test if the
corresponding light ray intersects the sphere, which is very simple to do.
However, this is only valid for a punctual light source, which is not the case
of the Sun. In the following function we compute an approximate (and biased)
soft shadow by taking the angular size of the Sun into account:
*/

float GetSunVisibility(vec3 point, vec3 sun_direction) {
  vec3 p = point - kSphereCenter;
  float p_dot_v = dot(p, sun_direction);
  float p_dot_p = dot(p, p);
  float ray_sphere_center_squared_distance = p_dot_p - p_dot_v * p_dot_v;
  float distance_to_intersection = -p_dot_v - sqrt(
      kSphereRadius * kSphereRadius - ray_sphere_center_squared_distance);
  if (distance_to_intersection > 0.0) {
    // Compute the distance between the view ray and the sphere, and the
    // corresponding (tangent of the) subtended angle. Finally, use this to
    // compute an approximate sun visibility.
    float ray_sphere_distance =
        kSphereRadius - sqrt(ray_sphere_center_squared_distance);
    float ray_sphere_angular_distance = -ray_sphere_distance / p_dot_v;
    return smoothstep(1.0, 0.0, ray_sphere_angular_distance / sun_size.x);
  }
  return 1.0;
}

/*
<p>The sphere also partially occludes the sky light, and we approximate this
effect with an ambient occlusion factor. The ambient occlusion factor due to a
sphere is given in <a href=
"http://webserver.dmt.upm.es/~isidoro/tc3/Radiation%20View%20factors.pdf"
>Radiation View Factors</a> (Isidoro Martinez, 1995). In the simple case where
the sphere is fully visible, it is given by the following function:
*/

float GetSkyVisibility(vec3 point) {
  vec3 p = point - kSphereCenter;
  float p_dot_p = dot(p, p);
  return
      1.0 + p.z / sqrt(p_dot_p) * kSphereRadius * kSphereRadius / p_dot_p;
}

void GetSphereShadowInOut(vec3 view_direction, vec3 sun_direction,
    out float d_in, out float d_out) {
  vec3 pos = camera - kSphereCenter;
  float pos_dot_sun = dot(pos, sun_direction);
  float view_dot_sun = dot(view_direction, sun_direction);
  float k = sun_size.x;
  float l = 1.0 + k * k;
  float a = 1.0 - l * view_dot_sun * view_dot_sun;
  float b = dot(pos, view_direction) - l * pos_dot_sun * view_dot_sun -
      k * kSphereRadius * view_dot_sun;
  float c = dot(pos, pos) - l * pos_dot_sun * pos_dot_sun -
      2.0 * k * kSphereRadius * pos_dot_sun - kSphereRadius * kSphereRadius;
  float discriminant = b * b - a * c;
  if (discriminant > 0.0) {
    d_in = max(0.0, (-b - sqrt(discriminant)) / a);
    d_out = (-b + sqrt(discriminant)) / a;
    // The values of d for which delta is equal to 0 and kSphereRadius / k.
    float d_base = -pos_dot_sun / view_dot_sun;
    float d_apex = -(pos_dot_sun + kSphereRadius / k) / view_dot_sun;
    if (view_dot_sun > 0.0) {
      d_in = max(d_in, d_apex);
      d_out = a > 0.0 ? min(d_out, d_base) : d_base;
    } else {
      d_in = a > 0.0 ? max(d_in, d_base) : d_base;
      d_out = min(d_out, d_apex);
    }
  } else {
    d_in = 0.0;
    d_out = 0.0;
  }
}

/*<h3>Main shading function</h3>

<p>Using these functions we can now implement the main shader function, which
computes the radiance from the scene for a given view ray. This function first
tests if the view ray intersects the sphere S. If so it computes the sun and
sky light received by the sphere at the intersection point, combines this with
the sphere BRDF and the aerial perspective between the camera and the sphere.
It then does the same with the ground, i.e. with the planet sphere P, and then
computes the sky radiance and transmittance. Finally, all these terms are
composited together (an opacity is also computed for each object, using an
approximate view cone - sphere intersection factor) to get the final radiance.

<p>We start with the computation of the intersections of the view ray with the
shadow volume of the sphere, because they are needed to get the aerial
perspective for the sphere and the planet:
*/

void main() {
  // Normalized view direction vector.
  vec3 view_direction = normalize(view_ray);
  // Tangent of the angle subtended by this fragment.
  float fragment_angular_size =
      length(dFdx(view_ray) + dFdy(view_ray)) / length(view_ray);

  float shadow_in;
  float shadow_out;
  GetSphereShadowInOut(view_direction, sun_direction, shadow_in, shadow_out);

  // Hack to fade out light shafts when the Sun is very close to the horizon.
  float lightshaft_fadein_hack = smoothstep(
      0.02, 0.04, dot(normalize(camera - earth_center), sun_direction));

/*
<p>We then test whether the view ray intersects the sphere S or not. If it does,
we compute an approximate (and biased) opacity value, using the same
approximation as in <code>GetSunVisibility</code>:
*/

  // Compute the distance between the view ray line and the sphere center,
  // and the distance between the camera and the intersection of the view
  // ray with the sphere (or NaN if there is no intersection).
  vec3 p = camera - kSphereCenter;
  float p_dot_v = dot(p, view_direction);
  float p_dot_p = dot(p, p);
  float ray_sphere_center_squared_distance = p_dot_p - p_dot_v * p_dot_v;
  float distance_to_intersection = -p_dot_v - sqrt(
      kSphereRadius * kSphereRadius - ray_sphere_center_squared_distance);

  // Compute the radiance reflected by the sphere, if the ray intersects it.
  float sphere_alpha = 0.0;
  vec3 sphere_radiance = vec3(0.0);

  // Cloud rendering
  vec3 boxPos = box_position;
  vec3 boxScale = box_scale;
  Ray cameraRay = CreateRay(camera, normalize(view_ray));
  Box box = CreateBox(boxPos, boxScale);
	RayHit hit = RayIntersectBox(cameraRay, box);
  if (hit.alpha != 0.0) {
    RenderVolume(cameraRay,hit,color,box);
  }
  //

  if (distance_to_intersection > 0.0 && (sqrt(ray_sphere_center_squared_distance) > kSphereRadius/2.0 || view_ray[0] > 0.0)) {
    // Compute the distance between the view ray and the sphere, and the
    // corresponding (tangent of the) subtended angle. Finally, use this to
    // compute the approximate analytic antialiasing factor sphere_alpha.
    float ray_sphere_distance =
        kSphereRadius - sqrt(ray_sphere_center_squared_distance);
    float ray_sphere_angular_distance = -ray_sphere_distance / p_dot_v;
    sphere_alpha =
        min(ray_sphere_angular_distance / fragment_angular_size, 1.0);

/*
<p>We can then compute the intersection point and its normal, and use them to
get the sun and sky irradiance received at this point. The reflected radiance
follows, by multiplying the irradiance with the sphere BRDF:
*/
    vec3 point = camera + view_direction * distance_to_intersection;
    vec3 normal = normalize(point - kSphereCenter);

    // Compute the radiance reflected by the sphere.
    vec3 sky_irradiance;
    vec3 sun_irradiance = GetSunAndSkyIrradiance(
        point - earth_center, normal, sun_direction, sky_irradiance);
    sphere_radiance =
        kSphereAlbedo * (1.0 / PI) * (sun_irradiance + sky_irradiance);

/*
<p>Finally, we take into account the aerial perspective between the camera and
the sphere, which depends on the length of this segment which is in shadow:
*/
    float shadow_length =
        max(0.0, min(shadow_out, distance_to_intersection) - shadow_in) *
        lightshaft_fadein_hack;
    vec3 transmittance;
    vec3 in_scatter = GetSkyRadianceToPoint(camera - earth_center,
        point - earth_center, shadow_length, sun_direction, transmittance);
    sphere_radiance = sphere_radiance * transmittance + in_scatter;
  }

/*
<p>In the following we repeat the same steps as above, but for the planet sphere
P instead of the sphere S (a smooth opacity is not really needed here, so we
don't compute it. Note also how we modulate the sun and sky irradiance received
on the ground by the sun and sky visibility factors):
*/

  // Compute the distance between the view ray line and the Earth center,
  // and the distance between the camera and the intersection of the view
  // ray with the ground (or NaN if there is no intersection).
  p = camera - earth_center;
  p_dot_v = dot(p, view_direction);
  p_dot_p = dot(p, p);
  float ray_earth_center_squared_distance = p_dot_p - p_dot_v * p_dot_v;
  distance_to_intersection = -p_dot_v - sqrt(
      earth_center.z * earth_center.z - ray_earth_center_squared_distance);

  // Compute the radiance reflected by the ground, if the ray intersects it.
  float ground_alpha = 0.0;
  vec3 ground_radiance = vec3(0.0);
  if (distance_to_intersection > 0.0) {
    vec3 point = camera + view_direction * distance_to_intersection;
    vec3 normal = normalize(point - earth_center);

    // Compute the radiance reflected by the ground.
    vec3 sky_irradiance;
    vec3 sun_irradiance = GetSunAndSkyIrradiance(
        point - earth_center, normal, sun_direction, sky_irradiance);
    ground_radiance = kGroundAlbedo * (1.0 / PI) * (
        sun_irradiance * GetSunVisibility(point, sun_direction) +
        sky_irradiance * GetSkyVisibility(point));

    float shadow_length =
        max(0.0, min(shadow_out, distance_to_intersection) - shadow_in) *
        lightshaft_fadein_hack;
    vec3 transmittance;
    vec3 in_scatter = GetSkyRadianceToPoint(camera - earth_center,
        point - earth_center, shadow_length, sun_direction, transmittance);
    ground_radiance = ground_radiance * transmittance + in_scatter;
    ground_alpha = 1.0;
  }

/*
<p>Finally, we compute the radiance and transmittance of the sky, and composite
together, from back to front, the radiance and opacities of all the objects of
the scene:
*/

  // Compute the radiance of the sky.
  float shadow_length = max(0.0, shadow_out - shadow_in) *
      lightshaft_fadein_hack;
  vec3 transmittance;
  vec3 radiance = GetSkyRadiance(
      camera - earth_center, view_direction, shadow_length, sun_direction,
      transmittance);

  // If the view ray intersects the Sun, add the Sun radiance.
  if (dot(view_direction, sun_direction) > sun_size.y) {
    radiance = radiance + transmittance * GetSolarRadiance();
  }
  radiance = mix(radiance, ground_radiance, ground_alpha);
  radiance = mix(radiance, sphere_radiance, sphere_alpha);
  color.rgb += 
      pow(vec3(1.0) - exp(-radiance / white_point * exposure), vec3(1.0 / 2.2));
  color.a = 1.0;
}
