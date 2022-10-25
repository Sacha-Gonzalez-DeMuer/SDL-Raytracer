//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <iostream>
#include <algorithm>
#include <thread> 
#include <future> //async stuff
#include <ppl.h>

using namespace dae;

#define ASYNC
//#define PARALLEL_FOR

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}


void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	camera.CalculateCameraToWorld();

	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	float FOV = tan((TO_RADIANS * camera.fovAngle)/ 2.f);
	float aspectRatio{ static_cast<float>(m_Width) / m_Height };


	const uint32_t numPixels = m_Width * m_Height;

#if defined(ASYNC)
	//async logic
	//..
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};

	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixels = numPixels % numCores;
	uint32_t currPixelIndex{ 0 };

	for(uint32_t coreID{0}; coreID < numCores; ++coreID)
	{
		uint32_t taskSize{ numPixelsPerTask };

		if(numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		async_futures.push_back(
			std::async(std::launch::async, [=, this]
				{
					const uint32_t pixelIndexEnd = currPixelIndex + taskSize;

					for(uint32_t pixelIndex{currPixelIndex}; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPixel(pScene, pixelIndex, FOV, aspectRatio, camera, lights, materials);
					}
				})
		);

		currPixelIndex += taskSize;

	}

	//Wait for all tasks
	for(const std::future<void>& f : async_futures)
	{
		f.wait();
	}


#elif defined(PARALLEL_FOR) //system chooses what happens together
	//parallel for logic
	//..
	concurrency::parallel_for(0u, numPixels, [=, this](int i)
	{
			RenderPixel(pScene, i, FOV, aspectRatio, camera, lights, materials);
	});


#else
	//synchronous logic
	//..
	for(uint32_t i{0}; i< numPixels; ++i)
	{
		RenderPixel(pScene, i, FOV, aspectRatio, camera, lights, materials);
	}

#endif
	

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}



void Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

	float rx = px + .5f;
	float ry = px + .5f;

	float cx = (2 * (rx / float(m_Width)) - 1) * aspectRatio * fov;
	float cy = (1 - (2 * ry / float(m_Height))) * fov;
	
	const Matrix camToWorld{ camera.cameraToWorld };
	Vector3 rayDirection{ 0,0, 1 };
	rayDirection.x = ((2.0f * (px + 0.5f) / float(m_Width)) - 1) * aspectRatio * fov;
	rayDirection.y = (1 - (2.0f * (py + 0.5f) / float(m_Height))) * fov;

	rayDirection = camToWorld.TransformVector(rayDirection).Normalized();
	
	Ray viewRay{ camera.origin, rayDirection };
	ColorRGB finalColor{};
	HitRecord closestHit{};

	pScene->GetClosestHit(viewRay, closestHit);

	if (closestHit.didHit) //FOR EACH PIXEL HIT BY OUR RAY
	{
		for (unsigned int i = 0; i < pScene->GetLights().size(); ++i) //FOR EACH LIGHT IN THE SCENE
		{
			Ray rayToLight{  };
			rayToLight.origin = closestHit.origin + closestHit.normal * .01f;
			rayToLight.direction = LightUtils::GetDirectionToLight(pScene->GetLights()[i], rayToLight.origin);
			rayToLight.max = rayToLight.direction.Magnitude();
			rayToLight.direction.Normalize();

			if (pScene->DoesHit(rayToLight) && m_ShadowsEnabled) //visible & shadowed
			{
				continue;
			} 
			else //visible & unshadowed
			{
				Light light{ pScene->GetLights()[i] };
				float cosAngle = Vector3::Dot(rayToLight.direction, closestHit.normal) <= 0.0f 
					? .0f 
					: Vector3::Dot(rayToLight.direction, closestHit.normal);

				switch (m_CurrentLightingMode)
				{
				case dae::Renderer::LightingMode::ObservedArea:
					finalColor += ColorRGB(1, 1, 1) * cosAngle;
					break;
				case dae::Renderer::LightingMode::Radiance:
					finalColor += LightUtils::GetRadiance(light, closestHit.origin);
					break;
				case dae::Renderer::LightingMode::BRDF:
					finalColor += materials[closestHit.materialIndex]->Shade(closestHit, rayToLight.direction, -viewRay.direction);
					break;
				case dae::Renderer::LightingMode::Combined:
					finalColor += LightUtils::GetRadiance(light, closestHit.origin) * materials[closestHit.materialIndex]->Shade(closestHit, rayToLight.direction, -viewRay.direction) * cosAngle;
					break;
				default:
					break;
				}
			}
		}

		
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}


bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	int current = int(m_CurrentLightingMode);
	++current;
	current = current % 4;
	m_CurrentLightingMode = LightingMode(current);
}