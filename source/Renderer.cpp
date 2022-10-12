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

using namespace dae;

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
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();
	float FOV = tan((TO_RADIANS * camera.fovAngle)/ 2.f);
	const Matrix camToWorld{ camera.CalculateCameraToWorld() };
	float aspectRatio{ float(m_Width) / m_Height };

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py) 
		{
			Vector3 rayDirection{ 0,0, 1 };
			rayDirection.x = ((2.0f * (px + 0.5f) / float(m_Width)) - 1) * aspectRatio * FOV;
			rayDirection.y = (1 - (2.0f * (py + 0.5f) / float(m_Height))) * FOV;

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
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
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
