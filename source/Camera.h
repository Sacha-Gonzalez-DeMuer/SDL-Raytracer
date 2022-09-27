#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};

		Vector3 forward{.266f, -.453f, .860f};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			//todo: W2
			Vector3 rightOBN{ Vector3::Cross(forward, up).Normalized() };
			Vector3 upOBN{ Vector3::Cross(rightOBN, forward).Normalized() };


			/*Matrix Yaw{
				{cos(totalYaw), 0, -sin(totalYaw), 0},
				{0,1,0,0},
				{sin(totalYaw), 0, cos(totalYaw), 0},
				{0,0,0,1} };

			Matrix Pitch{
				{1, 0, 0, 0},
				{0, cos(totalPitch), -sin(totalPitch), 0},
				{0, sin(totalPitch), cos(totalPitch), 0},
				{0,0,0,1}
			};*/
			
			return Matrix{{rightOBN, 0}, {upOBN, 0}, {forward, 0}, {origin, 1}};
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			//todo: W2
			//assert(false && "Not Implemented Yet");
		}
	};
}
