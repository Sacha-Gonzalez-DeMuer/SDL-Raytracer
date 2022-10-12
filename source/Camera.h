#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"
#include <iostream>

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
		float fovAngle{60.f};

		Vector3 forward{.266f, -.453f, .860f};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		const float moveSpeed{ 1.f };

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld()
		{
			 right =  Vector3::Cross(Vector3::UnitY,forward ).Normalized() ;
			 up = Vector3::Cross(forward, right).Normalized() ;
			
			return Matrix{{right}, {up}, {forward}, {origin}};
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float constSpeed{ moveSpeed * deltaTime };

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			origin += (pKeyboardState[SDL_SCANCODE_W] + -pKeyboardState[SDL_SCANCODE_S]) * constSpeed * forward;
			origin += (-pKeyboardState[SDL_SCANCODE_A] + pKeyboardState[SDL_SCANCODE_D]) * constSpeed * right;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			if (mouseState & SDL_BUTTON(1))
			{
				if (mouseState & SDL_BUTTON(3))
				{
					origin -= mouseY * constSpeed * forward;
					origin += mouseX * constSpeed * right;
				}
				else
				{
					origin += mouseY * constSpeed * forward;
					totalYaw += mouseX * constSpeed;
				}
			}


			if (mouseState & SDL_BUTTON(3))
			{
				totalPitch -= mouseY * constSpeed;
				totalYaw += mouseX * constSpeed;
			}


			Matrix finalRotation{ Matrix::CreateRotation(totalPitch, totalYaw, 0) };
			forward = finalRotation.TransformVector(Vector3::UnitZ).Normalized();
		}
	};
}
