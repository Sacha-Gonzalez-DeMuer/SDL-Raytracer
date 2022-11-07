#pragma once
#include <cassert>
#include "Math.h"
#include <iostream>
namespace dae
{
	namespace BRDF
	{
		/**
		 * \param kd Diffuse Reflection Coefficient
		 * \param cd Diffuse Color
		 * \return Lambert Diffuse Color
		 */
		static ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			//todo: W3
			return (cd * kd) / PI;
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{
			return  (cd * kd) / PI;
		}

		/**
		 * \brief todo
		 * \param ks Specular Reflection Coefficient
		 * \param exp Phong Exponent
		 * \param l Incoming (incident) Light Direction
		 * \param v View Direction
		 * \param n Normal of the Surface
		 * \return Phong Specular Color
		 */
		static ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			const Vector3 reflect{ l - (2 * (Vector3::Dot(n, l) ) ) *  n };
			float cosAngle{ Vector3::Dot(reflect, v) };
			if (cosAngle < 0) cosAngle = 0;
			const float specular{ ks * std::powf(cosAngle, exp) };

			return { specular, specular, specular };
		}

		/**
		 * \brief BRDF Fresnel Function >> Schlick
		 * \param h Normalized Halfvector between View and Light directions
		 * \param v Normalized View direction
		 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
		 * \return
		 */
		static ColorRGB FresnelFunction_Schlick(const Vector3& h, const Vector3& v, const ColorRGB& f0)
		{
			float angle{ (Vector3::Dot(h, v)) };
			if (angle < 0) angle = 0;
			
			return f0 + (ColorRGB{ 1,1,1 } - f0) * (std::powf(1 - angle, 5));
		}

		/**
		 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation								- squared(roughness))
		 * \param n Surface normal
		 * \param h Normalized half vector
		 * \param roughness Roughness of the material
		 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
		 */
		static float NormalDistribution_GGX(const Vector3& n, const Vector3& h, float roughness)
		{
			const float sqrtRoughness{ roughness * roughness };
			const float angle{ Vector3::Dot(n, h) };
			const float sqrtAngle{ angle * angle };

			return (sqrtRoughness) / 
				(PI * std::powf(( sqrtAngle * (sqrtRoughness - 1) + 1),2));
		}


		/**
		 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using SchlickGGX
		 */
		static float GeometryFunction_SchlickGGX(const Vector3& n, const Vector3& v, float roughness)
		{
			float angle{ Vector3::Dot(n, v) };
			if (angle < 0) angle = 0;
			const float k{
				((roughness + 1) * (roughness + 1))
				/ 8.0f };

			return { angle / (angle * (1-k) + k) };
		}

		/**
		 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param l Normalized light direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
		 */
		static float GeometryFunction_Smith(const Vector3& n, const Vector3& v, const Vector3& l, float roughness)
		{
			const float schlickView{ GeometryFunction_SchlickGGX(n, v, roughness) };
			const float schlickLight{  GeometryFunction_SchlickGGX(n, l, roughness) };
			const float smith{ schlickView * schlickLight };
			//if (smith < 0) smith = 0;

			return smith;
		}

	}
}