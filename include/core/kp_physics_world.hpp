//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#include <array>
#include <string>

#include "KalaHeaders/core_utils.hpp"
#include "KalaHeaders/math_utils.hpp"
#include "KalaHeaders/log_utils.hpp"

namespace KalaPhysics::Core
{
	using std::array;
	using std::string;
	using std::to_string;
	
	using u8 = uint8_t;
	
	using KalaHeaders::KalaMath::vec3;
	using KalaHeaders::KalaLog::Log;
	using KalaHeaders::KalaLog::LogType;
	
	//64 layers fit in a 64-bit bitmask for uint64_t for bitmasking collisions and colliders
	constexpr u8 MAX_LAYERS = 64;
	constexpr u8 MAX_LAYER_NAME_LENGTH = 50;

	inline const vec3 MAX_GRAVITY = 100.0f;
	
	class LIB_API PhysicsWorld
	{
	public:
		//The main physics update function that handles a
		//single simulation step per call based off of the passed deltaTime variable.
		//Increase substeps above 0 to break a larger simulation step into smaller substeps
		static void Update(
			f32 deltaTime,
			u8 substeps = 0);

		//Returns count of currently used layers
		static inline u64 GetLayerCount() { return layerCount; }

		//Add a new layer
		static inline void AddLayer(const string& layer)
		{
			if (layerCount >= MAX_LAYERS)
			{
				Log::Print(
					"Cannot add a new layer because max layer count '" + to_string(MAX_LAYERS) + "' has been reached!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}

			if (layer == "NONE")
			{
				Log::Print(
					"Cannot add a new layer with the name '" + layer + "' because that name is restricted!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}

			string clamped = layer;
			if (clamped.size() > MAX_LAYER_NAME_LENGTH) clamped.resize(MAX_LAYER_NAME_LENGTH);

			if (GetLayer(clamped) != 255)
			{
				Log::Print(
					"Cannot add a new layer with the name '" + clamped + "' because that name is already in use!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}

			layers[layerCount++] = clamped;
		}
		//Remove an existing layer
		static inline void RemoveLayer(const string& layer)
		{
			u8 layerIndex = GetLayer(layer);

			if (layerIndex == 255)
			{
				Log::Print(
					"Cannot remove an existing layer with the name '" + layer + "' because that name doesn't exist!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}

			layers[layerIndex] = layers[--layerCount];
		}
		//Reset layers
		static inline void RemoveAllLayers()
		{ 
			for (auto& l : layers) l.clear();
			layerCount = 0;
		}
		
		//Get layer name (or "NONE" if not found)
		static inline const string& GetLayer(u8 layer)
		{
			static string none = "NONE";
			
			if (layer >= layerCount) return none;
			
			return layers[layer];
		}
		//Get layer index (or 255 if not found)
		static inline u8 GetLayer(const string& layer)
		{
			for (u8 i = 0; i < layerCount; i++)
			{
				if (layers[i] == layer) return i;
			}
			
			return 255;
		}
		
		//Enable/disable collision between layers
		static inline void SetCollisionRule(
			u8 a,
			u8 b,
			bool value)
		{
			if (GetLayer(a) == "NONE")
			{
				Log::Print(
					"Cannot set collision rule because the first layer does not exist!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}
			if (GetLayer(b) == "NONE")
			{
				Log::Print(
					"Cannot set collision rule because the second layer does not exist!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return;
			}
			
			collisionMatrix[a][b] = value;
			collisionMatrix[b][a] = value;
		}
		//Check if both layers can collide
		static inline bool CanCollide(
			u8 a,
			u8 b)
		{
			if (GetLayer(a) == "NONE")
			{
				Log::Print(
					"Cannot check collision state because the first layer does not exist!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return false;
			}
			if (GetLayer(b) == "NONE")
			{
				Log::Print(
					"Cannot check collision state because the second layer does not exist!",
					"PHYSICS_WORLD",
					LogType::LOG_ERROR,
					2);

				return false;
			}
			
			return collisionMatrix[a][b];
		}

		static inline const vec3& GetGravity() { return gravity; }
		static inline void SetGravity(const vec3& newValue) { gravity = clamp(newValue, vec3(0.0f), MAX_GRAVITY); }
	private:
		static inline array<string, MAX_LAYERS> layers{};
		static inline u8 layerCount{};
		
		static inline bool collisionMatrix[MAX_LAYERS][MAX_LAYERS]{};

		static inline vec3 gravity = vec3(0.0f, -9.81f, 0.0f);
	};
}