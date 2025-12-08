//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <string>

#include "KalaHeaders/log_utils.hpp"

#include "core/kp_core.hpp"

using KalaHeaders::KalaLog::Log;
using KalaHeaders::KalaLog::LogType;

using std::to_string;

namespace KalaPhysics::Core
{
	void KalaPhysicsCore::CleanAllWindowResources(u32 windowID)
	{
		Log::Print(
			"Cleaning resources for window '" + to_string(windowID) + "'.",
			"KALAPHYSICS",
			LogType::LOG_INFO);
	}
	
	void KalaPhysicsCore::CleanAllResources()
	{
		Log::Print(
			"Cleaning all KalaPhysics resources.",
			"KALAPHYSICS",
			LogType::LOG_INFO);
	}
}