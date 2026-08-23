//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#ifdef __linux__
#include <csignal>
#endif

#include "log_utils.hpp"

#include "core/kp_core.hpp"

using KalaHeaders::KalaLog::Log;
using KalaHeaders::KalaLog::LogType;
using KalaHeaders::KalaLog::TimeFormat;
using KalaHeaders::KalaLog::DateFormat;

#ifdef __linux__
using std::raise;
#endif

namespace KalaPhysics::Core
{
	//The ID that is bumped by every object when it needs a new ID
	static u32 globalID{};

	u32 KalaPhysicsCore::GetGlobalID() { return globalID; }
	void KalaPhysicsCore::SetGlobalID(u32 newID) { globalID = newID; }

	void KalaPhysicsCore::ForceClose(
		const string& target,
		const string& reason)
	{
		Log::Print(
			"\n================"
			"\nFORCE CLOSE"
			"\n================\n",
			true);

		Log::Print(
			reason,
			target,
			LogType::LOG_ERROR,
			2,
			true,
			TimeFormat::TIME_NONE,
			DateFormat::DATE_NONE);

#ifdef _WIN32
		__debugbreak();
#else
		raise(SIGTRAP);
#endif

		_Exit(1);
	}
}