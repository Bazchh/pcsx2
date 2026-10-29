// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

// IAEmPS2 remote call (phase 2, card S8-02).
//
// Runs one function of the game on the EE with up to four integer arguments, the way a cheat device hooks a
// call: at a vsync (where the recompiler has flushed every register and the pc is a block boundary) the EE
// registers are saved, a0..a3, sp, ra and pc are rewritten, and the game resumes inside the function with a
// private stack. ra points to a trampoline (`b .; nop`) written in a reserved page; when the function returns
// the EE spins there until the next poll sees pc at the trampoline, reads v0/v1 and restores every register,
// so the game continues from the vsync as if nothing happened. The PINE thread drives it with blocking
// Host::RunOnCPUThread calls; nothing here runs on the PINE thread itself.
//
// Reserved RAM (never seen in use by the game of the proof, zero in every snapshot): the trampoline page at
// 0x01FF0000, the private stack below 0x01FF8000, and 0x01FF8000..0x01FFC000 for the caller's own data.

#pragma once

#include "common/Pcsx2Defs.h"

namespace IAEmCall
{
	static constexpr u32 TRAMPOLINE = 0x01FF0000;
	static constexpr u32 STACK_TOP = 0x01FF8000;
	static constexpr u32 CALLER_DATA = 0x01FF8000;
	static constexpr u32 MAX_ARGS = 4;

	enum Status : u8
	{
		STATUS_IDLE = 0,
		STATUS_RUNNING = 1,
		STATUS_DONE = 2,
		STATUS_ABORTED = 3,
	};

	struct Result
	{
		u8 status;
		u32 v0;
		u32 v1;
		u32 elapsed_ms;
	};

	/** Saves the registers and diverts the EE into the function. CPU thread only. False while a call is running. */
	bool Start(u32 func, const u32* args, u32 argc);
	/** True when the EE reached the trampoline: v0/v1 captured, registers restored. CPU thread only. */
	bool Poll();
	/** Restores the registers without waiting for the function (deadline passed). CPU thread only. */
	void Abort();
	/** Last call's outcome. Any thread. */
	Result Last();
} // namespace IAEmCall
