// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

// IAEmPS2 remote call (phase 2, card S8-02).
//
// Runs one function of the game on the EE with up to four integer arguments, the way a cheat device hooks a
// call: the EE registers are saved, a0..a3, sp, ra and pc are rewritten, and the game resumes inside the
// function with a private stack. ra points to a trampoline (`b .; nop`) written in a reserved page; when the
// function returns the EE spins there until the next poll sees pc at the trampoline, reads v0/v1 and restores
// every register, so the game continues as if nothing happened.
//
// Two ways to divert:
//  - immediate: at a vsync, where the recompiler has flushed every register and the pc is a block boundary.
//    Whatever kernel thread of the game is running then is the one hijacked; good for functions that return
//    without blocking (they must not call the game's task sleep or any semaphore).
//  - anchored: at a given pc, through a debugger breakpoint whose hook diverts instead of pausing. The game
//    thread that executes that pc is the one hijacked, so a pc in the main task's frame loop lets the function
//    sleep through the game's own scheduler (loading from disc, for instance) and come back.
// The registers are restored only when the EE is seen at the trampoline: that is the hijacked thread, by
// construction. A call that never returns leaves that thread inside the function ("lost"); the caller reloads
// a save state. The PINE thread drives it with blocking Host::RunOnCPUThread calls.
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
		STATUS_ABORTED = 3, // deadline passed with the EE at the trampoline: registers restored
		STATUS_ARMED = 4,   // anchored call waiting for the anchor pc
		STATUS_LOST = 5,    // deadline passed with the EE elsewhere: the hijacked thread is still in the function
	};

	struct Result
	{
		u8 status;
		u32 v0;
		u32 v1;
		u32 elapsed_ms;
	};

	/**
	 * Saves the registers and diverts the EE into the function (no anchor), or arms the diversion for the next
	 * time the EE reaches anchor_pc (breakpoint) or touches anchor_addr (memcheck on 16 bytes, reads and writes:
	 * the thread of the game that does the access is the one hijacked). CPU thread only. False while a call is
	 * running or armed.
	 */
	bool Start(u32 func, const u32* args, u32 argc, u32 anchor_pc, u32 anchor_addr);
	/** True when the call is over (done, aborted or lost). Restores the registers when the EE is at the trampoline. CPU thread only. */
	bool Poll();
	/** Ends the call at the deadline: restores if the EE is at the trampoline, otherwise marks it lost. CPU thread only. */
	void Abort();
	/** Last call's outcome. Any thread. */
	Result Last();

	/**
	 * Hook for dynarecCheckBreakpoint(): 0 = not ours, 1 = ours and nothing to do (continue, never pause),
	 * 2 = diverted (cpuRegs.pc changed: leave the block). CPU thread only.
	 */
	int OnBreakpoint(u32 pc);
	/** Same for dynarecMemcheck(), keyed by the memcheck range. CPU thread only. */
	int OnMemCheck(u32 start, u32 end);
} // namespace IAEmCall
