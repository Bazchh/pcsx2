// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "DebugTools/IAEmCall.h"
#include "DebugTools/Breakpoints.h"
#include "Memory.h"
#include "common/Console.h"
#include "R5900.h"
#include "VU.h"
#include "vtlb.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>

namespace IAEmCall
{
	// Only the registers the function may clobber are restored: counters, cycles and COP0 keep running.
	struct Saved
	{
		GPRregs gpr;
		GPR_reg hi;
		GPR_reg lo;
		u32 sa;
		u32 pc;
		fpuRegisters fpu;
		VECTOR vf[32];
		REG_VI vi[32];
		VECTOR acc;
		REG_VI q;
		REG_VI p;
	};

	static std::mutex s_lock;
	static Saved s_saved;
	static std::atomic<u8> s_status{STATUS_IDLE};
	static u32 s_func = 0;
	static u32 s_args[MAX_ARGS] = {};
	static u32 s_argc = 0;
	static u32 s_anchor = 0;
	static u32 s_anchor_addr = 0;
	static constexpr u32 ANCHOR_SPAN = 16;
	static u32 s_v0 = 0;
	static u32 s_v1 = 0;
	static s64 s_start_ms = 0;
	static u32 s_elapsed_ms = 0;

	static s64 NowMs()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	static void SaveRegisters()
	{
		std::memcpy(&s_saved.gpr, &cpuRegs.GPR, sizeof(s_saved.gpr));
		s_saved.hi = cpuRegs.HI;
		s_saved.lo = cpuRegs.LO;
		s_saved.sa = cpuRegs.sa;
		s_saved.pc = cpuRegs.pc;
		std::memcpy(&s_saved.fpu, &fpuRegs, sizeof(s_saved.fpu));
		std::memcpy(s_saved.vf, vuRegs[0].VF, sizeof(s_saved.vf));
		std::memcpy(s_saved.vi, vuRegs[0].VI, sizeof(s_saved.vi));
		s_saved.acc = vuRegs[0].ACC;
		s_saved.q = vuRegs[0].q;
		s_saved.p = vuRegs[0].p;
	}

	static void RestoreRegisters()
	{
		std::memcpy(&cpuRegs.GPR, &s_saved.gpr, sizeof(s_saved.gpr));
		cpuRegs.HI = s_saved.hi;
		cpuRegs.LO = s_saved.lo;
		cpuRegs.sa = s_saved.sa;
		cpuRegs.pc = s_saved.pc;
		std::memcpy(&fpuRegs, &s_saved.fpu, sizeof(s_saved.fpu));
		std::memcpy(vuRegs[0].VF, s_saved.vf, sizeof(s_saved.vf));
		std::memcpy(vuRegs[0].VI, s_saved.vi, sizeof(s_saved.vi));
		vuRegs[0].ACC = s_saved.acc;
		vuRegs[0].q = s_saved.q;
		vuRegs[0].p = s_saved.p;
	}

	static void SetReg(u32 index, u32 value)
	{
		// What a MIPS `li`/`lw` would leave: the 32-bit value sign-extended to 64, upper half of the 128 untouched.
		cpuRegs.GPR.r[index].SD[0] = static_cast<s64>(static_cast<s32>(value));
	}

	static bool AtTrampoline()
	{
		return cpuRegs.pc == TRAMPOLINE || cpuRegs.pc == TRAMPOLINE + 4;
	}

	// Diverts the EE into the function right now. Caller holds the lock and is on the CPU thread at a block boundary.
	static void Divert()
	{
		SaveRegisters();
		for (u32 i = 0; i < s_argc; i++)
			SetReg(4 + i, s_args[i]); // a0..a3
		SetReg(29, STACK_TOP - 16); // sp, with the 16 bytes the ABI reserves for the callee
		SetReg(31, TRAMPOLINE); // ra
		cpuRegs.pc = s_func;
		s_start_ms = NowMs();
		s_status.store(STATUS_RUNNING, std::memory_order_release);
	}

	static void DropAnchor()
	{
		if (s_anchor != 0)
		{
			CBreakPoints::RemoveBreakPoint(BREAKPOINT_EE, s_anchor);
			s_anchor = 0;
		}
		if (s_anchor_addr != 0)
		{
			CBreakPoints::RemoveMemCheck(BREAKPOINT_EE, s_anchor_addr, s_anchor_addr + ANCHOR_SPAN);
			s_anchor_addr = 0;
		}
	}

	bool Start(u32 func, const u32* args, u32 argc, u32 anchor_pc, u32 anchor_addr)
	{
		std::unique_lock<std::mutex> guard(s_lock);
		const u8 st = s_status.load(std::memory_order_acquire);
		if (st == STATUS_RUNNING || st == STATUS_ARMED)
			return false;
		if (argc > MAX_ARGS)
			return false;

		// `b .` (beq zero, zero, -1) and its delay slot. Written every time: cheap, and the page is ours.
		vtlb_ramWrite<mem32_t>(TRAMPOLINE, 0x1000FFFFu);
		vtlb_ramWrite<mem32_t>(TRAMPOLINE + 4, 0u);

		s_func = func;
		s_argc = argc;
		for (u32 i = 0; i < MAX_ARGS; i++)
			s_args[i] = (i < argc) ? args[i] : 0;
		s_v0 = 0;
		s_v1 = 0;
		s_elapsed_ms = 0;
		s_start_ms = NowMs();

		if (anchor_pc == 0 && anchor_addr == 0)
		{
			Divert();
			return true;
		}
		s_anchor = anchor_pc;
		s_anchor_addr = anchor_addr & ~(ANCHOR_SPAN - 1);
		s_status.store(STATUS_ARMED, std::memory_order_release);
		guard.unlock();
		// Adding the breakpoint or memcheck resets the recompiler so every block gets the check; the hooks below
		// then divert the first time the game's thread reaches it.
		if (s_anchor != 0)
			CBreakPoints::AddBreakPoint(BREAKPOINT_EE, s_anchor);
		if (s_anchor_addr != 0)
			CBreakPoints::AddMemCheck(BREAKPOINT_EE, s_anchor_addr, s_anchor_addr + ANCHOR_SPAN, MEMCHECK_READWRITE, MEMCHECK_BREAK);
		Console.WriteLn("IAEmCall: armed (pc 0x%08x, addr 0x%08x) for 0x%08x; at pc now 0x%08x", s_anchor, s_anchor_addr, func, cpuRegs.pc);
		return true;
	}

	int OnMemCheck(u32 start, u32 end)
	{
		std::lock_guard<std::mutex> guard(s_lock);
		if (s_anchor_addr == 0 || start != s_anchor_addr || end != s_anchor_addr + ANCHOR_SPAN)
			return 0;
		if (s_status.load(std::memory_order_acquire) != STATUS_ARMED)
			return 1;
		Divert();
		Console.WriteLn("IAEmCall: diverted into 0x%08x from a memory access", s_func);
		return 2;
	}

	static u32 s_hook_log = 0;

	int OnBreakpoint(u32 pc)
	{
		std::lock_guard<std::mutex> guard(s_lock);
		if (s_hook_log < 8)
		{
			s_hook_log++;
			Console.WriteLn("IAEmCall: breakpoint hook at 0x%08x (anchor 0x%08x, status %u)", pc, s_anchor, s_status.load());
		}
		if (s_anchor == 0 || pc != s_anchor)
			return 0;
		if (s_status.load(std::memory_order_acquire) != STATUS_ARMED)
			return 1;
		Divert();
		Console.WriteLn("IAEmCall: diverted into 0x%08x", s_func);
		return 2;
	}

	bool Poll()
	{
		std::lock_guard<std::mutex> guard(s_lock);
		const u8 st = s_status.load(std::memory_order_acquire);
		if (st == STATUS_ARMED)
			return false;
		if (st != STATUS_RUNNING)
			return true;
		if (!AtTrampoline())
			return false;
		s_v0 = cpuRegs.GPR.n.v0.UL[0];
		s_v1 = cpuRegs.GPR.n.v1.UL[0];
		s_elapsed_ms = static_cast<u32>(NowMs() - s_start_ms);
		RestoreRegisters();
		DropAnchor();
		s_status.store(STATUS_DONE, std::memory_order_release);
		Console.WriteLn("IAEmCall: done, v0 0x%08x after %u ms", s_v0, s_elapsed_ms);
		return true;
	}

	void Abort()
	{
		std::lock_guard<std::mutex> guard(s_lock);
		const u8 st = s_status.load(std::memory_order_acquire);
		if (st != STATUS_RUNNING && st != STATUS_ARMED)
			return;
		s_elapsed_ms = static_cast<u32>(NowMs() - s_start_ms);
		Console.WriteLn("IAEmCall: abort with status %u, pc now 0x%08x, hook calls %u", st, cpuRegs.pc, s_hook_log);
		if (st == STATUS_RUNNING && AtTrampoline())
		{
			RestoreRegisters();
			s_status.store(STATUS_ABORTED, std::memory_order_release);
		}
		else
		{
			// Restoring here would clobber whatever thread is running now; the hijacked one (if any) stays where it is.
			s_status.store(st == STATUS_ARMED ? STATUS_ABORTED : STATUS_LOST, std::memory_order_release);
		}
		DropAnchor();
	}

	Result Last()
	{
		std::lock_guard<std::mutex> guard(s_lock);
		Result r;
		r.status = s_status.load(std::memory_order_acquire);
		r.v0 = s_v0;
		r.v1 = s_v1;
		r.elapsed_ms = s_elapsed_ms;
		return r;
	}
} // namespace IAEmCall
