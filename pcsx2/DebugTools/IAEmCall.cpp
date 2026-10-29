// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "DebugTools/IAEmCall.h"
#include "Memory.h"
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

	bool Start(u32 func, const u32* args, u32 argc)
	{
		std::lock_guard<std::mutex> guard(s_lock);
		if (s_status.load(std::memory_order_acquire) == STATUS_RUNNING)
			return false;
		if (argc > MAX_ARGS)
			return false;

		// `b .` (beq zero, zero, -1) and its delay slot. Written every time: cheap, and the page is ours.
		vtlb_ramWrite<mem32_t>(TRAMPOLINE, 0x1000FFFFu);
		vtlb_ramWrite<mem32_t>(TRAMPOLINE + 4, 0u);

		SaveRegisters();
		for (u32 i = 0; i < argc; i++)
			SetReg(4 + i, args[i]); // a0..a3
		SetReg(29, STACK_TOP - 16); // sp, with the 16 bytes the ABI reserves for the callee
		SetReg(31, TRAMPOLINE); // ra
		cpuRegs.pc = func;

		s_v0 = 0;
		s_v1 = 0;
		s_elapsed_ms = 0;
		s_start_ms = NowMs();
		s_status.store(STATUS_RUNNING, std::memory_order_release);
		return true;
	}

	bool Poll()
	{
		std::lock_guard<std::mutex> guard(s_lock);
		if (s_status.load(std::memory_order_acquire) != STATUS_RUNNING)
			return true;
		if (cpuRegs.pc != TRAMPOLINE && cpuRegs.pc != TRAMPOLINE + 4)
			return false;
		s_v0 = cpuRegs.GPR.n.v0.UL[0];
		s_v1 = cpuRegs.GPR.n.v1.UL[0];
		s_elapsed_ms = static_cast<u32>(NowMs() - s_start_ms);
		RestoreRegisters();
		s_status.store(STATUS_DONE, std::memory_order_release);
		return true;
	}

	void Abort()
	{
		std::lock_guard<std::mutex> guard(s_lock);
		if (s_status.load(std::memory_order_acquire) != STATUS_RUNNING)
			return;
		s_elapsed_ms = static_cast<u32>(NowMs() - s_start_ms);
		RestoreRegisters();
		s_status.store(STATUS_ABORTED, std::memory_order_release);
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
