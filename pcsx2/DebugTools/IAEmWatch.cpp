// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "DebugTools/IAEmWatch.h"
#include "DebugTools/Breakpoints.h"
#include "Host.h"
#include "Memory.h"
#include "R5900.h"
#include "R5900OpcodeTables.h"
#include "VMManager.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>

namespace IAEmWatch
{
	// Open addressing keyed by pc. The size bounds the memory touched by the hook; a full table drops hits
	// and counts them, it never grows.
	static constexpr u32 TABLE_SIZE = 4096;

	static std::mutex s_lock;
	static Entry s_table[TABLE_SIZE];
	static u8 s_used[TABLE_SIZE];
	static u32 s_entries = 0;
	static u32 s_total_hits = 0;
	static u32 s_dropped = 0;
	static bool s_active = false;
	static u32 s_start = 0;
	static u32 s_end = 0;
	static std::atomic<s64> s_deadline_ms{0};

	static s64 NowMs()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	static void ResetTable()
	{
		std::memset(s_used, 0, sizeof(s_used));
		s_entries = 0;
		s_total_hits = 0;
		s_dropped = 0;
	}

	void Set(u32 start, u32 end, u8 cond, u32 duration_ms)
	{
		Clear();
		{
			std::lock_guard<std::mutex> guard(s_lock);
			ResetTable();
			s_active = true;
			s_start = start;
			s_end = end;
		}
		s_deadline_ms.store(duration_ms > 0 ? (NowMs() + duration_ms) : 0, std::memory_order_release);
		// MEMCHECK_BREAK is what makes the recompiler emit the call; the hook is what decides to continue.
		CBreakPoints::AddMemCheck(BREAKPOINT_EE, start, end, static_cast<MemCheckCondition>(cond & MEMCHECK_READWRITE), MEMCHECK_BREAK);
	}

	void Clear()
	{
		u32 start = 0, end = 0;
		bool active = false;
		{
			std::lock_guard<std::mutex> guard(s_lock);
			active = s_active;
			start = s_start;
			end = s_end;
			s_active = false;
		}
		s_deadline_ms.store(0, std::memory_order_release);
		if (active)
			CBreakPoints::RemoveMemCheck(BREAKPOINT_EE, start, end);
	}

	Stats Read(std::vector<Entry>* out, bool reset)
	{
		std::lock_guard<std::mutex> guard(s_lock);
		Stats st = {};
		st.active = s_active ? 1 : 0;
		st.start = s_start;
		st.end = s_end;
		st.total_hits = s_total_hits;
		st.dropped = s_dropped;
		st.entries = s_entries;
		const s64 deadline = s_deadline_ms.load(std::memory_order_acquire);
		st.ms_left = (deadline == 0) ? 0 : static_cast<u32>(std::max<s64>(deadline - NowMs(), 0));
		if (out)
		{
			out->clear();
			out->reserve(s_entries);
			for (u32 i = 0; i < TABLE_SIZE; i++)
			{
				if (s_used[i])
					out->push_back(s_table[i]);
			}
		}
		if (reset)
			ResetTable();
		return st;
	}

	void CheckDeadline()
	{
		s64 deadline = s_deadline_ms.load(std::memory_order_acquire);
		if (deadline == 0 || NowMs() < deadline)
			return;
		if (!s_deadline_ms.compare_exchange_strong(deadline, 0))
			return;
		Host::RunOnCPUThread([] {
			Clear();
		});
	}

	bool OnMemCheckHit(size_t memcheck_index)
	{
		const auto& checks = CBreakPoints::GetMemChecks(BREAKPOINT_EE);
		if (memcheck_index >= checks.size())
			return false;
		const MemCheck& mc = checks[memcheck_index];

		std::lock_guard<std::mutex> guard(s_lock);
		if (!s_active || mc.start != s_start || mc.end != s_end)
			return false;

		s_total_hits++;

		// Same resolution the debugger uses for its log: a load/store in a branch delay slot is one word past pc.
		const u32 pc = cpuRegs.pc;
		const u32 access_pc = (isMemcheckNeeded(pc) == 2) ? pc + 4 : pc;
		const u32 op = memRead32(access_pc);
		const R5900::OPCODE& opcode = R5900::GetInstruction(op);
		const u32 rs = (op >> 21) & 0x1F;
		const u32 rt = (op >> 16) & 0x1F;
		u32 addr = cpuRegs.GPR.r[rs].UL[0] + static_cast<s16>(op);
		u8 size_bits = 0;
		switch (opcode.flags & MEMTYPE_MASK)
		{
			case MEMTYPE_BYTE: size_bits = 8; break;
			case MEMTYPE_HALF: size_bits = 16; break;
			case MEMTYPE_WORD: size_bits = 32; break;
			case MEMTYPE_DWORD: size_bits = 64; break;
			case MEMTYPE_QWORD: size_bits = 128; addr &= ~0x0F; break;
			default: break;
		}
		const u8 is_store = (opcode.flags & IS_STORE) ? 1 : 0;
		// For SWC1/SQC2 the rt field names a coprocessor register; the GPR value recorded is then meaningless.
		const u64 value = cpuRegs.GPR.r[rt].UD[0];

		u32 slot = (access_pc >> 2) & (TABLE_SIZE - 1);
		for (u32 probe = 0; probe < TABLE_SIZE; probe++)
		{
			Entry& e = s_table[slot];
			if (!s_used[slot])
			{
				s_used[slot] = 1;
				s_entries++;
				e.pc = access_pc;
				e.op = op;
				e.first_addr = addr;
				e.last_addr = addr;
				e.count = 1;
				e.size_bits = size_bits;
				e.is_store = is_store;
				e.pad = 0;
				e.last_value = value;
				for (u32 r = 0; r < 32; r++)
					e.gpr_lo[r] = cpuRegs.GPR.r[r].UL[0];
				return true;
			}
			if (e.pc == access_pc)
			{
				e.last_addr = addr;
				e.last_value = value;
				e.count++;
				return true;
			}
			slot = (slot + 1) & (TABLE_SIZE - 1);
		}
		s_dropped++;
		return true;
	}
} // namespace IAEmWatch
