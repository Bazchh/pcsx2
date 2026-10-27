// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

// IAEmPS2 memory watch (phase 2, card S8-01).
//
// A write/read watch over one address range, built on the debugger's memchecks: the recompiler already calls
// dynarecMemcheck() for every load/store that touches a registered range. Instead of pausing the VM there, the
// watch records the hit in a fixed table keyed by the program counter of the access (opcode, last address, last
// value, hit count, and a snapshot of the 32 general registers at the first hit) and lets the game continue.
// The PINE thread reads the table; the CPU thread fills it. Nothing allocates inside the hook.

#pragma once

#include "common/Pcsx2Defs.h"

#include <vector>

namespace IAEmWatch
{
	struct Entry
	{
		u32 pc;         // address of the load/store instruction (delay slot resolved)
		u32 op;         // the instruction word
		u32 first_addr; // effective address at the first hit
		u32 last_addr;  // effective address at the last hit
		u32 count;      // hits recorded for this pc
		u8 size_bits;   // 8, 16, 32, 64 or 128
		u8 is_store;    // 1 = store, 0 = load
		u16 pad;
		u64 last_value; // rt at the last hit: the value stored (stores) or the previous rt (loads)
		u32 gpr_lo[32]; // low 32 bits of the general registers at the first hit
	};
	static_assert(sizeof(Entry) == 160);

	struct Stats
	{
		u32 active;      // 1 while a watch is registered
		u32 start;       // watched range
		u32 end;
		u32 total_hits;  // hits seen since the watch was set (including the ones that no longer fit the table)
		u32 dropped;     // hits that found the table full
		u32 entries;     // distinct pcs recorded
		u32 ms_left;     // ms until the watch removes itself; 0 = no deadline
	};

	/** Registers the watch (replacing any previous one) and the memcheck behind it. CPU thread only. */
	void Set(u32 start, u32 end, u8 cond, u32 duration_ms);
	/** Removes the watch and the memcheck. CPU thread only. */
	void Clear();
	/** Copies the table (and optionally resets it) so the caller can compare windows. Any thread. */
	Stats Read(std::vector<Entry>* out, bool reset);
	/** Removes the watch on the CPU thread once its deadline has passed. Call from the PINE thread. */
	void CheckDeadline();

	/**
	 * Hook for dynarecMemcheck(): records the hit when the memcheck at that index belongs to the watch.
	 * Returns true when the recompiler should continue without pausing the VM. CPU thread only.
	 */
	bool OnMemCheckHit(size_t memcheck_index);
} // namespace IAEmWatch
