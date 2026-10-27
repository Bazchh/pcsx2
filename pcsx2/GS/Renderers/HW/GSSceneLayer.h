// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

// IAEmPS2 scene layer (phase 4 spike, card S7-00).
//
// One entry point, called by the hardware renderer right before each batch goes to the device: it records a
// signature per draw (texture hash, vertex count, screen-space bounding box, depth range), applies "hide" rules,
// and injects a flat quad after the N-th depth-tested draw of the frame. Rules come from the PINE thread; the
// GS thread copies them once per frame, so per-draw work touches only thread-local data.

#pragma once

#include "common/Pcsx2Defs.h"
#include "GS/GSVector.h"

#include <vector>

struct GSHWDrawConfig;
struct GSVertex;
class GSDrawingContext;

namespace GSSceneLayer
{
	struct Signature
	{
		u64 tex_hash;
		u64 shape_hash; // hash of the quantised texture coordinates: invariant to camera and pose, identifies the mesh chunk
		u32 nverts;
		s16 x0, y0, x1, y1; // screen-space pixels, context offset already removed
		u32 z_min, z_max;
		u8 fallback;  // 1 = no hash cache entry: identity derived from the texture address instead
		u8 qualifies; // 1 = render target + depth buffer + depth test + depth write (a "world" draw)
		u16 pad;
	};
	static_assert(sizeof(Signature) == 40);

	enum RuleKind : u8
	{
		Hide = 1,
		InjectQuad = 2,
	};

	struct Rule
	{
		u32 id;
		u8 kind;
		u64 tex_hash;       // Hide: texture hash to hide
		u64 shape_hash;     // Hide: 0 = any chunk with that texture; otherwise only this chunk
		s16 x0, y0, x1, y1; // InjectQuad: screen-space rect in pixels
		u32 z;              // InjectQuad: raw GS depth value
		u32 rgba;           // InjectQuad: colour
		u32 after_nth;      // InjectQuad: inject after the N-th qualifying draw of the frame (1 = first)
		s64 deadline_ms;    // steady clock; 0 = no deadline
	};

	struct RuleStats
	{
		bool exists;
		u32 frames_total;   // frames since the rule was added
		u32 frames_seen;    // frames in which the target signature was drawn (Hide) / injection point existed (InjectQuad)
		u32 frames_applied; // frames in which the rule acted
	};

	struct LayerStats
	{
		u32 frames;             // frames observed by the layer
		u64 ns_total;           // CPU time spent in the layer, all frames
		u64 ns_last_frame;
		u32 draws_last_frame;
		u32 qualifying_last_frame;
		u32 hashed_last_frame;  // draws whose signature came from a real texture hash
	};

	// GS thread. Returns true when the draw must be skipped (hidden by a rule).
	bool OnDraw(int frame, u64 tex_hash, bool fallback, const GSVertex* verts, u32 nverts, bool fst, const GSVector4i& bbox, u32 z_min, u32 z_max, bool qualifies);
	// GS thread. Call after the batch was sent (or skipped); injects quads when a rule asks for this draw.
	void AfterDraw(const GSHWDrawConfig& conf, const GSDrawingContext* ctx, bool qualifies);

	// PINE thread.
	std::vector<Signature> LastFrame(u32* frame_no);
	u32 AddRule(Rule rule, u32 duration_ms);
	void ClearRules(u32 id); // 0 = all
	RuleStats GetRuleStats(u32 id);
	LayerStats GetLayerStats();
	s64 NowMs();
} // namespace GSSceneLayer
