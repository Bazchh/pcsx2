// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "GS/Renderers/HW/GSSceneLayer.h"
#include "GS/Renderers/Common/GSDevice.h"
#include "GS/Renderers/Common/GSVertex.h"
#include "GS/GSDrawingContext.h"
#include "GS/GSRegs.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <unordered_map>

namespace GSSceneLayer
{
	namespace
	{
		struct Counters
		{
			u32 frames_total = 0;
			u32 frames_seen = 0;
			u32 frames_applied = 0;
		};

		// Shared between threads, under s_mutex.
		std::mutex s_mutex;
		std::vector<Signature> s_last_frame;
		u32 s_last_frame_no = 0;
		std::vector<Rule> s_rules;
		std::unordered_map<u32, Counters> s_counters;
		u32 s_next_id = 1;
		LayerStats s_stats = {};

		// GS thread only.
		int s_cur_frame = -1;
		std::vector<Signature> s_cur_list;
		std::vector<Rule> s_cur_rules;
		std::vector<u8> s_cur_seen;
		std::vector<u8> s_cur_applied;
		u32 s_cur_qualifying = 0;
		u32 s_cur_hashed = 0;
		u64 s_cur_ns = 0;
		alignas(32) GSVertex s_quad[4];
		u16 s_quad_idx[6] = {0, 1, 2, 1, 3, 2};

		u64 NowNs()
		{
			return static_cast<u64>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
		}

		// Frame boundary: publish the finished frame, pull the current rule table. One lock per frame.
		void BeginFrame(int frame)
		{
			std::lock_guard<std::mutex> lock(s_mutex);
			if (s_cur_frame >= 0)
			{
				s_last_frame.swap(s_cur_list);
				s_last_frame_no = static_cast<u32>(s_cur_frame);
				s_stats.frames++;
				s_stats.ns_total += s_cur_ns;
				s_stats.ns_last_frame = s_cur_ns;
				s_stats.draws_last_frame = static_cast<u32>(s_last_frame.size());
				s_stats.qualifying_last_frame = s_cur_qualifying;
				s_stats.hashed_last_frame = s_cur_hashed;
				for (size_t i = 0; i < s_cur_rules.size(); i++)
				{
					Counters& c = s_counters[s_cur_rules[i].id];
					c.frames_total++;
					c.frames_seen += s_cur_seen[i];
					c.frames_applied += s_cur_applied[i];
				}
			}
			const s64 now = NowMs();
			s_rules.erase(std::remove_if(s_rules.begin(), s_rules.end(), [now](const Rule& r) { return r.deadline_ms != 0 && now >= r.deadline_ms; }), s_rules.end());
			s_cur_rules = s_rules;
			s_cur_seen.assign(s_cur_rules.size(), 0);
			s_cur_applied.assign(s_cur_rules.size(), 0);
			s_cur_list.clear();
			s_cur_qualifying = 0;
			s_cur_hashed = 0;
			s_cur_ns = 0;
			s_cur_frame = frame;
		}

		void DrawInjectedQuad(const GSHWDrawConfig& conf, const GSDrawingContext* ctx, const Rule& r)
		{
			// Raw GS coordinates are 12.4 fixed point and include the context offset; the bbox we report has it removed.
			const u16 ofx = static_cast<u16>(ctx->XYOFFSET.OFX);
			const u16 ofy = static_cast<u16>(ctx->XYOFFSET.OFY);
			const s16 xs[4] = {r.x0, r.x1, r.x0, r.x1};
			const s16 ys[4] = {r.y0, r.y0, r.y1, r.y1};
			for (int i = 0; i < 4; i++)
			{
				GSVertex& v = s_quad[i];
				std::memset(&v, 0, sizeof(v));
				v.XYZ.X = static_cast<u16>((static_cast<s32>(xs[i]) << 4) + ofx);
				v.XYZ.Y = static_cast<u16>((static_cast<s32>(ys[i]) << 4) + ofy);
				v.XYZ.Z = r.z;
				v.RGBAQ.R = static_cast<u8>(r.rgba & 0xFF);
				v.RGBAQ.G = static_cast<u8>((r.rgba >> 8) & 0xFF);
				v.RGBAQ.B = static_cast<u8>((r.rgba >> 16) & 0xFF);
				v.RGBAQ.A = static_cast<u8>((r.rgba >> 24) & 0xFF);
				v.RGBAQ.Q = 1.0f;
			}

			// Start from the triggering draw (targets, depth test, formats, constant buffers) and strip everything
			// that would need a texture, blending, alpha or destination-alpha emulation, or barriers.
			GSHWDrawConfig cfg = conf;
			cfg.tex = nullptr;
			cfg.pal = nullptr;
			cfg.verts = s_quad;
			cfg.indices = s_quad_idx;
			cfg.nverts = 4;
			cfg.nindices = 6;
			cfg.indices_per_prim = 3;
			cfg.topology = GSHWDrawConfig::Topology::Triangle;
			cfg.drawlist = nullptr;
			cfg.drawlist_bbox = nullptr;
			cfg.drawarea = cfg.scissor;
			cfg.line_expand = false;

			cfg.vs.key = 0;
			cfg.vs.fst = 1;
			cfg.vs.tme = 0;
			cfg.vs.iip = 0;
			cfg.vs.expand = GSHWDrawConfig::VSExpand::None;

			cfg.ps.tfx = TFX_NONE;
			cfg.ps.tcc = 0;
			cfg.ps.fst = 1;
			cfg.ps.fog = 0;
			cfg.ps.iip = 0;
			cfg.ps.date = 0;
			cfg.ps.atst = GSHWDrawConfig::PS_ATST::NONE;
			cfg.ps.afail = GSHWDrawConfig::PS_AFAIL::KEEP;
			cfg.ps.wms = 0;
			cfg.ps.wmt = 0;
			cfg.ps.adjs = 0;
			cfg.ps.adjt = 0;
			cfg.ps.ltf = 0;
			cfg.ps.shuffle = 0;
			cfg.ps.shuffle_same = 0;
			cfg.ps.real16src = 0;
			cfg.ps.process_ba = 0;
			cfg.ps.process_rg = 0;
			cfg.ps.shuffle_across = 0;
			cfg.ps.write_rg = 0;
			cfg.ps.fbmask = 0;
			cfg.ps.blend_a = 0;
			cfg.ps.blend_b = 0;
			cfg.ps.blend_c = 0;
			cfg.ps.blend_d = 0;
			cfg.ps.fixed_one_a = 0;
			cfg.ps.blend_hw = 0;
			cfg.ps.a_masked = 0;
			cfg.ps.blend_mix = 0;
			cfg.ps.round_inv = 0;
			cfg.ps.pabe = 0;
			cfg.ps.no_color = 0;
			cfg.ps.channel = 0;
			cfg.ps.dither = 0;
			cfg.ps.dither_adjust = 0;
			cfg.ps.tcoffsethack = 0;
			cfg.ps.urban_chaos_hle = 0;
			cfg.ps.tales_of_abyss_hle = 0;
			cfg.ps.tex_is_fb = 0;
			cfg.ps.automatic_lod = 0;
			cfg.ps.manual_lod = 0;
			cfg.ps.point_sampler = 0;
			cfg.ps.region_rect = 0;
			cfg.ps.scanmsk = 0;
			cfg.ps.aa1 = GSHWDrawConfig::PS_AA1::NONE;
			cfg.ps.abe = 0;

			cfg.blend = GSHWDrawConfig::BlendState();
			cfg.blend_multi_pass.enable = false;
			cfg.alpha_second_pass.enable = false;
			cfg.alpha_test = GSHWDrawConfig::AlphaTestMode::NONE;
			cfg.destination_alpha = GSHWDrawConfig::DestinationAlphaMode::Off;
			cfg.sampler = GSHWDrawConfig::SamplerSelector();
			cfg.colormask = GSHWDrawConfig::ColorMaskSelector();
			cfg.require_one_barrier = false;
			cfg.require_full_barrier = false;
			cfg.tex_hazard = GSHWDrawConfig::TEX_HAZARD_NONE;

			g_gs_device->RenderHW(cfg);
		}
	} // namespace

	s64 NowMs()
	{
		return static_cast<s64>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
	}

	bool OnDraw(int frame, u64 tex_hash, bool fallback, u32 nverts, const GSVector4i& bbox, u32 z_min, u32 z_max, bool qualifies)
	{
		const u64 t0 = NowNs();
		if (frame != s_cur_frame)
			BeginFrame(frame);

		Signature sig = {};
		sig.tex_hash = tex_hash;
		sig.nverts = nverts;
		sig.x0 = static_cast<s16>(std::clamp(bbox.x, -32768, 32767));
		sig.y0 = static_cast<s16>(std::clamp(bbox.y, -32768, 32767));
		sig.x1 = static_cast<s16>(std::clamp(bbox.z, -32768, 32767));
		sig.y1 = static_cast<s16>(std::clamp(bbox.w, -32768, 32767));
		sig.z_min = z_min;
		sig.z_max = z_max;
		sig.fallback = fallback ? 1 : 0;
		sig.qualifies = qualifies ? 1 : 0;
		s_cur_list.push_back(sig);
		if (qualifies)
			s_cur_qualifying++;
		if (!fallback)
			s_cur_hashed++;

		bool hide = false;
		for (size_t i = 0; i < s_cur_rules.size(); i++)
		{
			const Rule& r = s_cur_rules[i];
			if (r.kind == Hide && r.tex_hash == tex_hash)
			{
				s_cur_seen[i] = 1;
				s_cur_applied[i] = 1;
				hide = true;
			}
		}
		s_cur_ns += NowNs() - t0;
		return hide;
	}

	void AfterDraw(const GSHWDrawConfig& conf, const GSDrawingContext* ctx, bool qualifies)
	{
		if (!qualifies || s_cur_rules.empty())
			return;
		const u64 t0 = NowNs();
		for (size_t i = 0; i < s_cur_rules.size(); i++)
		{
			const Rule& r = s_cur_rules[i];
			if (r.kind != InjectQuad || r.after_nth != s_cur_qualifying)
				continue;
			s_cur_seen[i] = 1;
			DrawInjectedQuad(conf, ctx, r);
			s_cur_applied[i] = 1;
		}
		s_cur_ns += NowNs() - t0;
	}

	std::vector<Signature> LastFrame(u32* frame_no)
	{
		std::lock_guard<std::mutex> lock(s_mutex);
		if (frame_no)
			*frame_no = s_last_frame_no;
		return s_last_frame;
	}

	u32 AddRule(Rule rule, u32 duration_ms)
	{
		std::lock_guard<std::mutex> lock(s_mutex);
		rule.id = s_next_id++;
		rule.deadline_ms = duration_ms ? (NowMs() + static_cast<s64>(duration_ms)) : 0;
		if (rule.kind == InjectQuad && rule.after_nth == 0)
			rule.after_nth = 1;
		s_rules.push_back(rule);
		s_counters[rule.id] = Counters();
		return rule.id;
	}

	void ClearRules(u32 id)
	{
		std::lock_guard<std::mutex> lock(s_mutex);
		if (id == 0)
			s_rules.clear();
		else
			s_rules.erase(std::remove_if(s_rules.begin(), s_rules.end(), [id](const Rule& r) { return r.id == id; }), s_rules.end());
	}

	RuleStats GetRuleStats(u32 id)
	{
		std::lock_guard<std::mutex> lock(s_mutex);
		RuleStats st = {};
		const auto it = s_counters.find(id);
		if (it == s_counters.end())
			return st;
		st.exists = true;
		st.frames_total = it->second.frames_total;
		st.frames_seen = it->second.frames_seen;
		st.frames_applied = it->second.frames_applied;
		return st;
	}

	LayerStats GetLayerStats()
	{
		std::lock_guard<std::mutex> lock(s_mutex);
		return s_stats;
	}
} // namespace GSSceneLayer
