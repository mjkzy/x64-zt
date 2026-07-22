#include "std_include.hpp"
#include "zonetool/utils/utils.hpp"
#include "havok.hpp"
#include "havok_debug.hpp"

#include <cmath>
#include <cstdarg>
#include <map>
#include <set>
#include <unordered_map>

// Human-readable dumper for the IW7 world-collision havok packfile
// (hk_2014.2.5-r1, fileVersion 11, MSVC x64 layout). Ports the validated
// python research tooling (research/iw7-collision: parse_hkx.py,
// mesh_decode.py, check_tree_aabbs.py) into zonetool so blob problems can be
// diagnosed straight from a dump/build run.
//
// All structure offsets below were verified against iw7_ship.exe decode
// paths (see research/iw7-collision notes):
//   shape+96            -> global fixup to hknpCompressedMeshShapeData
//   data+16             -> hkcdStaticMeshTree (inline)
//   tree+0/64/80/96/112/128/144 -> nodes/sections/primitives/sharedVerticesIndex/
//                                  packedVertices/sharedVertices/primitiveDataRuns
//   tree+16 / tree+32   -> domain min/max (float4)
//   data+176            -> hkcdSimdTree, nodes hkArray @ +8 (fixup data+0xB8)
//   section stride 96   -> +16 min, +32 max, +48 codecParms[6], +72 firstPacked,
//                          +76 svHandle, +80 primHandle, +84 drHandle,
//                          +88 numPacked, +89 numShared, +92 sharedDescWindow
namespace zonetool::iw7
{
	namespace havok
	{
		namespace debug
		{
			namespace
			{
				constexpr float K_NODE = 0.004424778761061947f; // 1/226
				constexpr float R21 = 1.0f / ((1 << 21) - 1);
				constexpr float R22 = 1.0f / ((1 << 22) - 1);

				std::string fmt_str(const char* fmt, ...)
				{
					char buf[2048];

					va_list ap;
					va_start(ap, fmt);
					const auto len = vsnprintf(buf, sizeof(buf), fmt, ap);
					va_end(ap);

					if (len < 0)
					{
						return {};
					}

					if (static_cast<std::size_t>(len) < sizeof(buf))
					{
						return std::string(buf, static_cast<std::size_t>(len));
					}

					std::string big(static_cast<std::size_t>(len), '\0');
					va_start(ap, fmt);
					vsnprintf(big.data(), big.size() + 1, fmt, ap);
					va_end(ap);
					return big;
				}

				struct local_fixup { int from; int to; };
				struct global_fixup { int from; int section; int to; };
				struct virtual_fixup { int obj; int name_section; int name_off; };

				struct pf_section
				{
					std::string tag;
					int abs_start{};
					int local_off{};
					int global_off{};
					int virtual_off{};
					int exports_off{};
					int imports_off{};
					int end_off{};
					std::vector<local_fixup> locals;
					std::vector<global_fixup> globals;
					std::vector<virtual_fixup> virtuals;
				};

				struct packfile_view
				{
					const std::uint8_t* buf{};
					std::size_t size{};
					int file_version{};
					int num_sections{};
					int contents_section_index{};
					int contents_section_offset{};
					int contents_classname_section_index{};
					std::string contents_version;
					std::vector<pf_section> sections;
					std::map<int, std::pair<std::string, std::uint32_t>> classnames; // name-offset -> (name, sig)

					// data-section accessors
					const std::uint8_t* d{};
					std::size_t d_size{};
					std::unordered_map<int, int> locmap;
					std::unordered_map<int, std::pair<int, int>> globmap;
					std::unordered_map<int, std::string> virtmap; // obj off -> classname

					void require(std::size_t off, std::size_t count) const
					{
						if (off + count > this->size)
						{
							throw std::runtime_error(fmt_str(
								"havok_debug: read out of bounds (off=%zu count=%zu size=%zu)",
								off, count, this->size));
						}
					}

					template <typename T> T get(std::size_t off) const
					{
						this->require(off, sizeof(T));
						T v;
						std::memcpy(&v, this->buf + off, sizeof(T));
						return v;
					}

					void require_data(std::size_t off, std::size_t count) const
					{
						if (off + count > this->d_size)
						{
							throw std::runtime_error(fmt_str(
								"havok_debug: data-section read out of bounds (off=%zu count=%zu size=%zu)",
								off, count, this->d_size));
						}
					}

					template <typename T> T data(std::size_t off) const
					{
						this->require_data(off, sizeof(T));
						T v;
						std::memcpy(&v, this->d + off, sizeof(T));
						return v;
					}

					// hkArray at data offset -> (data offset of elements or -1, count)
					std::pair<int, unsigned int> array(int off) const
					{
						const auto count = this->data<std::uint32_t>(off + 8);
						const auto it = this->locmap.find(off);
						return { it == this->locmap.end() ? -1 : it->second, count };
					}
				};

				packfile_view parse_packfile(const char* bytes, unsigned int size)
				{
					packfile_view pf{};
					pf.buf = reinterpret_cast<const std::uint8_t*>(bytes);
					pf.size = size;

					if (pf.get<std::uint32_t>(0) != 0x57E0E057 || pf.get<std::uint32_t>(4) != 0x10C0C010)
					{
						throw std::runtime_error("havok_debug: bad packfile magic");
					}

					pf.file_version = pf.get<std::int32_t>(12);
					pf.num_sections = pf.get<std::int32_t>(0x14);
					pf.contents_section_index = pf.get<std::int32_t>(0x18);
					pf.contents_section_offset = pf.get<std::int32_t>(0x1C);
					pf.contents_classname_section_index = pf.get<std::int32_t>(0x20);

					char version[17]{};
					pf.require(0x28, 16);
					std::memcpy(version, pf.buf + 0x28, 16);
					pf.contents_version = version;

					const auto predicate_array_size = pf.get<std::uint16_t>(0x3E);
					const auto stride = pf.file_version == 11 ? 64 : 48;
					const auto base = 0x40 + predicate_array_size;

					for (auto i = 0; i < pf.num_sections; i++)
					{
						const auto off = base + i * stride;
						pf_section s{};

						char tag[20]{};
						pf.require(off, 19);
						std::memcpy(tag, pf.buf + off, 19);
						s.tag = tag;

						s.abs_start = pf.get<std::int32_t>(off + 0x14);
						s.local_off = pf.get<std::int32_t>(off + 0x18);
						s.global_off = pf.get<std::int32_t>(off + 0x1C);
						s.virtual_off = pf.get<std::int32_t>(off + 0x20);
						s.exports_off = pf.get<std::int32_t>(off + 0x24);
						s.imports_off = pf.get<std::int32_t>(off + 0x28);
						s.end_off = pf.get<std::int32_t>(off + 0x2C);

						for (auto p = s.abs_start + s.local_off; p + 8 <= s.abs_start + s.global_off; p += 8)
						{
							const auto from = pf.get<std::int32_t>(p);
							const auto to = pf.get<std::int32_t>(p + 4);
							if (from != -1)
							{
								s.locals.push_back({ from, to });
							}
						}
						for (auto p = s.abs_start + s.global_off; p + 12 <= s.abs_start + s.virtual_off; p += 12)
						{
							const auto from = pf.get<std::int32_t>(p);
							const auto sec = pf.get<std::int32_t>(p + 4);
							const auto to = pf.get<std::int32_t>(p + 8);
							if (from != -1)
							{
								s.globals.push_back({ from, sec, to });
							}
						}
						for (auto p = s.abs_start + s.virtual_off; p + 12 <= s.abs_start + s.exports_off; p += 12)
						{
							const auto obj = pf.get<std::int32_t>(p);
							const auto cns = pf.get<std::int32_t>(p + 4);
							const auto cno = pf.get<std::int32_t>(p + 8);
							if (obj != -1)
							{
								s.virtuals.push_back({ obj, cns, cno });
							}
						}

						pf.sections.push_back(std::move(s));
					}

					// classnames: u32 signature, u8 0x09, zero-terminated name
					{
						const auto& cn = pf.sections.at(pf.contents_classname_section_index);
						const auto* cd = pf.buf + cn.abs_start;
						const auto cd_size = static_cast<std::size_t>(cn.end_off);
						std::size_t p = 0;
						while (p + 5 <= cd_size)
						{
							std::uint32_t sig;
							std::memcpy(&sig, cd + p, 4);
							if (cd[p + 4] != 0x09)
							{
								break;
							}
							const auto nstart = p + 5;
							auto nend = nstart;
							while (nend < cd_size && cd[nend] != 0)
							{
								nend++;
							}
							pf.classnames[static_cast<int>(nstart)] = {
								std::string(reinterpret_cast<const char*>(cd + nstart), nend - nstart), sig };
							p = nend + 1;
						}
					}

					// data-section shortcuts
					const auto& ds = pf.sections.at(pf.contents_section_index);
					pf.d = pf.buf + ds.abs_start;
					pf.d_size = static_cast<std::size_t>(ds.end_off);
					for (const auto& f : ds.locals)
					{
						pf.locmap[f.from] = f.to;
					}
					for (const auto& f : ds.globals)
					{
						pf.globmap[f.from] = { f.section, f.to };
					}
					for (const auto& f : ds.virtuals)
					{
						const auto it = pf.classnames.find(f.name_off);
						if (it != pf.classnames.end())
						{
							pf.virtmap[f.obj] = it->second.first;
						}
					}

					return pf;
				}

				// ------------------------------------------------------------------
				// compressed mesh decode (validated against iw7_ship.exe)
				// ------------------------------------------------------------------
				struct vec3 { float x, y, z; };

				struct decoded_triangle
				{
					vec3 v[3];
					std::uint16_t shape_tag;
					int section;
					int prim;
				};

				struct section_info
				{
					float min[3];
					float max[3];
					int prim_count;      // raw primitive slots (incl. quads/dead)
					int tri_count;       // decoded triangles
					int num_packed;
					int num_shared;
					int node_count;
					int datarun_count;
				};

				struct decoded_mesh
				{
					int shape_off{ -1 };
					int data_off{ -1 };
					float dmin[3]{};
					float dmax[3]{};
					unsigned int top_node_count{};
					unsigned int section_count{};
					unsigned int primitive_count{};
					unsigned int shared_vertices_index_count{};
					unsigned int packed_vertex_count{};
					unsigned int shared_vertex_count{};
					unsigned int datarun_count{};
					int simd_node_off{ -1 };
					unsigned int simd_node_count{};
					std::vector<section_info> sections;
					std::vector<decoded_triangle> triangles;
				};

				bool find_mesh_shape(const packfile_view& pf, int* shape_off, int* data_off)
				{
					for (const auto& [obj, name] : pf.virtmap)
					{
						if (name == "hknpCompressedMeshShape")
						{
							*shape_off = obj;
							const auto it = pf.globmap.find(obj + 96);
							if (it == pf.globmap.end())
							{
								return false;
							}
							*data_off = it->second.second;
							return true;
						}
					}
					return false;
				}

				decoded_mesh decode_mesh(const packfile_view& pf, bool want_triangles)
				{
					decoded_mesh m{};
					if (!find_mesh_shape(pf, &m.shape_off, &m.data_off))
					{
						return m;
					}

					const auto tree = m.data_off + 16;
					for (auto a = 0; a < 3; a++)
					{
						m.dmin[a] = pf.data<float>(tree + 16 + 4 * a);
						m.dmax[a] = pf.data<float>(tree + 32 + 4 * a);
					}

					const auto [top_off, top_cnt] = pf.array(tree + 0);
					const auto [sec_off, sec_cnt] = pf.array(tree + 64);
					const auto [prim_off, prim_cnt] = pf.array(tree + 80);
					const auto [svi_off, svi_cnt] = pf.array(tree + 96);
					const auto [pv_off, pv_cnt] = pf.array(tree + 112);
					const auto [sv_off, sv_cnt] = pf.array(tree + 128);
					const auto [dr_off, dr_cnt] = pf.array(tree + 144);

					m.top_node_count = top_cnt;
					m.section_count = sec_cnt;
					m.primitive_count = prim_cnt;
					m.shared_vertices_index_count = svi_cnt;
					m.packed_vertex_count = pv_cnt;
					m.shared_vertex_count = sv_cnt;
					m.datarun_count = dr_cnt;

					// simd tree: hkcdSimdTree @ data+176, nodes hkArray @ +8
					{
						const auto it = pf.locmap.find(m.data_off + 176 + 8);
						m.simd_node_off = it == pf.locmap.end() ? -1 : it->second;
						m.simd_node_count = pf.data<std::uint32_t>(m.data_off + 176 + 16);
					}

					const float ext[3] = {
						m.dmax[0] - m.dmin[0], m.dmax[1] - m.dmin[1], m.dmax[2] - m.dmin[2] };

					const auto dec_shared = [&](unsigned int desc_index) -> vec3
					{
						const auto D = pf.data<std::uint64_t>(sv_off + 8 * desc_index);
						const auto qx = static_cast<float>(D & 0x1FFFFF);
						const auto qy = static_cast<float>((D >> 21) & 0x1FFFFF);
						const auto qz = static_cast<float>((D >> 42) & 0x3FFFFF);
						return {
							m.dmin[0] + qx * R21 * ext[0],
							m.dmin[1] + qy * R21 * ext[1],
							m.dmin[2] + qz * R22 * ext[2] };
					};

					for (auto si = 0u; si < sec_cnt; si++)
					{
						const auto so = sec_off + 96 * si;
						section_info info{};
						for (auto a = 0; a < 3; a++)
						{
							info.min[a] = pf.data<float>(so + 16 + 4 * a);
							info.max[a] = pf.data<float>(so + 32 + 4 * a);
						}

						float codec[6];
						for (auto i = 0; i < 6; i++)
						{
							codec[i] = pf.data<float>(so + 48 + 4 * i);
						}
						const auto first_packed = pf.data<std::uint32_t>(so + 72);
						const auto sv_handle = pf.data<std::uint32_t>(so + 76);
						const auto prim_handle = pf.data<std::uint32_t>(so + 80);
						const auto dr_handle = pf.data<std::uint32_t>(so + 84);
						const auto num_packed = pf.data<std::uint8_t>(so + 88);
						const auto num_shared = pf.data<std::uint8_t>(so + 89);
						const auto sv_window = pf.data<std::uint32_t>(so + 92);

						info.num_packed = num_packed;
						info.num_shared = num_shared;
						info.node_count = pf.data<std::uint32_t>(so + 8);
						info.prim_count = prim_handle & 0xFF;
						info.datarun_count = dr_handle & 0xFF;

						if (want_triangles)
						{
							const auto prim_base = static_cast<int>(prim_handle >> 8);
							const auto sv_base_idx = static_cast<int>(sv_handle >> 8);
							const auto shared_window = sv_window * 65536u;

							const auto local_vert = [&](std::uint8_t idx) -> vec3
							{
								if (idx < num_packed)
								{
									const auto P = pf.data<std::uint32_t>(pv_off + 4 * (first_packed + idx));
									return {
										codec[0] + static_cast<float>(P & 0x7FF) * codec[3],
										codec[1] + static_cast<float>((P >> 11) & 0x7FF) * codec[4],
										codec[2] + static_cast<float>((P >> 22) & 0x3FF) * codec[5] };
								}
								const auto li = idx - num_packed;
								const auto desc = pf.data<std::uint16_t>(svi_off + 2 * (sv_base_idx + li));
								return dec_shared(shared_window + desc);
							};

							const auto tag_for = [&](int p) -> std::uint16_t
							{
								const auto dr_base = static_cast<int>(dr_handle >> 8);
								for (auto r = 0; r < info.datarun_count; r++)
								{
									const auto value = pf.data<std::uint16_t>(dr_off + 4 * (dr_base + r));
									const auto index = pf.data<std::uint8_t>(dr_off + 4 * (dr_base + r) + 2);
									const auto count = pf.data<std::uint8_t>(dr_off + 4 * (dr_base + r) + 3);
									if (index <= p && p < index + count)
									{
										return value;
									}
								}
								return 0;
							};

							for (auto p = 0; p < info.prim_count; p++)
							{
								const auto a = pf.data<std::uint8_t>(prim_off + 4 * (prim_base + p));
								const auto b = pf.data<std::uint8_t>(prim_off + 4 * (prim_base + p) + 1);
								const auto c = pf.data<std::uint8_t>(prim_off + 4 * (prim_base + p) + 2);
								const auto dd = pf.data<std::uint8_t>(prim_off + 4 * (prim_base + p) + 3);
								if (a == 0xDE && b == 0xAD && c == 0 && dd == 0)
								{
									continue; // dead padding primitive
								}
								const auto tag = tag_for(p);
								const auto va = local_vert(a);
								const auto vb = local_vert(b);
								const auto vc = local_vert(c);
								m.triangles.push_back({ { va, vb, vc }, tag, static_cast<int>(si), p });
								info.tri_count++;
								if (dd != c)
								{
									const auto vd = local_vert(dd);
									m.triangles.push_back({ { va, vc, vd }, tag, static_cast<int>(si), p });
									info.tri_count++;
								}
							}
						}

						m.sections.push_back(info);
					}

					return m;
				}

				// ------------------------------------------------------------------
				// tree validation (engine-exact dequant; port of check_tree_aabbs.py)
				// ------------------------------------------------------------------
				struct boxf { float min[3]; float max[3]; };

				boxf refine(const boxf& parent, std::uint8_t b0, std::uint8_t b1, std::uint8_t b2)
				{
					// child.min = parent.min + hiNib^2 * ext / 226
					// child.max = parent.max - loNib^2 * ext / 226
					const std::uint8_t bytes[3] = { b0, b1, b2 };
					boxf out = parent;
					for (auto a = 0; a < 3; a++)
					{
						const auto ext = parent.max[a] - parent.min[a];
						const auto hi = static_cast<float>(bytes[a] >> 4);
						const auto lo = static_cast<float>(bytes[a] & 0xF);
						out.min[a] = parent.min[a] + hi * hi * ext * K_NODE;
						out.max[a] = parent.max[a] - lo * lo * ext * K_NODE;
					}
					return out;
				}

				bool contains(const boxf& outer, const boxf& inner, float eps, float* worst)
				{
					auto w = 0.0f;
					for (auto a = 0; a < 3; a++)
					{
						w = std::max(w, outer.min[a] - inner.min[a]);
						w = std::max(w, inner.max[a] - outer.max[a]);
					}
					if (worst && w > *worst)
					{
						*worst = w;
					}
					return w <= eps;
				}

				struct validation_result
				{
					int bad_section_aabbs{};      // A: stored section AABB vs decoded content
					int bad_top_leaf_boxes{};     // B: refined top-tree leaf box vs section AABB
					int bad_prim_leaf_boxes{};    // C: refined section-tree leaf box vs prim AABB
					int top_sections_seen{};
					int prim_leaves_checked{};
					float worst_a{};
					float worst_b{};
					float worst_c{};
					std::string error;
				};

				validation_result validate_trees(const packfile_view& pf, const decoded_mesh& m)
				{
					validation_result r{};
					constexpr auto EPS = 1e-3f;
					if (m.data_off < 0 || m.section_count == 0)
					{
						return r;
					}

					const auto tree = m.data_off + 16;
					const auto [top_off, top_cnt] = pf.array(tree + 0);
					const auto [sec_off, sec_cnt] = pf.array(tree + 64);

					// per-section triangle point AABBs / per-prim AABBs
					std::vector<boxf> sec_content(sec_cnt);
					std::vector<std::map<int, boxf>> prim_aabbs(sec_cnt);
					std::vector<bool> sec_has_content(sec_cnt, false);
					for (const auto& t : m.triangles)
					{
						auto& box = sec_content[t.section];
						auto& pmapped = prim_aabbs[t.section];
						boxf tb{};
						for (auto a = 0; a < 3; a++)
						{
							const float vals[3] = {
								(&t.v[0].x)[a], (&t.v[1].x)[a], (&t.v[2].x)[a] };
							tb.min[a] = std::min({ vals[0], vals[1], vals[2] });
							tb.max[a] = std::max({ vals[0], vals[1], vals[2] });
						}
						if (!sec_has_content[t.section])
						{
							box = tb;
							sec_has_content[t.section] = true;
						}
						else
						{
							for (auto a = 0; a < 3; a++)
							{
								box.min[a] = std::min(box.min[a], tb.min[a]);
								box.max[a] = std::max(box.max[a], tb.max[a]);
							}
						}
						const auto it = pmapped.find(t.prim);
						if (it == pmapped.end())
						{
							pmapped[t.prim] = tb;
						}
						else
						{
							for (auto a = 0; a < 3; a++)
							{
								it->second.min[a] = std::min(it->second.min[a], tb.min[a]);
								it->second.max[a] = std::max(it->second.max[a], tb.max[a]);
							}
						}
					}

					const auto stored_box = [&](unsigned int si) -> boxf
					{
						boxf b{};
						const auto so = sec_off + 96 * si;
						for (auto a = 0; a < 3; a++)
						{
							b.min[a] = pf.data<float>(so + 16 + 4 * a);
							b.max[a] = pf.data<float>(so + 32 + 4 * a);
						}
						return b;
					};

					// A
					for (auto si = 0u; si < sec_cnt; si++)
					{
						if (!sec_has_content[si])
						{
							continue;
						}
						const auto sb = stored_box(si);
						if (!contains(sb, sec_content[si], EPS, &r.worst_a))
						{
							r.bad_section_aabbs++;
						}
					}

					// B: walk top tree (5-byte Codec3Axis5 nodes).
					// nav bit7 SET = internal (delta = ((nav&0x7F)<<8)|b4, right = cur+2*delta),
					// bit7 CLEAR = leaf (section = (nav<<8)|b4). Root's own AABB bytes are
					// never applied (root box = stored domain).
					if (top_off >= 0 && top_cnt > 0)
					{
						boxf domain{};
						for (auto a = 0; a < 3; a++)
						{
							domain.min[a] = m.dmin[a];
							domain.max[a] = m.dmax[a];
						}
						struct entry { unsigned int ni; boxf box; bool is_root; };
						std::vector<entry> stack;
						stack.push_back({ 0, domain, true });
						std::set<unsigned int> visited;
						while (!stack.empty())
						{
							const auto e = stack.back();
							stack.pop_back();
							if (e.ni >= top_cnt || visited.count(e.ni))
							{
								r.error = fmt_str("top-tree walk out of bounds/loop at node %u", e.ni);
								break;
							}
							visited.insert(e.ni);
							const auto no = top_off + e.ni * 5;
							const auto b0 = pf.data<std::uint8_t>(no);
							const auto b1 = pf.data<std::uint8_t>(no + 1);
							const auto b2 = pf.data<std::uint8_t>(no + 2);
							const auto nav = pf.data<std::uint8_t>(no + 3);
							const auto b4 = pf.data<std::uint8_t>(no + 4);
							const auto box = e.is_root ? e.box : refine(e.box, b0, b1, b2);
							if (nav & 0x80)
							{
								const auto delta = ((nav & 0x7F) << 8) | b4;
								stack.push_back({ e.ni + 1, box, false });
								stack.push_back({ e.ni + 2 * delta, box, false });
							}
							else
							{
								const auto si = static_cast<unsigned int>((nav << 8) | b4);
								r.top_sections_seen++;
								if (si < sec_cnt && !contains(box, stored_box(si), EPS, &r.worst_b))
								{
									r.bad_top_leaf_boxes++;
								}
							}
						}
					}

					// C: walk each section tree (4-byte Codec3Axis4 nodes).
					// UNLIKE the 5/6-byte codecs this one is bit0-keyed:
					//   nav bit0 SET   = internal, right = cur + (nav & 0xFE)
					//   nav bit0 CLEAR = leaf, primitive index = nav >> 1
					// Confirmed two ways: sub_14102F280 does
					// "movzx eax,[r8+3]; test al,1; jz leaf; and eax,0FEh; add eax,r9d",
					// and the genuine retail mp_paris blob (13066464 bytes) walks
					// 400/400 sections exact under bit0 and 1/400 under bit7.
					// (Beware: research/iw7-collision/test_blobs/*.hkx are OUR OWN
					// re-encoded output, not retail - do not validate against them.)
					// Root box = stored section AABB; root's own AABB bytes never applied.
					for (auto si = 0u; si < sec_cnt; si++)
					{
						const auto so = sec_off + 96 * si;
						const auto it = pf.locmap.find(static_cast<int>(so));
						const auto node_cnt = pf.data<std::uint32_t>(so + 8);
						if (it == pf.locmap.end() || node_cnt == 0)
						{
							continue;
						}
						const auto nodes_off = it->second;
						struct entry { unsigned int ni; boxf box; bool is_root; };
						std::vector<entry> stack;
						stack.push_back({ 0, stored_box(si), true });
						while (!stack.empty())
						{
							const auto e = stack.back();
							stack.pop_back();
							if (e.ni >= node_cnt)
							{
								r.error = fmt_str("section %u tree walk out of bounds at node %u", si, e.ni);
								break;
							}
							const auto no = nodes_off + e.ni * 4;
							const auto b0 = pf.data<std::uint8_t>(no);
							const auto b1 = pf.data<std::uint8_t>(no + 1);
							const auto b2 = pf.data<std::uint8_t>(no + 2);
							const auto nav = pf.data<std::uint8_t>(no + 3);
							const auto box = e.is_root ? e.box : refine(e.box, b0, b1, b2);
							if (nav & 1)
							{
								stack.push_back({ e.ni + 1, box, false });
								stack.push_back({ e.ni + static_cast<unsigned int>(nav & 0xFE), box, false });
							}
							else
							{
								const auto p = nav >> 1;
								const auto pit = prim_aabbs[si].find(p);
								if (pit != prim_aabbs[si].end())
								{
									r.prim_leaves_checked++;
									if (!contains(box, pit->second, EPS, &r.worst_c))
									{
										r.bad_prim_leaf_boxes++;
									}
								}
							}
						}
					}

					return r;
				}

				// ------------------------------------------------------------------
				// report writers
				// ------------------------------------------------------------------
				void write_text_report(const std::string& out_path, const packfile_view& pf,
					const decoded_mesh& m, const validation_result& v, unsigned int blob_size)
				{
					std::string out;
					out.reserve(256 * 1024);

					out += fmt_str("havok packfile report (%u bytes)\n", blob_size);
					out += fmt_str("fileVersion=%d contentsVersion=%s contentsSection=%d rootOffset=%d\n\n",
						pf.file_version, pf.contents_version.data(), pf.contents_section_index, pf.contents_section_offset);

					out += "sections:\n";
					for (const auto& s : pf.sections)
					{
						out += fmt_str("  %-15s abs=%-8d end=%-8d localFixups=%zu globalFixups=%zu virtualFixups=%zu\n",
							s.tag.data(), s.abs_start, s.end_off, s.locals.size(), s.globals.size(), s.virtuals.size());
					}

					out += "\nclassnames:\n";
					for (const auto& [off, entry] : pf.classnames)
					{
						out += fmt_str("  off=%-6d sig=0x%08X %s\n", off, entry.second, entry.first.data());
					}

					// virtual fixup class inventory (object counts by class)
					{
						std::map<std::string, int> inv;
						for (const auto& [obj, name] : pf.virtmap)
						{
							inv[name]++;
						}
						out += "\nobject inventory (data section):\n";
						for (const auto& [name, count] : inv)
						{
							out += fmt_str("  %5d  %s\n", count, name.data());
						}
					}

					// root shapeTagData table (HavokPhysicsShapeList + 104)
					{
						const auto root_it = pf.virtmap.find(pf.contents_section_offset);
						const auto root_class = root_it == pf.virtmap.end() ? std::string("?") : root_it->second;
						out += fmt_str("\nroot class: %s\n", root_class.data());
						if (root_class == "HavokPhysicsShapeList")
						{
							const auto [td_off, td_cnt] = pf.array(pf.contents_section_offset + 104);
							out += fmt_str("shapeTagData entries: %u\n", td_cnt);
							for (auto i = 0u; td_off >= 0 && i < td_cnt; i++)
							{
								const auto eo = td_off + i * 24;
								out += fmt_str(
									"  [%3u] collisionFilterInfo=0x%08X materialCRC=0x%08X materialId=%u userData=0x%llX\n",
									i,
									pf.data<std::uint32_t>(eo),
									pf.data<std::uint32_t>(eo + 4),
									pf.data<std::uint16_t>(eo + 8),
									pf.data<std::uint64_t>(eo + 16));
							}
						}
					}

					if (m.data_off < 0)
					{
						out += "\nno hknpCompressedMeshShape in this blob (nothing further to decode)\n";
					}
					else
					{
						out += fmt_str("\nhknpCompressedMeshShape @ %d, data @ %d\n", m.shape_off, m.data_off);
						out += fmt_str("domain min (%.2f %.2f %.2f) max (%.2f %.2f %.2f)\n",
							m.dmin[0], m.dmin[1], m.dmin[2], m.dmax[0], m.dmax[1], m.dmax[2]);
						out += fmt_str(
							"counts: topNodes=%u sections=%u primitives=%u sharedVertsIndex=%u packedVerts=%u sharedVerts=%u dataRuns=%u\n",
							m.top_node_count, m.section_count, m.primitive_count,
							m.shared_vertices_index_count, m.packed_vertex_count,
							m.shared_vertex_count, m.datarun_count);
						out += fmt_str("simdTree: nodes=%u dataOffset=%d %s\n",
							m.simd_node_count, m.simd_node_off,
							m.simd_node_off < 0 ? "(MISSING - engine query dispatchers deref this without a null check!)" : "");
						out += fmt_str("decoded triangles: %zu\n", m.triangles.size());

						out += fmt_str(
							"\ntree validation (engine-exact dequant, eps=0.001):\n"
							"  A stored-section-AABB violations : %d (worst %.3f)\n"
							"  B top-tree leaf-box violations   : %d/%d sections visited (worst %.3f)\n"
							"  C section-tree leaf-box violations: %d/%d prim leaves (worst %.3f)\n",
							v.bad_section_aabbs, v.worst_a,
							v.bad_top_leaf_boxes, v.top_sections_seen, v.worst_b,
							v.bad_prim_leaf_boxes, v.prim_leaves_checked, v.worst_c);
						if (!v.error.empty())
						{
							out += fmt_str("  WALK ERROR: %s\n", v.error.data());
						}

						// triangle centroid z histogram (64-unit bands)
						{
							std::map<int, int> hist;
							for (const auto& t : m.triangles)
							{
								const auto zc = (t.v[0].z + t.v[1].z + t.v[2].z) / 3.0f;
								hist[static_cast<int>(std::floor(zc / 64.0f)) * 64]++;
							}
							out += "\ntriangle centroid z histogram (64-unit bands):\n";
							for (const auto& [band, count] : hist)
							{
								out += fmt_str("  %8d : %d\n", band, count);
							}
						}

						out += "\nper-section table (index, stored AABB, prims/tris, packed/shared, nodes, dataruns):\n";
						for (auto si = 0u; si < m.sections.size(); si++)
						{
							const auto& s = m.sections[si];
							out += fmt_str(
								"  [%4u] min(%9.2f %9.2f %9.2f) max(%9.2f %9.2f %9.2f) prims=%-3d tris=%-3d packed=%-3d shared=%-3d nodes=%-3d runs=%d\n",
								si, s.min[0], s.min[1], s.min[2], s.max[0], s.max[1], s.max[2],
								s.prim_count, s.tri_count, s.num_packed, s.num_shared, s.node_count, s.datarun_count);
						}
					}

					auto file = filesystem::file(out_path);
					file.open("wb");
					file.write(out.data(), out.size(), 1);
					file.close();
				}

				void write_obj(const std::string& out_path, const decoded_mesh& m)
				{
					if (m.triangles.empty())
					{
						return;
					}
					std::string out;
					out.reserve(m.triangles.size() * 128);
					out += "# decoded IW7 world collision mesh\n";
					auto cur_section = -1;
					for (const auto& t : m.triangles)
					{
						if (t.section != cur_section)
						{
							cur_section = t.section;
							out += fmt_str("g section_%d\n", cur_section);
						}
						for (const auto& vv : t.v)
						{
							out += fmt_str("v %.6f %.6f %.6f\n", vv.x, vv.y, vv.z);
						}
					}
					out += "\n";
					// faces reference the flat vertex stream (1-based)
					std::size_t idx = 1;
					for (std::size_t i = 0; i < m.triangles.size(); i++, idx += 3)
					{
						out += fmt_str("f %zu %zu %zu\n", idx, idx + 1, idx + 2);
					}

					auto file = filesystem::file(out_path);
					file.open("wb");
					file.write(out.data(), out.size(), 1);
					file.close();
				}
			}

			void dump_readable(const std::string& hkx_path, const char* data, unsigned int size)
			{
				if (!data || !size)
				{
					return;
				}

				try
				{
					const auto pf = parse_packfile(data, size);
					const auto mesh = decode_mesh(pf, true);
					const auto validation = validate_trees(pf, mesh);

					write_text_report(hkx_path + ".txt", pf, mesh, validation, size);
					write_obj(hkx_path + ".obj", mesh);

					ZONETOOL_INFO("havok debug report written: %s.txt (%zu triangles, %u sections)",
						hkx_path.data(), mesh.triangles.size(), mesh.section_count);
				}
				catch (const std::exception& e)
				{
					ZONETOOL_ERROR("havok debug dump failed for \"%s\": %s", hkx_path.data(), e.what());
				}
			}

			void log_summary(const std::string& hkx_path, const char* data, unsigned int size)
			{
				if (!data || !size)
				{
					return;
				}

				// FNV-1a over the whole blob. Without this two blobs that differ only in
				// node encoding print identical summaries, and there is no way to tell
				// from the log which file the zone actually ingested.
				std::uint64_t hash = 0xCBF29CE484222325ull;
				for (auto i = 0u; i < size; i++)
				{
					hash ^= static_cast<std::uint8_t>(data[i]);
					hash *= 0x100000001B3ull;
				}

				try
				{
					const auto pf = parse_packfile(data, size);
					const auto mesh = decode_mesh(pf, false);
					const auto root_it = pf.virtmap.find(pf.contents_section_offset);
					const auto root_class = root_it == pf.virtmap.end() ? std::string("?") : root_it->second;

					if (mesh.data_off < 0)
					{
						ZONETOOL_INFO("havok blob \"%s\": %u bytes hash=%016llX root=%s, no compressed mesh shape",
							hkx_path.data(), size, hash, root_class.data());
						return;
					}

					ZONETOOL_INFO(
						"havok blob \"%s\": %u bytes hash=%016llX root=%s sections=%u prims=%u simdNodes=%u domain=(%.0f %.0f %.0f)..(%.0f %.0f %.0f)",
						hkx_path.data(), size, hash, root_class.data(), mesh.section_count, mesh.primitive_count,
						mesh.simd_node_count,
						mesh.dmin[0], mesh.dmin[1], mesh.dmin[2],
						mesh.dmax[0], mesh.dmax[1], mesh.dmax[2]);
				}
				catch (const std::exception& e)
				{
					ZONETOOL_ERROR("havok blob summary failed for \"%s\": %s", hkx_path.data(), e.what());
				}
			}
		}
	}
}
