#include "std_include.hpp"
#include "havok_encode.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

// From-scratch encoder for the IW7 world-collision havok packfile
// (hk_2014.2.5-r1, fileVersion 11, MSVC x64 layout): triangle soup ->
// HavokPhysicsShapeList -> hknpCompressedMeshShape -> hknpCompressedMeshShapeData
// with a two-level compressed BVH.
//
// This is a straight port of the validated python research prototype
// (research/iw7-collision: write_hkx.py + mesh_encoder.py); every structure
// offset matches the decoder in havok_debug.cpp, which is the self-validator
// for the blobs produced here.
//
// Container quirks reproduced from retail (writer_findings.md /
// retail_sample_findings.md):
//   * 3 sections: __classnames__, __types__ (empty), __data__ (contents idx 2)
//   * header contentsVersion char[16]: "hk_2014.2.5-r1\0" + one 0xFF filler
//   * section headers are 0xFF filled, tag strncpy'd into 19 bytes, the
//     m_nullByte at +0x13 and the 16 trailing pad bytes stay 0xFF
//   * classnames data and every fixup table region pad to 16 with 0xFF
//   * __data__ object payload pads with 0x00
//   * maxPredicate = 0x0015, predicateArraySizePlusPadding = 0, no array
namespace zonetool::iw7
{
	namespace havok
	{
		namespace encode
		{
			namespace
			{
				// ------------------------------------------------------------------
				// 4-byte Codec3Axis4 (per-section sub-tree) nav encoding: bit0-keyed.
				// SETTLED - do not flip this back without re-reading the note below.
				//
				// true  (bit7) : internal = nav & 0x80, delta = nav & 0x7F,
				//                right = cur + 2*delta, leaf prim index = nav
				// false (bit0) : internal = nav & 0x01,
				//                right = cur + (nav & 0xFE), leaf prim index = nav >> 1
				//
				// The runtime walker sub_14102F280 does:
				//   movzx eax,[r8+3] ; test al,1 ; jz leaf ; and eax,0FEh ; add eax,r9d
				// and the leaf path shifts (shr r15d,1) before indexing primitives.
				// 17 four-byte walkers in the image use "and ,0FEh"; none use "and ,7Fh".
				//
				// Data agrees: the GENUINE retail mp_paris colmap.hkx (13066464 bytes,
				// dumped from the shipped fastfile) walks 400/400 sections exact under
				// bit0 and 1/400 under bit7. Every retail internal nav is odd, every
				// leaf nav even.
				//
				// TRAP: research/iw7-collision/test_blobs/*.hkx are OUR OWN re-encoded
				// output (mp_paris there is 4667232 bytes, not 13066464). Validating
				// against those only proves the writer agrees with itself - it is what
				// produced a bogus "retail is bit7" conclusion once already.
				//
				// The 5-byte Codec3Axis5 top tree really is bit7 (confirmed at
				// 0x1410380f3) and is hardcoded below. The two codecs differ.
				constexpr bool SECTION_NAV_BIT7 = false;

				// ---- container constants ----
				constexpr std::uint32_t MAGIC0 = 0x57E0E057;
				constexpr std::uint32_t MAGIC1 = 0x10C0C010;
				constexpr std::int32_t FILE_VERSION = 11;
				constexpr std::size_t HEADER_SIZE = 64;
				constexpr std::size_t SEC_HEADER_SIZE = 64;
				constexpr std::int32_t NUM_SECTIONS = 3;
				constexpr std::int32_t DATA_SECTION_INDEX = 2;
				constexpr std::uint16_t MAX_PREDICATE = 0x0015;
				constexpr std::uint16_t PREDICATE_ARRAY_SIZE = 0;
				constexpr std::uint32_t ARRAY_FLAG = 0x80000000; // DONT_DEALLOCATE

				constexpr const char* CONTENTS_VERSION = "hk_2014.2.5-r1";

				constexpr const char* CLASS_LIST = "HavokPhysicsShapeList";
				constexpr const char* CLASS_SHAPE = "hknpCompressedMeshShape";
				constexpr const char* CLASS_DATA = "hknpCompressedMeshShapeData";

				// ---- codec constants (must match havok_debug.cpp exactly) ----
				constexpr float K_NODE = 0.004424778761061947f; // 1/226
				constexpr float R21 = 1.0f / ((1 << 21) - 1);
				constexpr float R22 = 1.0f / ((1 << 22) - 1);
				constexpr std::uint32_t Q21 = (1u << 21) - 1;
				constexpr std::uint32_t Q22 = (1u << 22) - 1;

				// <=128 prims/section (prim<<1|tri must fit the low 8 bits of the
				// shape key, and the 1-byte nav payload is 7 bits)
				constexpr std::size_t CAP_PRIM = 128;
				// <=255 shared verts/section (numSharedIndices is a u8)
				constexpr std::size_t CAP_VERT = 255;
				// u16 remap -> the shared descriptor pool is paged into windows
				constexpr std::size_t SHARED_WINDOW = 65536;

				// object sizes taken from the IW7 reflection tables
				constexpr std::size_t SIZE_LIST = 152;
				constexpr std::size_t SIZE_SHAPE = 160;
				constexpr std::size_t SIZE_DATA = 256;
				constexpr std::size_t SIZE_SECTION = 96;

				struct classname_entry { const char* name; std::uint32_t signature; };

				// harvested getSignature() CRCs (retail_sample_findings.md)
				constexpr classname_entry REFLECTION_PREAMBLE[] = {
					{ "hkClass",         0x33D42383 },
					{ "hkClassMember",   0xB0EFA719 },
					{ "hkClassEnum",     0x8A3609CF },
					{ "hkClassEnumItem", 0xCE6F8A6C },
				};
				constexpr std::uint32_t SIG_LIST = 0xC909A395;
				constexpr std::uint32_t SIG_SHAPE = 0x1318CC9F;
				constexpr std::uint32_t SIG_DATA = 0x54FD8D57;

				// ------------------------------------------------------------------
				// crc32 (IEEE, zlib compatible) - materialCRC
				// ------------------------------------------------------------------
				std::uint32_t crc32_bytes(const void* data, std::size_t len)
				{
					static const auto table = []
					{
						std::array<std::uint32_t, 256> t{};
						for (auto i = 0u; i < 256; i++)
						{
							auto c = i;
							for (auto k = 0; k < 8; k++)
							{
								c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
							}
							t[i] = c;
						}
						return t;
					}();

					const auto* p = static_cast<const std::uint8_t*>(data);
					auto c = 0xFFFFFFFFu;
					for (std::size_t i = 0; i < len; i++)
					{
						c = table[(c ^ p[i]) & 0xFF] ^ (c >> 8);
					}
					return c ^ 0xFFFFFFFFu;
				}

				// ------------------------------------------------------------------
				// byte buffer
				// ------------------------------------------------------------------
				struct byte_buffer
				{
					std::vector<std::uint8_t> d;

					std::size_t size() const { return this->d.size(); }

					void pad_to(std::size_t align, std::uint8_t fill)
					{
						while (this->d.size() % align)
						{
							this->d.push_back(fill);
						}
					}

					void append(const void* src, std::size_t len)
					{
						const auto* p = static_cast<const std::uint8_t*>(src);
						this->d.insert(this->d.end(), p, p + len);
					}

					void append_zero(std::size_t len)
					{
						this->d.insert(this->d.end(), len, static_cast<std::uint8_t>(0));
					}

					template <typename T> void put(std::size_t off, T value)
					{
						std::memcpy(this->d.data() + off, &value, sizeof(T));
					}

					template <typename T> void push(T value)
					{
						const auto off = this->d.size();
						this->d.resize(off + sizeof(T));
						std::memcpy(this->d.data() + off, &value, sizeof(T));
					}
				};

				// ------------------------------------------------------------------
				// Codec D: node AABB nibbles. decode_node_aabb() must stay
				// bit-identical to havok_debug.cpp's refine().
				// ------------------------------------------------------------------
				void decode_node_aabb(const std::uint8_t* ab, const float* pmin, const float* pmax,
					float* cmin, float* cmax)
				{
					for (auto a = 0; a < 3; a++)
					{
						const auto ext = pmax[a] - pmin[a];
						const auto hi = static_cast<float>(ab[a] >> 4);
						const auto lo = static_cast<float>(ab[a] & 0xF);
						cmin[a] = pmin[a] + hi * hi * ext * K_NODE;
						cmax[a] = pmax[a] - lo * lo * ext * K_NODE;
					}
				}

				// conservative: the stored box always contains the true box
				void encode_node_nibbles(const float* cmin, const float* cmax,
					const float* pmin, const float* pmax, std::uint8_t* out)
				{
					for (auto a = 0; a < 3; a++)
					{
						const auto ext = pmax[a] - pmin[a];
						const auto s = ext * K_NODE;
						if (!(s > 0.0f))
						{
							out[a] = 0;
							continue;
						}

						const auto dmin = std::max(0.0f, cmin[a] - pmin[a]);
						const auto dmax = std::max(0.0f, pmax[a] - cmax[a]);

						auto hi = static_cast<int>(std::floor(std::sqrt(dmin / s)));
						auto lo = static_cast<int>(std::floor(std::sqrt(dmax / s)));
						hi = std::clamp(hi, 0, 15);
						lo = std::clamp(lo, 0, 15);

						// guard against a rounding overshoot making the stored box
						// smaller than the true box
						while (hi > 0 && pmin[a] + static_cast<float>(hi) * static_cast<float>(hi) * ext * K_NODE > cmin[a])
						{
							hi--;
						}
						while (lo > 0 && pmax[a] - static_cast<float>(lo) * static_cast<float>(lo) * ext * K_NODE < cmax[a])
						{
							lo--;
						}

						out[a] = static_cast<std::uint8_t>((hi << 4) | lo);
					}
				}

				// ------------------------------------------------------------------
				// BVH build (full binary tree, DFS pre-order, delta = left leaf count)
				// ------------------------------------------------------------------
				struct tree_leaf
				{
					float bmin[3];
					float bmax[3];
					float centroid[3];
					unsigned int value;
				};

				struct tree_node
				{
					std::uint8_t ab[3];
					bool is_leaf;
					unsigned int payload;
				};

				struct tree_builder
				{
					std::vector<tree_node> nodes;

					unsigned int rec(std::vector<tree_leaf>& items, std::size_t lo, std::size_t hi,
						const float* pmin, const float* pmax, bool is_root)
					{
						const auto count = hi - lo;
						if (count == 1)
						{
							tree_node nd{};
							if (!is_root)
							{
								encode_node_nibbles(items[lo].bmin, items[lo].bmax, pmin, pmax, nd.ab);
							}
							nd.is_leaf = true;
							nd.payload = items[lo].value;
							this->nodes.push_back(nd);
							return 1;
						}

						float emin[3];
						float emax[3];
						tree_node nd{};

						// the walker never applies the root node's own aabb bytes -
						// the root box is the stored domain / section box, so encode
						// the root's children against that box unchanged.
						if (is_root)
						{
							for (auto a = 0; a < 3; a++)
							{
								emin[a] = pmin[a];
								emax[a] = pmax[a];
							}
						}
						else
						{
							float bmin[3];
							float bmax[3];
							for (auto a = 0; a < 3; a++)
							{
								bmin[a] = items[lo].bmin[a];
								bmax[a] = items[lo].bmax[a];
							}
							for (auto i = lo + 1; i < hi; i++)
							{
								for (auto a = 0; a < 3; a++)
								{
									bmin[a] = std::min(bmin[a], items[i].bmin[a]);
									bmax[a] = std::max(bmax[a], items[i].bmax[a]);
								}
							}
							encode_node_nibbles(bmin, bmax, pmin, pmax, nd.ab);
							decode_node_aabb(nd.ab, pmin, pmax, emin, emax);
						}

						const auto my_index = this->nodes.size();
						this->nodes.push_back({});

						auto axis = 0;
						for (auto a = 1; a < 3; a++)
						{
							if (emax[a] - emin[a] > emax[axis] - emin[axis])
							{
								axis = a;
							}
						}

						std::stable_sort(this->begin_of(items, lo), this->begin_of(items, hi),
							[axis](const tree_leaf& a, const tree_leaf& b)
							{
								return a.centroid[axis] < b.centroid[axis];
							});

						const auto mid = lo + count / 2;
						const auto left_leaves = this->rec(items, lo, mid, emin, emax, false);
						this->rec(items, mid, hi, emin, emax, false);

						nd.is_leaf = false;
						nd.payload = left_leaves;
						this->nodes[my_index] = nd;
						return static_cast<unsigned int>(count);
					}

				private:
					static std::vector<tree_leaf>::iterator begin_of(std::vector<tree_leaf>& v, std::size_t i)
					{
						return v.begin() + static_cast<std::ptrdiff_t>(i);
					}
				};

				std::vector<std::uint8_t> build_tree(std::vector<tree_leaf>& leaves,
					const float* root_min, const float* root_max, int nav_bytes, unsigned int* node_count)
				{
					tree_builder builder{};
					builder.rec(leaves, 0, leaves.size(), root_min, root_max, true);

					std::vector<std::uint8_t> out;
					out.reserve(builder.nodes.size() * (3 + nav_bytes));
					for (const auto& nd : builder.nodes)
					{
						out.push_back(nd.ab[0]);
						out.push_back(nd.ab[1]);
						out.push_back(nd.ab[2]);

						if (nav_bytes == 1)
						{
							if (nd.payload > 0x7F)
							{
								throw std::runtime_error("havok_encode: section sub-tree nav payload overflow");
							}
							std::uint8_t nav;
							if (SECTION_NAV_BIT7)
							{
								nav = static_cast<std::uint8_t>(nd.payload | (nd.is_leaf ? 0u : 0x80u));
							}
							else
							{
								nav = static_cast<std::uint8_t>((nd.payload << 1) | (nd.is_leaf ? 0u : 1u));
							}
							out.push_back(nav);
						}
						else
						{
							if (nd.payload > 0x7FFF)
							{
								throw std::runtime_error("havok_encode: top-tree nav payload overflow");
							}
							auto nav0 = static_cast<std::uint8_t>((nd.payload >> 8) & 0x7F);
							const auto nav1 = static_cast<std::uint8_t>(nd.payload & 0xFF);
							if (!nd.is_leaf)
							{
								nav0 |= 0x80;
							}
							out.push_back(nav0);
							out.push_back(nav1);
						}
					}

					*node_count = static_cast<unsigned int>(builder.nodes.size());
					return out;
				}

				// ------------------------------------------------------------------
				// __data__ payload assembler
				// ------------------------------------------------------------------
				struct local_fixup { std::int32_t from; std::int32_t to; };
				struct global_fixup { std::int32_t from; std::int32_t section; std::int32_t to; };
				struct virtual_fixup { std::int32_t obj; std::int32_t name_section; const char* name; };

				struct payload
				{
					byte_buffer buf;
					std::vector<local_fixup> locals;
					std::vector<global_fixup> globals;
					std::vector<virtual_fixup> virtuals;

					std::int32_t place_object(std::size_t size, const char* classname)
					{
						this->buf.pad_to(16, 0x00);
						const auto off = static_cast<std::int32_t>(this->buf.size());
						this->buf.append_zero(size);
						this->virtuals.push_back({ off, 0, classname });
						return off;
					}

					std::int32_t block(const void* data, std::size_t size)
					{
						this->buf.pad_to(16, 0x00);
						const auto off = static_cast<std::int32_t>(this->buf.size());
						if (size)
						{
							this->buf.append(data, size);
						}
						return off;
					}

					// hkArray { void* data; u32 count; u32 capacityAndFlags; }
					void set_array(std::int32_t member_off, std::int32_t block_off, unsigned int count)
					{
						this->buf.put<std::uint64_t>(member_off, 0);
						this->buf.put<std::uint32_t>(member_off + 8, count);
						this->buf.put<std::uint32_t>(member_off + 12, count | ARRAY_FLAG);
						if (block_off >= 0 && count > 0)
						{
							this->locals.push_back({ member_off, block_off });
						}
					}

					void set_pointer(std::int32_t member_off, std::int32_t target_off)
					{
						this->buf.put<std::uint64_t>(member_off, 0);
						this->globals.push_back({ member_off, DATA_SECTION_INDEX, target_off });
					}
				};

				// ------------------------------------------------------------------
				// container assembly
				// ------------------------------------------------------------------
				void write_section_header(byte_buffer& out, const char* tag, std::int32_t abs_start,
					std::int32_t local_off, std::int32_t global_off, std::int32_t virtual_off,
					std::int32_t exports_off, std::int32_t imports_off, std::int32_t end_off)
				{
					std::uint8_t h[SEC_HEADER_SIZE];
					std::memset(h, 0xFF, sizeof(h));

					const auto len = std::strlen(tag);
					std::memset(h, 0x00, 19);
					std::memcpy(h, tag, std::min<std::size_t>(len, 19));

					const std::int32_t fields[7] = {
						abs_start, local_off, global_off, virtual_off, exports_off, imports_off, end_off };
					std::memcpy(h + 0x14, fields, sizeof(fields));

					out.append(h, sizeof(h));
				}

				std::vector<std::uint8_t> assemble(const std::vector<classname_entry>& classnames,
					const char* root_classname, const byte_buffer& object_payload,
					const std::vector<local_fixup>& locals, const std::vector<global_fixup>& globals,
					const std::vector<virtual_fixup>& virtuals)
				{
					// ---- __classnames__ ----
					byte_buffer cn{};
					std::unordered_map<std::string, std::int32_t> name_off;
					for (const auto& e : classnames)
					{
						cn.push<std::uint32_t>(e.signature);
						cn.push<std::uint8_t>(0x09);
						name_off[e.name] = static_cast<std::int32_t>(cn.size());
						cn.append(e.name, std::strlen(e.name) + 1);
					}
					cn.pad_to(16, 0xFF);

					const auto root_it = name_off.find(root_classname);
					if (root_it == name_off.end())
					{
						throw std::runtime_error("havok_encode: root class missing from classnames");
					}

					// ---- __data__ ----
					byte_buffer data = object_payload;
					data.pad_to(16, 0x00);
					const auto local_off = static_cast<std::int32_t>(data.size());

					for (const auto& f : locals)
					{
						data.push<std::int32_t>(f.from);
						data.push<std::int32_t>(f.to);
					}
					data.pad_to(16, 0xFF);
					const auto global_off = static_cast<std::int32_t>(data.size());

					for (const auto& f : globals)
					{
						data.push<std::int32_t>(f.from);
						data.push<std::int32_t>(f.section);
						data.push<std::int32_t>(f.to);
					}
					data.pad_to(16, 0xFF);
					const auto virtual_off = static_cast<std::int32_t>(data.size());

					for (const auto& f : virtuals)
					{
						const auto it = name_off.find(f.name);
						if (it == name_off.end())
						{
							throw std::runtime_error("havok_encode: virtual fixup class missing from classnames");
						}
						data.push<std::int32_t>(f.obj);
						data.push<std::int32_t>(f.name_section);
						data.push<std::int32_t>(it->second);
					}
					data.pad_to(16, 0xFF);
					const auto end_off = static_cast<std::int32_t>(data.size());

					const auto cn_abs = static_cast<std::int32_t>(HEADER_SIZE + NUM_SECTIONS * SEC_HEADER_SIZE);
					const auto data_abs = cn_abs + static_cast<std::int32_t>(cn.size());
					const auto cn_size = static_cast<std::int32_t>(cn.size());

					// ---- header ----
					byte_buffer out{};
					{
						std::uint8_t h[HEADER_SIZE]{};
						const std::uint32_t magic[2] = { MAGIC0, MAGIC1 };
						std::memcpy(h + 0x00, magic, sizeof(magic));
						const std::int32_t user_tag = 0;
						std::memcpy(h + 0x08, &user_tag, 4);
						std::memcpy(h + 0x0C, &FILE_VERSION, 4);
						h[0x10] = 8; // bytesInPointer
						h[0x11] = 1; // littleEndian
						h[0x12] = 0; // reuseBaseClassPadding
						h[0x13] = 1; // emptyBaseClassOptimization

						const std::int32_t contents[5] = {
							NUM_SECTIONS, DATA_SECTION_INDEX, 0, 0, root_it->second };
						std::memcpy(h + 0x14, contents, sizeof(contents));

						const auto vlen = std::strlen(CONTENTS_VERSION);
						std::memcpy(h + 0x28, CONTENTS_VERSION, vlen);
						h[0x28 + vlen] = 0x00;
						h[0x37] = 0xFF; // retail leaves the char[16] tail 0xFF

						const std::int32_t flags = 0;
						std::memcpy(h + 0x38, &flags, 4);
						std::memcpy(h + 0x3C, &MAX_PREDICATE, 2);
						std::memcpy(h + 0x3E, &PREDICATE_ARRAY_SIZE, 2);

						out.append(h, sizeof(h));
					}

					write_section_header(out, "__classnames__", cn_abs,
						cn_size, cn_size, cn_size, cn_size, cn_size, cn_size);
					write_section_header(out, "__types__", data_abs, 0, 0, 0, 0, 0, 0);
					write_section_header(out, "__data__", data_abs,
						local_off, global_off, virtual_off, end_off, end_off, end_off);

					out.append(cn.d.data(), cn.d.size());
					out.append(data.d.data(), data.d.size());
					return out.d;
				}

				// ------------------------------------------------------------------
				// vertex quantization (shared 21/21/22 against the mesh domain)
				// ------------------------------------------------------------------
				std::uint64_t quantize_shared(const float* v, const float* dmin, const float* ext)
				{
					const auto q = [](float value, float mn, float e, std::uint32_t m) -> std::uint64_t
					{
						if (!(e > 0.0f))
						{
							return 0;
						}
						const auto f = (static_cast<double>(value) - static_cast<double>(mn)) / static_cast<double>(e);
						const auto qi = static_cast<long long>(std::llround(f * static_cast<double>(m)));
						if (qi < 0)
						{
							return 0;
						}
						if (qi > static_cast<long long>(m))
						{
							return m;
						}
						return static_cast<std::uint64_t>(qi);
					};

					return q(v[0], dmin[0], ext[0], Q21)
						| (q(v[1], dmin[1], ext[1], Q21) << 21)
						| (q(v[2], dmin[2], ext[2], Q22) << 42);
				}

				// must stay bit-identical to havok_debug.cpp's dec_shared()
				void dequantize_shared(std::uint64_t d, const float* dmin, const float* ext, float* out)
				{
					const auto qx = static_cast<float>(d & 0x1FFFFF);
					const auto qy = static_cast<float>((d >> 21) & 0x1FFFFF);
					const auto qz = static_cast<float>((d >> 42) & 0x3FFFFF);
					out[0] = dmin[0] + qx * R21 * ext[0];
					out[1] = dmin[1] + qy * R21 * ext[1];
					out[2] = dmin[2] + qz * R22 * ext[2];
				}

				unsigned int bit_length(unsigned int v)
				{
					auto n = 0u;
					while (v)
					{
						n++;
						v >>= 1;
					}
					return n;
				}
			}

			unsigned int material_crc(const std::string& name)
			{
				auto n = name;
				if (n.rfind("PM_", 0) == 0)
				{
					n = n.substr(3);
				}
				n = "PM_" + n;
				return crc32_bytes(n.data(), n.size());
			}

			std::string build_world_collision_blob(const std::vector<triangle>& tris,
				const std::vector<tag_entry>& tags, const std::string& shape_name,
				unsigned int shape_contents, build_stats* stats)
			{
				if (tris.empty())
				{
					throw std::runtime_error("havok_encode: no triangles");
				}

				// ---- global domain ----
				float dmin[3];
				float dmax[3];
				for (auto a = 0; a < 3; a++)
				{
					dmin[a] = tris[0].v[0][a];
					dmax[a] = tris[0].v[0][a];
				}
				for (const auto& t : tris)
				{
					for (auto i = 0; i < 3; i++)
					{
						for (auto a = 0; a < 3; a++)
						{
							dmin[a] = std::min(dmin[a], t.v[i][a]);
							dmax[a] = std::max(dmax[a], t.v[i][a]);
						}
					}
				}

				// Pad the stored domain slightly so no dequantized vertex can land
				// on (or a float ulp past) the domain corners - the top-tree leaf
				// boxes are refined from the stored domain, so a section box that
				// touches it exactly can fail containment by an ulp.
				float ext[3];
				for (auto a = 0; a < 3; a++)
				{
					const auto pad = 0.125f + (dmax[a] - dmin[a]) * 1e-5f;
					dmin[a] -= pad;
					dmax[a] += pad;
					ext[a] = dmax[a] - dmin[a];
				}

				// ---- quantize + dedup verts into the shared descriptor pool ----
				std::vector<std::uint64_t> shared;
				std::vector<std::array<float, 3>> vpos;
				std::unordered_map<std::uint64_t, unsigned int> desc_map;
				struct tri_ref { unsigned int g[3]; unsigned int tag; };
				std::vector<tri_ref> tri_gids;
				tri_gids.reserve(tris.size());

				for (const auto& t : tris)
				{
					tri_ref r{};
					for (auto i = 0; i < 3; i++)
					{
						const auto d = quantize_shared(t.v[i], dmin, ext);
						const auto it = desc_map.find(d);
						if (it != desc_map.end())
						{
							r.g[i] = it->second;
						}
						else
						{
							const auto gid = static_cast<unsigned int>(shared.size());
							desc_map.emplace(d, gid);
							shared.push_back(d);
							std::array<float, 3> p{};
							dequantize_shared(d, dmin, ext, p.data());
							vpos.push_back(p);
							r.g[i] = gid;
						}
					}
					r.tag = t.tag_index;
					tri_gids.push_back(r);
				}

				// ---- morton-ordered greedy sectioning ----
				std::vector<std::uint32_t> order(tri_gids.size());
				{
					std::vector<std::uint64_t> keys(tri_gids.size());
					for (std::size_t i = 0; i < tri_gids.size(); i++)
					{
						float c[3]{};
						for (auto a = 0; a < 3; a++)
						{
							c[a] = (vpos[tri_gids[i].g[0]][a] + vpos[tri_gids[i].g[1]][a]
								+ vpos[tri_gids[i].g[2]][a]) / 3.0f;
						}
						std::uint32_t gi[3]{};
						for (auto a = 0; a < 3; a++)
						{
							if (ext[a] > 0.0f)
							{
								auto q = static_cast<int>((c[a] - dmin[a]) / ext[a] * 1023.0f);
								q = std::clamp(q, 0, 1023);
								gi[a] = static_cast<std::uint32_t>(q);
							}
						}
						std::uint64_t key = 0;
						for (auto b = 0; b < 10; b++)
						{
							key |= static_cast<std::uint64_t>((gi[0] >> b) & 1) << (3 * b);
							key |= static_cast<std::uint64_t>((gi[1] >> b) & 1) << (3 * b + 1);
							key |= static_cast<std::uint64_t>((gi[2] >> b) & 1) << (3 * b + 2);
						}
						keys[i] = key;
						order[i] = static_cast<std::uint32_t>(i);
					}
					std::stable_sort(order.begin(), order.end(),
						[&keys](std::uint32_t a, std::uint32_t b) { return keys[a] < keys[b]; });
				}

				std::vector<std::vector<std::uint32_t>> sections;
				{
					std::vector<std::uint32_t> cur;
					std::vector<unsigned int> cur_verts;
					const auto has_vert = [&cur_verts](unsigned int g)
					{
						return std::find(cur_verts.begin(), cur_verts.end(), g) != cur_verts.end();
					};

					for (const auto ti : order)
					{
						const auto& r = tri_gids[ti];
						auto added = 0;
						for (auto i = 0; i < 3; i++)
						{
							auto dup = false;
							for (auto k = 0; k < i; k++)
							{
								dup |= r.g[k] == r.g[i];
							}
							if (!dup && !has_vert(r.g[i]))
							{
								added++;
							}
						}

						if (!cur.empty() && (cur.size() >= CAP_PRIM || cur_verts.size() + added > CAP_VERT))
						{
							sections.push_back(cur);
							cur.clear();
							cur_verts.clear();
						}

						for (auto i = 0; i < 3; i++)
						{
							if (!has_vert(r.g[i]))
							{
								cur_verts.push_back(r.g[i]);
							}
						}
						cur.push_back(ti);
					}
					if (!cur.empty())
					{
						sections.push_back(cur);
					}
				}

				const auto num_sections = static_cast<unsigned int>(sections.size());
				if (num_sections > 32768)
				{
					throw std::runtime_error("havok_encode: too many sections for the 15-bit top-tree nav");
				}

				// ---- shapeTagData ----
				std::vector<tag_entry> tagdata = tags;
				if (tagdata.empty())
				{
					tagdata.push_back({ shape_contents, 0, 0xFFFF, 0 });
				}

				// ---- per-section encode ----
				std::vector<std::uint8_t> g_primitives;
				std::vector<std::uint16_t> g_shared_index;
				std::vector<std::uint32_t> g_dataruns;
				std::vector<std::vector<std::uint8_t>> section_nodes;
				std::vector<tree_leaf> top_leaves;

				struct section_record
				{
					float smin[3];
					float smax[3];
					float codec[6];
					std::uint32_t sv_handle;
					std::uint32_t prim_handle;
					std::uint32_t dr_handle;
					std::uint8_t num_shared;
					std::uint16_t leaf_index;
					std::uint32_t window;
				};
				std::vector<section_record> records;

				std::vector<std::uint64_t> sv_pool;
				std::unordered_map<unsigned int, unsigned int> win_map;
				auto cur_window = 0u;

				auto prim_off = 0u;
				auto dr_off = 0u;
				auto svi_off = 0u;
				auto max_shape_key = 0u;
				auto total_tris = 0u;

				std::vector<unsigned int> local_ids;
				std::unordered_map<unsigned int, unsigned int> local_map;

				for (auto si = 0u; si < num_sections; si++)
				{
					const auto& tri_list = sections[si];

					local_ids.clear();
					local_map.clear();
					for (const auto ti : tri_list)
					{
						for (auto i = 0; i < 3; i++)
						{
							const auto g = tri_gids[ti].g[i];
							if (local_map.emplace(g, static_cast<unsigned int>(local_ids.size())).second)
							{
								local_ids.push_back(g);
							}
						}
					}

					const auto num_shared = static_cast<unsigned int>(local_ids.size());
					if (num_shared > CAP_VERT)
					{
						throw std::runtime_error("havok_encode: section shared vertex overflow");
					}

					// shared-descriptor window paging (u16 remap is window relative)
					{
						auto new_count = 0u;
						for (const auto g : local_ids)
						{
							if (!win_map.count(g))
							{
								new_count++;
							}
						}
						if (win_map.size() + new_count > SHARED_WINDOW)
						{
							while (sv_pool.size() < static_cast<std::size_t>(cur_window + 1) * SHARED_WINDOW)
							{
								sv_pool.push_back(0);
							}
							cur_window++;
							win_map.clear();
						}
						for (const auto g : local_ids)
						{
							if (!win_map.count(g))
							{
								win_map.emplace(g, static_cast<unsigned int>(win_map.size()));
								sv_pool.push_back(shared[g]);
							}
						}
					}

					section_record rec{};
					rec.window = cur_window;
					rec.leaf_index = static_cast<std::uint16_t>(si);
					rec.num_shared = static_cast<std::uint8_t>(num_shared);

					for (auto a = 0; a < 3; a++)
					{
						rec.smin[a] = vpos[local_ids[0]][a];
						rec.smax[a] = vpos[local_ids[0]][a];
					}
					for (const auto g : local_ids)
					{
						for (auto a = 0; a < 3; a++)
						{
							rec.smin[a] = std::min(rec.smin[a], vpos[g][a]);
							rec.smax[a] = std::max(rec.smax[a], vpos[g][a]);
						}
					}

					// primitives + dataruns + sub-tree leaves
					std::vector<tree_leaf> prim_leaves;
					prim_leaves.reserve(tri_list.size());
					auto run_value = 0u;
					auto run_start = 0u;
					auto has_run = false;
					auto run_count = 0u;

					for (auto p = 0u; p < tri_list.size(); p++)
					{
						const auto& r = tri_gids[tri_list[p]];
						auto a = static_cast<std::uint8_t>(local_map[r.g[0]]);
						auto b = static_cast<std::uint8_t>(local_map[r.g[1]]);
						auto c = static_cast<std::uint8_t>(local_map[r.g[2]]);
						// 0xDEAD0000 is the dead-primitive sentinel
						if (a == 0xDE && b == 0xAD && c == 0)
						{
							const auto t = a;
							a = b;
							b = c;
							c = t;
						}
						g_primitives.push_back(a);
						g_primitives.push_back(b);
						g_primitives.push_back(c);
						g_primitives.push_back(c);

						tree_leaf leaf{};
						for (auto ax = 0; ax < 3; ax++)
						{
							leaf.bmin[ax] = std::min({ vpos[r.g[0]][ax], vpos[r.g[1]][ax], vpos[r.g[2]][ax] });
							leaf.bmax[ax] = std::max({ vpos[r.g[0]][ax], vpos[r.g[1]][ax], vpos[r.g[2]][ax] });
							leaf.centroid[ax] = (leaf.bmin[ax] + leaf.bmax[ax]) * 0.5f;
						}
						leaf.value = p;
						prim_leaves.push_back(leaf);

						if (!has_run)
						{
							has_run = true;
							run_value = r.tag;
							run_start = p;
						}
						else if (r.tag != run_value)
						{
							g_dataruns.push_back(static_cast<std::uint32_t>(
								(run_value & 0xFFFF) | ((run_start & 0xFF) << 16) | (((p - run_start) & 0xFF) << 24)));
							run_count++;
							run_value = r.tag;
							run_start = p;
						}

						const auto sk = (si << 8) | (p << 1);
						max_shape_key = std::max(max_shape_key, sk);
						total_tris++;
					}
					if (has_run)
					{
						const auto n = static_cast<unsigned int>(tri_list.size()) - run_start;
						g_dataruns.push_back(static_cast<std::uint32_t>(
							(run_value & 0xFFFF) | ((run_start & 0xFF) << 16) | ((n & 0xFF) << 24)));
						run_count++;
					}

					const auto num_prims = static_cast<unsigned int>(tri_list.size());
					if (num_prims > CAP_PRIM)
					{
						throw std::runtime_error("havok_encode: section primitive overflow");
					}

					unsigned int node_count = 0;
					section_nodes.push_back(build_tree(prim_leaves, rec.smin, rec.smax, 1, &node_count));

					tree_leaf top{};
					for (auto a = 0; a < 3; a++)
					{
						top.bmin[a] = rec.smin[a];
						top.bmax[a] = rec.smax[a];
						top.centroid[a] = (rec.smin[a] + rec.smax[a]) * 0.5f;
					}
					top.value = si;
					top_leaves.push_back(top);

					for (const auto g : local_ids)
					{
						g_shared_index.push_back(static_cast<std::uint16_t>(win_map[g]));
					}

					if ((prim_off >> 24) || (dr_off >> 24) || (svi_off >> 24))
					{
						throw std::runtime_error("havok_encode: section handle offset overflow");
					}
					rec.prim_handle = (prim_off << 8) | (num_prims & 0xFF);
					rec.dr_handle = (dr_off << 8) | (run_count & 0xFF);
					rec.sv_handle = (svi_off << 8);

					for (auto a = 0; a < 3; a++)
					{
						rec.codec[a] = rec.smin[a];
					}
					rec.codec[3] = (rec.smax[0] - rec.smin[0]) / 2047.0f;
					rec.codec[4] = (rec.smax[1] - rec.smin[1]) / 2047.0f;
					rec.codec[5] = (rec.smax[2] - rec.smin[2]) / 1023.0f;

					records.push_back(rec);

					prim_off += num_prims;
					dr_off += run_count;
					svi_off += num_shared;
				}

				unsigned int top_node_count = 0;
				const auto top_bytes = build_tree(top_leaves, dmin, dmax, 2, &top_node_count);

				const auto bits_per_key = std::max(1u, bit_length(max_shape_key));

				// =================================================================
				// serialize the __data__ object payload
				// =================================================================
				payload p{};
				const auto list_off = p.place_object(SIZE_LIST, CLASS_LIST);
				const auto shape_off = p.place_object(SIZE_SHAPE, CLASS_SHAPE);
				const auto data_off = p.place_object(SIZE_DATA, CLASS_DATA);

				// ---------- HavokPhysicsShapeList ----------
				{
					const std::uint64_t null_ptr = 0;
					const auto shapes_blk = p.block(&null_ptr, 8);
					p.set_array(list_off + 0, shapes_blk, 1);
					p.set_pointer(shapes_blk, shape_off);

					const std::int32_t zero_index = 0;
					p.set_array(list_off + 16, p.block(&zero_index, 4), 1);

					const auto name_str = p.block(shape_name.data(), shape_name.size() + 1);
					const auto names_blk = p.block(&null_ptr, 8);
					p.set_array(list_off + 32, names_blk, 1);
					p.locals.push_back({ names_blk, name_str });

					const auto vert_count = static_cast<std::int32_t>(shared.size());
					p.set_array(list_off + 48, p.block(&vert_count, 4), 1);
					const auto tri_count = static_cast<std::int32_t>(total_tris);
					p.set_array(list_off + 64, p.block(&tri_count, 4), 1);

					const float minmax[8] = {
						dmin[0], dmin[1], dmin[2], 0.0f, dmax[0], dmax[1], dmax[2], 0.0f };
					p.set_array(list_off + 80, p.block(minmax, sizeof(minmax)), 2);

					p.buf.put<std::int32_t>(list_off + 96, 0); // numWorldGeoShapes

					byte_buffer td{};
					for (const auto& e : tagdata)
					{
						td.push<std::uint32_t>(e.collision_filter_info);
						td.push<std::uint32_t>(e.material_crc);
						td.push<std::uint16_t>(e.material_id);
						td.append_zero(6);
						td.push<std::uint64_t>(e.user_data);
					}
					p.set_array(list_off + 104, p.block(td.d.data(), td.d.size()),
						static_cast<unsigned int>(tagdata.size()));

					const auto contents = static_cast<std::uint32_t>(shape_contents);
					p.set_array(list_off + 120, p.block(&contents, 4), 1);

					const std::int32_t convex_counts = 0;
					p.set_array(list_off + 136, p.block(&convex_counts, 4), 1);
				}

				// ---------- hknpCompressedMeshShape ----------
				{
					p.buf.put<std::uint16_t>(shape_off + 16, 0x0004); // IS_COMPOSITE_SHAPE
					p.buf.put<std::uint8_t>(shape_off + 18, static_cast<std::uint8_t>(bits_per_key));
					p.buf.put<std::uint8_t>(shape_off + 19, 2); // dispatchType = COMPOSITE
					p.buf.put<float>(shape_off + 20, 0.0f); // convexRadius
					p.buf.put<std::uint64_t>(shape_off + 24, 0); // userData
					p.buf.put<std::uint64_t>(shape_off + 32, 0); // properties = null

					// edgeWeldingMap: retail's empty pattern
					p.buf.put<std::uint32_t>(shape_off + 48, 0xFFFFFFFF);
					p.buf.put<std::uint32_t>(shape_off + 52, 0);
					p.set_array(shape_off + 56, -1, 0);
					p.set_array(shape_off + 72, -1, 0);
					p.buf.put<std::uint32_t>(shape_off + 88, 0xFFFFFFFF); // shapeTagCodecInfo

					p.set_pointer(shape_off + 96, data_off);

					const auto quad_bits = max_shape_key / 2 + 1;
					const auto quad_words = (quad_bits + 31) / 32;
					std::vector<std::uint32_t> quad(quad_words, 0);
					p.set_array(shape_off + 104, p.block(quad.data(), quad.size() * 4), quad_words);
					p.buf.put<std::uint32_t>(shape_off + 104 + 16, quad_bits);

					const auto tri_bits = max_shape_key + 1;
					const auto tri_words = (tri_bits + 31) / 32;
					std::vector<std::uint32_t> tri_field(tri_words, 0);
					p.set_array(shape_off + 128, p.block(tri_field.data(), tri_field.size() * 4), tri_words);
					p.buf.put<std::uint32_t>(shape_off + 128 + 16, tri_bits);

					p.buf.put<std::int32_t>(shape_off + 152, 0); // numTriangles
					p.buf.put<std::int32_t>(shape_off + 156, 0); // numConvexShapes
				}

				// ---------- hknpCompressedMeshShapeData ----------
				{
					const auto tree = data_off + 16;

					p.set_array(tree + 0, p.block(top_bytes.data(), top_bytes.size()), top_node_count);

					const float domain[8] = {
						dmin[0], dmin[1], dmin[2], 0.0f, dmax[0], dmax[1], dmax[2], 0.0f };
					std::memcpy(p.buf.d.data() + tree + 16, domain, sizeof(domain));

					p.buf.put<std::int32_t>(tree + 48, static_cast<std::int32_t>(total_tris));
					p.buf.put<std::int32_t>(tree + 52, static_cast<std::int32_t>(bits_per_key));
					p.buf.put<std::uint32_t>(tree + 56, max_shape_key);

					// sub-tree node blocks first, then the section image array
					std::vector<std::int32_t> subtree_off(num_sections);
					for (auto si = 0u; si < num_sections; si++)
					{
						subtree_off[si] = p.block(section_nodes[si].data(), section_nodes[si].size());
					}

					byte_buffer sec_img{};
					for (auto si = 0u; si < num_sections; si++)
					{
						const auto& r = records[si];
						const auto node_count = static_cast<std::uint32_t>(section_nodes[si].size() / 4);
						const auto base = sec_img.size();
						sec_img.append_zero(SIZE_SECTION);

						sec_img.put<std::uint64_t>(base + 0, 0);
						sec_img.put<std::uint32_t>(base + 8, node_count);
						sec_img.put<std::uint32_t>(base + 12, node_count | ARRAY_FLAG);

						const float box[8] = {
							r.smin[0], r.smin[1], r.smin[2], 0.0f, r.smax[0], r.smax[1], r.smax[2], 0.0f };
						std::memcpy(sec_img.d.data() + base + 16, box, sizeof(box));
						std::memcpy(sec_img.d.data() + base + 48, r.codec, sizeof(r.codec));

						sec_img.put<std::uint32_t>(base + 72, 0); // firstPackedVertex
						sec_img.put<std::uint32_t>(base + 76, r.sv_handle);
						sec_img.put<std::uint32_t>(base + 80, r.prim_handle);
						sec_img.put<std::uint32_t>(base + 84, r.dr_handle);
						sec_img.put<std::uint8_t>(base + 88, 0); // numPackedVertices
						sec_img.put<std::uint8_t>(base + 89, r.num_shared);
						sec_img.put<std::uint16_t>(base + 90, r.leaf_index);
						sec_img.put<std::uint32_t>(base + 92, r.window);
					}
					const auto sec_blk = p.block(sec_img.d.data(), sec_img.d.size());
					p.set_array(tree + 64, sec_blk, num_sections);

					for (auto si = 0u; si < num_sections; si++)
					{
						if (!section_nodes[si].empty())
						{
							p.locals.push_back({ sec_blk + static_cast<std::int32_t>(SIZE_SECTION * si), subtree_off[si] });
						}
					}

					p.set_array(tree + 80, p.block(g_primitives.data(), g_primitives.size()),
						static_cast<unsigned int>(g_primitives.size() / 4));
					p.set_array(tree + 96, p.block(g_shared_index.data(), g_shared_index.size() * 2),
						static_cast<unsigned int>(g_shared_index.size()));
					p.set_array(tree + 112, -1, 0); // packedVertices
					p.set_array(tree + 128, p.block(sv_pool.data(), sv_pool.size() * 8),
						static_cast<unsigned int>(sv_pool.size()));
					p.set_array(tree + 144, p.block(g_dataruns.data(), g_dataruns.size() * 4),
						static_cast<unsigned int>(g_dataruns.size()));

					// hkcdSimdTree. Retail ships a REAL one (mp_paris: 70179 nodes) and
					// the query dispatcher at 0x141005F45 dereferences m_data with no
					// null check and without ever reading m_size:
					//   mov rax,[rbx+0B8h]; movaps xmm0,[rax+70h]; cmpleps xmm0,[rax+80h]
					// sizeof(hkcdSimdTreeNode) == 0x70, so that compares node[1].min
					// against node[1].max. An empty array (null m_data) is therefore an
					// access violation, and all-zero filler passes the compare on every
					// lane and routes queries into the SIMD walker, which finds nothing.
					// Emit two nodes whose AABB is inverted (min +3.4e38 > max -3.4e38)
					// so every lane fails and the dispatcher falls back to the
					// compressed mesh tree walk.
					constexpr std::uint32_t SIMD_EMPTY_MIN = 0x7F7FFFEE; // +3.402770e38
					constexpr std::uint32_t SIMD_EMPTY_MAX = 0xFF7FFFEE; // -3.402770e38
					std::vector<std::uint32_t> simd_nodes;
					for (auto n = 0; n < 2; n++)
					{
						for (auto axis = 0; axis < 3; axis++) // SoA: minA[4] then maxA[4]
						{
							simd_nodes.insert(simd_nodes.end(), 4, SIMD_EMPTY_MIN);
							simd_nodes.insert(simd_nodes.end(), 4, SIMD_EMPTY_MAX);
						}
						simd_nodes.insert(simd_nodes.end(), 4, 0u); // child indices
					}
					p.set_array(data_off + 176 + 8,
						p.block(simd_nodes.data(), simd_nodes.size() * 4), 2);
					// connectivity: three empty hkArrays
					p.set_array(data_off + 200 + 0, -1, 0);
					p.set_array(data_off + 200 + 16, -1, 0);
					p.set_array(data_off + 200 + 32, -1, 0);
				}

				// =================================================================
				// container
				// =================================================================
				std::vector<classname_entry> classnames(std::begin(REFLECTION_PREAMBLE), std::end(REFLECTION_PREAMBLE));
				classnames.push_back({ CLASS_LIST, SIG_LIST });
				classnames.push_back({ CLASS_SHAPE, SIG_SHAPE });
				classnames.push_back({ CLASS_DATA, SIG_DATA });

				const auto blob = assemble(classnames, CLASS_LIST, p.buf, p.locals, p.globals, p.virtuals);

				if (stats)
				{
					stats->triangle_count = total_tris;
					stats->section_count = num_sections;
					stats->shared_vertex_count = static_cast<unsigned int>(sv_pool.size());
					stats->shared_vertex_windows = cur_window + 1;
					stats->top_node_count = top_node_count;
					stats->bits_per_key = bits_per_key;
					for (auto a = 0; a < 3; a++)
					{
						stats->domain_min[a] = dmin[a];
						stats->domain_max[a] = dmax[a];
					}
				}

				return std::string(reinterpret_cast<const char*>(blob.data()), blob.size());
			}

			std::string build_empty_shape_list_blob()
			{
				payload p{};
				const auto list_off = p.place_object(SIZE_LIST, CLASS_LIST);

				p.set_array(list_off + 0, -1, 0); // shapes
				p.set_array(list_off + 16, -1, 0); // shapeIndices
				p.set_array(list_off + 32, -1, 0); // names
				p.set_array(list_off + 48, -1, 0); // vertexCounts
				p.set_array(list_off + 64, -1, 0); // triangleCounts
				p.set_array(list_off + 80, -1, 0); // bounds
				p.buf.put<std::int32_t>(list_off + 96, 0); // numWorldGeoShapes
				p.set_array(list_off + 104, -1, 0); // shapeTagData
				p.set_array(list_off + 120, -1, 0); // contents
				p.set_array(list_off + 136, -1, 0); // convexShapeCounts

				std::vector<classname_entry> classnames(std::begin(REFLECTION_PREAMBLE), std::end(REFLECTION_PREAMBLE));
				classnames.push_back({ CLASS_LIST, SIG_LIST });

				const auto blob = assemble(classnames, CLASS_LIST, p.buf, p.locals, p.globals, p.virtuals);
				return std::string(reinterpret_cast<const char*>(blob.data()), blob.size());
			}
		}
	}
}
