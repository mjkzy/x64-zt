#include <std_include.hpp>

#include "zonetool/h1/converter/iw7/include.hpp"
#include "clipmap_collision.hpp"
#ifdef EXPERIMENTAL_IW7

#include "zonetool/iw7/common/havok_encode.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

// H1 -> IW7 world collision geometry extraction.
//
// Rebuilds the collidable world surface from the live zonetool::h1::clipMap_t:
//   * brush polytope faces (winding walk ported from
//     zonetool/iw6/converter/h1/assets/physworld.cpp, which is the validated
//     reimplementation of the engine's own CM_GetWindingForBrushFace)
//   * terrain / patch collision partitions (triIndices over pCollisionData.verts)
// Brushes and partitions owned by a cmodel (script brushmodel) are excluded -
// those become their own physics assets, not part of the world mesh.
//
// The resulting triangle soup + shapeTagData table is handed to
// zonetool::iw7::havok::encode::build_world_collision_blob.
namespace zonetool::h1
{
	namespace converter::iw7
	{
		namespace world_collision
		{
			namespace
			{
				namespace hk = zonetool::iw7::havok::encode;

				constexpr int CONTENTS_SOLID = 0x00000001;
				constexpr int CONTENTS_PLAYERCLIP = 0x00010000;
				constexpr int CONTENTS_DETAIL = 0x08000000;

				// IW7 shape tag decode is stubbed, so every primitive in the mesh
				// is solid to every query - only brushes a player must collide
				// with may be emitted. Sky / shot-clip / missile-clip brushes are
				// dropped entirely rather than tagged.
				constexpr int PLAYER_SOLID_MASK = CONTENTS_SOLID | CONTENTS_PLAYERCLIP;

				// Retail IW7 collisionFilterInfo words never exceed 0x00032280
				// (CoD contents bits 0..17); strip the detail flag.
				constexpr int CONTENTS_FILTER_MASK = ~CONTENTS_DETAIL;

				struct axial_planes_t
				{
					float planes[6][4];
				};

				struct winding_t
				{
					float pts[4096][3];
					unsigned int count;
				};

				axial_planes_t get_axial_planes(const Bounds* bounds)
				{
					axial_planes_t axial_planes{};
					axial_planes.planes[0][0] = -1.f;
					axial_planes.planes[0][3] = -1.f * (bounds->midPoint[0] - bounds->halfSize[0]);

					axial_planes.planes[1][0] = 1.f;
					axial_planes.planes[1][3] = bounds->midPoint[0] + bounds->halfSize[0];

					axial_planes.planes[2][1] = -1.f;
					axial_planes.planes[2][3] = -1.f * (bounds->midPoint[1] - bounds->halfSize[1]);

					axial_planes.planes[3][1] = 1.f;
					axial_planes.planes[3][3] = bounds->midPoint[1] + bounds->halfSize[1];

					axial_planes.planes[4][2] = -1.f;
					axial_planes.planes[4][3] = -1.f * (bounds->midPoint[2] - bounds->halfSize[2]);

					axial_planes.planes[5][2] = 1.f;
					axial_planes.planes[5][3] = bounds->midPoint[2] + bounds->halfSize[2];

					return axial_planes;
				}

				// Bounds-checked plane fetch. The engine trusts planeIndex blindly;
				// we are walking a foreign asset, so a bad index must abort the face
				// rather than fault.
				const cplane_s* get_plane(const ClipInfo* info, unsigned int plane_index)
				{
					if (!info->planes || plane_index >= static_cast<unsigned int>(info->planeCount))
					{
						return nullptr;
					}
					return &info->planes[plane_index];
				}

				// verbatim port of physworld.cpp's get_winding_for_brush_face
				void get_winding_for_brush_face(const ClipInfo* info, const cbrush_t* brush,
					unsigned int brush_side, winding_t* out_winding, const axial_planes_t* axial_planes)
				{
					constexpr auto max_verts = 4096;

					float plane_1[4]{};
					float plane_2[4]{};
					float plane_3[4]{};

					auto adjacent_side_offset = 0;
					auto edge_count = 0;

					out_winding->count = 0;

					if (brush_side >= 6)
					{
						const auto side = &brush->sides[brush_side - 6];
						adjacent_side_offset = side->firstAdjacentSideOffset;
						edge_count = side->edgeCount;

						const auto plane = get_plane(info, side->planeIndex);
						if (!plane)
						{
							return;
						}
						plane_1[0] = plane->normal[0];
						plane_1[1] = plane->normal[1];
						plane_1[2] = plane->normal[2];
						plane_1[3] = plane->dist;
					}
					else
					{
						const auto index = 3 * (brush_side & 1) + (brush_side >> 1);
						adjacent_side_offset = brush->firstAdjacentSideOffsets[0][index];
						edge_count = brush->edgeCount[0][index];

						plane_1[0] = axial_planes->planes[brush_side][0];
						plane_1[1] = axial_planes->planes[brush_side][1];
						plane_1[2] = axial_planes->planes[brush_side][2];
						plane_1[3] = axial_planes->planes[brush_side][3];
					}

					const auto v51 = plane_1[3];
					const auto v52 = plane_1[2];
					const auto v53 = plane_1[0];

					if (edge_count < 3 || edge_count > max_verts)
					{
						return;
					}

					const auto adjacent_side = &brush->baseAdjacentSide[adjacent_side_offset];
					const auto side_index = adjacent_side[edge_count - 1];

					if (side_index >= 6u)
					{
						const auto plane = get_plane(info, brush->sides[side_index - 6].planeIndex);
						if (!plane)
						{
							return;
						}
						plane_2[0] = plane->normal[0];
						plane_2[1] = plane->normal[1];
						plane_2[2] = plane->normal[2];
						plane_2[3] = plane->dist;
					}
					else
					{
						plane_2[0] = axial_planes->planes[side_index][0];
						plane_2[1] = axial_planes->planes[side_index][1];
						plane_2[2] = axial_planes->planes[side_index][2];
						plane_2[3] = axial_planes->planes[side_index][3];
					}

					auto current_point = 0u;
					const auto points = out_winding->pts;

					for (auto i = 0; i < edge_count; i++)
					{
						const auto current_side = adjacent_side[i];
						if (current_side >= 6u)
						{
							const auto plane = get_plane(info, brush->sides[current_side - 6].planeIndex);
							if (!plane)
							{
								out_winding->count = 0;
								return;
							}
							plane_3[0] = plane->normal[0];
							plane_3[1] = plane->normal[1];
							plane_3[2] = plane->normal[2];
							plane_3[3] = plane->dist;
						}
						else
						{
							plane_3[0] = axial_planes->planes[current_side][0];
							plane_3[1] = axial_planes->planes[current_side][1];
							plane_3[2] = axial_planes->planes[current_side][2];
							plane_3[3] = axial_planes->planes[current_side][3];
						}

						const auto v38 = (plane_3[2] * plane_2[1]) - (plane_2[2] * plane_3[1]);
						const auto v39 = (plane_1[2] * plane_3[1]) - (plane_1[1] * plane_3[2]);
						const auto v40 = (plane_1[1] * plane_2[2]) - (plane_1[2] * plane_2[1]);
						const auto v41 = (plane_2[0] * v39) + (plane_1[0] * v38) + (plane_3[0] * v40);

						if (std::fabs(v41) >= 0.001f)
						{
							points[current_point][0] = (((v39 * plane_2[3]) + (v38 * v51)) + (v40 * plane_3[3])) * (1.f / v41);
							points[current_point][1] = (((((v53 * plane_3[2]) - (plane_3[0] * v52)) * plane_2[3])
								+ (((plane_3[0] * plane_2[2]) - (plane_2[0] * plane_3[2])) * v51))
								+ (((plane_2[0] * v52) - (v53 * plane_2[2])) * plane_3[3])) * (1.f / v41);
							points[current_point][2] = (((((plane_3[0] * plane_1[1]) - (v53 * plane_3[1])) * plane_2[3])
								+ (((plane_2[0] * plane_3[1]) - (plane_3[0] * plane_2[1])) * v51))
								+ (((v53 * plane_2[1]) - (plane_2[0] * plane_1[1])) * plane_3[3])) * (1.f / v41);

							if (!current_point)
							{
								current_point++;
							}
							else
							{
								const auto dx = points[current_point - 1][0] - points[current_point][0];
								const auto dy = points[current_point - 1][1] - points[current_point][1];
								const auto dz = points[current_point - 1][2] - points[current_point][2];
								if ((dx * dx) + (dy * dy) + (dz * dz) >= 1.0f)
								{
									current_point++;
								}
							}
						}

						plane_2[0] = plane_3[0];
						plane_2[1] = plane_3[1];
						plane_2[2] = plane_3[2];
						plane_2[3] = plane_3[3];
					}

					out_winding->count = current_point;

					if (current_point > 1)
					{
						const auto last = current_point - 1;
						const auto dx = points[0][0] - points[last][0];
						const auto dy = points[0][1] - points[last][1];
						const auto dz = points[0][2] - points[last][2];
						if ((dx * dx) + (dy * dy) + (dz * dz) < 1.0f)
						{
							out_winding->count = last;
						}
					}

					if (out_winding->count < 3)
					{
						out_winding->count = 0;
					}
				}

				// ------------------------------------------------------------------
				// cmodel ownership (script brushmodels are excluded from the world)
				// ------------------------------------------------------------------
				void collect_cmodel_brushes_r(clipMap_t* asset, unsigned int node_index,
					std::unordered_set<unsigned int>& owned, std::unordered_set<unsigned int>& visited)
				{
					const auto& tree = asset->info.bCollisionTree;
					if (node_index >= tree.leafbrushNodesCount || !visited.insert(node_index).second)
					{
						return;
					}

					const auto* node = &tree.leafbrushNodes[node_index];
					if (node->leafBrushCount > 0)
					{
						for (auto o = 0; o < node->leafBrushCount; o++)
						{
							owned.insert(node->data.leaf.brushes[o]);
						}
						return;
					}

					if (node->data.children.childOffset[0])
					{
						collect_cmodel_brushes_r(asset, node_index + node->data.children.childOffset[0], owned, visited);
					}
					if (node->data.children.childOffset[1])
					{
						collect_cmodel_brushes_r(asset, node_index + node->data.children.childOffset[1], owned, visited);
					}
				}

				void collect_cmodel_partitions_r(clipMap_t* asset, int tree_index,
					std::unordered_set<unsigned int>& owned, std::unordered_set<int>& visited)
				{
					const auto& tree = asset->info.pCollisionTree;
					if (tree_index < 0 || tree_index >= tree.aabbTreeCount || !visited.insert(tree_index).second)
					{
						return;
					}

					const auto* node = &tree.aabbTrees[tree_index];
					if (node->childCount != 0)
					{
						for (auto i = 0u; i < node->childCount; i++)
						{
							collect_cmodel_partitions_r(asset, node->u.firstChildIndex + i, owned, visited);
						}
						return;
					}

					owned.insert(static_cast<unsigned int>(node->u.partitionIndex));
				}

				// ------------------------------------------------------------------
				// H1 surface type -> IW7 physics material name
				// ------------------------------------------------------------------
				const char* surface_type_to_iw7(unsigned int type)
				{
					// IW7 retail set: Concrete, Plaster, Glass, Rock, Metal_Solid,
					// Metal_Sheet, Wood, Brick, Dirt, Plastic, Grass, Cloth,
					// Foliage, Cushion, Rubber
					static const char* const table[] = {
						"Concrete",    // 0x00 DEFAULT
						"Wood",        // 0x01 BARK
						"Brick",       // 0x02 BRICK
						"Cloth",       // 0x03 CARPET
						"Cloth",       // 0x04 CLOTH
						"Concrete",    // 0x05 CONCRETE
						"Dirt",        // 0x06 DIRT
						"Cushion",     // 0x07 FLESH
						"Foliage",     // 0x08 FOLIAGE_DEBRIS
						"Glass",       // 0x09 GLASS
						"Grass",       // 0x0A GRASS
						"Rock",        // 0x0B GRAVEL
						"Concrete",    // 0x0C ICE
						"Metal_Solid", // 0x0D METAL_SOLID
						"Metal_Sheet", // 0x0E METAL_GRATE
						"Dirt",        // 0x0F MUD
						"Cloth",       // 0x10 PAPER
						"Plaster",     // 0x11 PLASTER
						"Rock",        // 0x12 ROCK
						"Dirt",        // 0x13 SAND
						"Dirt",        // 0x14 SNOW
						"Concrete",    // 0x15 WATER_WAIST
						"Wood",        // 0x16 WOOD_SOLID
						"Concrete",    // 0x17 ASPHALT
						"Plaster",     // 0x18 CERAMIC
						"Plastic",     // 0x19 PLASTIC_SOLID
						"Rubber",      // 0x1A RUBBER
						"Foliage",     // 0x1B FRUIT
						"Metal_Solid", // 0x1C PAINTEDMETAL
						"Plastic",     // 0x1D RIOTSHIELD
						"Dirt",        // 0x1E SLUSH
						"Concrete",    // 0x1F ASPHALT_WET
						"Concrete",    // 0x20 ASPHALT_DEBRIS
						"Concrete",    // 0x21 CONCRETE_WET
						"Concrete",    // 0x22 CONCRETE_DEBRIS
						"Foliage",     // 0x23 FOLIAGE_VEGETATION
						"Foliage",     // 0x24 FOLIAGE_LEAVES
						"Grass",       // 0x25 GRASS_TALL
						"Metal_Sheet", // 0x26 METAL_HOLLOW
						"Metal_Sheet", // 0x27 METAL_VEHICLE
						"Metal_Sheet", // 0x28 METAL_THIN
						"Metal_Sheet", // 0x29 METAL_WET
						"Metal_Sheet", // 0x2A METAL_DEBRIS
						"Plastic",     // 0x2B PLASTIC_HOLLOW
						"Plastic",     // 0x2C PLASTIC_TARP
						"Rock",        // 0x2D ROCK_WET
						"Rock",        // 0x2E ROCK_DEBRIS
						"Concrete",    // 0x2F WATER_ANKLE
						"Concrete",    // 0x30 WATER_KNEE
						"Concrete",    // 0x31 WATER_HOLLOW
						"Wood",        // 0x32 WOOD_HOLLOW
						"Wood",        // 0x33 WOOD_DEBRIS
						"Cushion",     // 0x34 CUSHION
					};

					if (type < std::extent_v<decltype(table)>)
					{
						return table[type];
					}
					return "Concrete";
				}

				std::string map_material(const ClipMaterial* material)
				{
					const auto surface_flags = static_cast<unsigned int>(material->surfaceFlags);
					const auto type = (surface_flags >> 20) & 0xFF;

					if (type != 0 && type < 0x35)
					{
						return surface_type_to_iw7(type);
					}

					if (material->name)
					{
						auto name = std::string(material->name);
						std::transform(name.begin(), name.end(), name.begin(),
							[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

						static const std::pair<const char*, const char*> hints[] = {
							{ "glass", "Glass" }, { "metal", "Metal_Solid" }, { "grate", "Metal_Sheet" },
							{ "wood", "Wood" }, { "brick", "Brick" }, { "rock", "Rock" }, { "stone", "Rock" },
							{ "dirt", "Dirt" }, { "mud", "Dirt" }, { "sand", "Dirt" }, { "grass", "Grass" },
							{ "concrete", "Concrete" }, { "plaster", "Plaster" }, { "cloth", "Cloth" },
							{ "carpet", "Cloth" }, { "rubber", "Rubber" }, { "plastic", "Plastic" },
							{ "foliage", "Foliage" }, { "cushion", "Cushion" }, { "flesh", "Cushion" },
						};

						for (const auto& [hint, pm] : hints)
						{
							if (name.find(hint) != std::string::npos)
							{
								return pm;
							}
						}
					}

					return surface_type_to_iw7(type);
				}

				// ------------------------------------------------------------------
				// triangle accumulation
				// ------------------------------------------------------------------
				struct tri_key
				{
					std::int32_t v[9];

					bool operator==(const tri_key& other) const
					{
						return std::memcmp(this->v, other.v, sizeof(this->v)) == 0;
					}
				};

				struct tri_key_hash
				{
					std::size_t operator()(const tri_key& k) const noexcept
					{
						std::size_t h = 1469598103934665603ull;
						const auto* p = reinterpret_cast<const std::uint8_t*>(k.v);
						for (std::size_t i = 0; i < sizeof(k.v); i++)
						{
							h = (h ^ p[i]) * 1099511628211ull;
						}
						return h;
					}
				};

				struct soup
				{
					std::vector<hk::triangle> triangles;
					std::vector<hk::tag_entry> tags;
					std::unordered_map<std::string, unsigned int> tag_lookup;
					std::unordered_set<tri_key, tri_key_hash> seen;

					unsigned int tag_index(const std::string& pm_name, int contents)
					{
						const auto cfi = static_cast<unsigned int>(contents & CONTENTS_FILTER_MASK);
						const auto key = pm_name + "/" + std::to_string(cfi);
						const auto it = this->tag_lookup.find(key);
						if (it != this->tag_lookup.end())
						{
							return it->second;
						}

						const auto index = static_cast<unsigned int>(this->tags.size());
						this->tag_lookup.emplace(key, index);
						this->tags.push_back({ cfi, hk::material_crc(pm_name), 0xFFFF, 0 });
						return index;
					}

					bool add(const float* a, const float* b, const float* c, unsigned int tag)
					{
						tri_key key{};
						const float* src[3] = { a, b, c };
						for (auto i = 0; i < 3; i++)
						{
							for (auto o = 0; o < 3; o++)
							{
								key.v[i * 3 + o] = static_cast<std::int32_t>(std::lround(src[i][o] * 1000.0f));
							}
						}
						if (!this->seen.insert(key).second)
						{
							return false;
						}

						hk::triangle t{};
						std::memcpy(t.v[0], a, sizeof(float[3]));
						std::memcpy(t.v[1], b, sizeof(float[3]));
						std::memcpy(t.v[2], c, sizeof(float[3]));
						t.tag_index = tag;
						this->triangles.push_back(t);
						return true;
					}
				};

				// 6 box faces of a pure-AABB brush, wound CCW as seen from outside
				// (matches havok's convention, so no further flip is needed).
				// side order matches get_axial_planes: 0=-X 1=+X 2=-Y 3=+Y 4=-Z 5=+Z
				void add_box_brush(soup& out, const Bounds* bounds, const cbrush_t* brush,
					int contents, const ClipInfo* info, unsigned int* tri_count)
				{
					const float lo[3] = {
						bounds->midPoint[0] - bounds->halfSize[0],
						bounds->midPoint[1] - bounds->halfSize[1],
						bounds->midPoint[2] - bounds->halfSize[2] };
					const float hi[3] = {
						bounds->midPoint[0] + bounds->halfSize[0],
						bounds->midPoint[1] + bounds->halfSize[1],
						bounds->midPoint[2] + bounds->halfSize[2] };

					const float quads[6][4][3] = {
						{ { lo[0], lo[1], lo[2] }, { lo[0], lo[1], hi[2] }, { lo[0], hi[1], hi[2] }, { lo[0], hi[1], lo[2] } },
						{ { hi[0], lo[1], lo[2] }, { hi[0], hi[1], lo[2] }, { hi[0], hi[1], hi[2] }, { hi[0], lo[1], hi[2] } },
						{ { lo[0], lo[1], lo[2] }, { hi[0], lo[1], lo[2] }, { hi[0], lo[1], hi[2] }, { lo[0], lo[1], hi[2] } },
						{ { lo[0], hi[1], lo[2] }, { lo[0], hi[1], hi[2] }, { hi[0], hi[1], hi[2] }, { hi[0], hi[1], lo[2] } },
						{ { lo[0], lo[1], lo[2] }, { lo[0], hi[1], lo[2] }, { hi[0], hi[1], lo[2] }, { hi[0], lo[1], lo[2] } },
						{ { lo[0], lo[1], hi[2] }, { hi[0], lo[1], hi[2] }, { hi[0], hi[1], hi[2] }, { lo[0], hi[1], hi[2] } },
					};

					for (auto side = 0; side < 6; side++)
					{
						const auto index = 3 * (side & 1) + (side >> 1);
						const auto material_num = static_cast<unsigned short>(brush->axialMaterialNum[0][index]);

						auto pm = std::string("Concrete");
						if (material_num != 0xFFFF && info->materials && material_num < info->numMaterials)
						{
							pm = map_material(&info->materials[material_num]);
						}
						const auto tag = out.tag_index(pm, contents);

						if (out.add(quads[side][0], quads[side][1], quads[side][2], tag))
						{
							(*tri_count)++;
						}
						if (out.add(quads[side][0], quads[side][2], quads[side][3], tag))
						{
							(*tri_count)++;
						}
					}
				}
			}

			std::string generate(clipMap_t* asset, stats* out_stats)
			{
				stats st{};

				if (!asset)
				{
					return {};
				}

				// NOT asset->pInfo. H1's Load_clipMap_t ends with
				//   Load_ClipInfo(1);
				//   Load_ClipInfoFixup(t37, (char *)&cm + 0x10);
				// i.e. pInfo is rewritten to point at the GLOBAL cm clipMap's
				// ClipInfo, not this asset's. At db_link_x_asset_entry time cm is
				// still zeroed, so pInfo->planes is null and every
				// info->planes[planeIndex] deref faults (observed: plane =
				// 0x1AA18 == null + 5454 * sizeof(cplane_s)).
				// The asset's own info is also the one bdata/pdata come from, so
				// brush->sides[..].planeIndex only indexes it consistently.
				const auto* info = &asset->info;
				if (!info->planes || info->planeCount <= 0)
				{
					ZONETOOL_ERROR("clipmap \"%s\" has no collision planes, skipping world collision generation",
						asset->name ? asset->name : "null");
					return {};
				}

				const auto& bdata = asset->info.bCollisionData;
				const auto& pdata = asset->info.pCollisionData;

				// ---- cmodel (script brushmodel) ownership ----
				std::unordered_set<unsigned int> owned_brushes;
				std::unordered_set<unsigned int> owned_partitions;
				{
					std::unordered_set<unsigned int> visited_nodes;
					std::unordered_set<int> visited_trees;
					for (auto i = 0u; i < asset->numSubModels; i++)
					{
						const auto* cmodel = &asset->cmodels[i];
						collect_cmodel_brushes_r(asset, static_cast<unsigned int>(cmodel->leaf.leafBrushNode),
							owned_brushes, visited_nodes);
						for (auto o = 0u; o < cmodel->leaf.collAabbCount; o++)
						{
							collect_cmodel_partitions_r(asset, static_cast<int>(cmodel->leaf.firstCollAabbIndex + o),
								owned_partitions, visited_trees);
						}
					}
				}

				// partition -> material, from every aabb-tree leaf
				std::unordered_map<unsigned int, unsigned short> partition_material;
				for (auto i = 0; i < asset->info.pCollisionTree.aabbTreeCount; i++)
				{
					const auto* node = &asset->info.pCollisionTree.aabbTrees[i];
					if (node->childCount == 0)
					{
						partition_material[static_cast<unsigned int>(node->u.partitionIndex)] = node->materialIndex;
					}
				}

				st.brush_count = bdata.numBrushes;
				st.cmodel_brush_count = static_cast<unsigned int>(owned_brushes.size());
				st.partition_count = static_cast<unsigned int>(pdata.partitionCount);
				st.cmodel_partition_count = static_cast<unsigned int>(owned_partitions.size());

				soup out{};
				out.triangles.reserve(bdata.numBrushes * 12 + pdata.triCount);

				const auto resolve_material = [&](unsigned short material_num) -> std::string
				{
					if (material_num == 0xFFFF || !info->materials || material_num >= info->numMaterials)
					{
						return "Concrete";
					}
					return map_material(&info->materials[material_num]);
				};

				// ---- brushes ----
				auto winding = std::make_unique<winding_t>();
				for (auto bi = 0u; bi < bdata.numBrushes; bi++)
				{
					if (owned_brushes.contains(bi))
					{
						continue;
					}

					const auto* brush = &bdata.brushes[bi];
					const auto contents = bdata.brushContents ? bdata.brushContents[bi] : CONTENTS_SOLID;

					if ((contents & PLAYER_SOLID_MASK) == 0)
					{
						st.skipped_nonsolid++;
						continue;
					}

					const auto* bounds = &bdata.brushBounds[bi];

					if (brush->numsides == 0 || !brush->sides || !brush->baseAdjacentSide)
					{
						st.box_brushes++;
						add_box_brush(out, bounds, brush, contents, info, &st.brush_tris);
						continue;
					}

					const auto axial_planes = get_axial_planes(bounds);
					for (auto side_index = 0; side_index < brush->numsides + 6; side_index++)
					{
						get_winding_for_brush_face(info, brush, side_index, winding.get(), &axial_planes);
						if (winding->count < 3)
						{
							continue;
						}

						unsigned short material_num;
						if (side_index >= 6)
						{
							material_num = brush->sides[side_index - 6].materialNum;
						}
						else
						{
							const auto index = 3 * (side_index & 1) + (side_index >> 1);
							material_num = static_cast<unsigned short>(brush->axialMaterialNum[0][index]);
						}

						const auto tag = out.tag_index(resolve_material(material_num), contents);

						// fan triangulation with REVERSED winding
						for (auto k = 1u; k + 1 < winding->count; k++)
						{
							if (out.add(winding->pts[k + 1], winding->pts[k], winding->pts[0], tag))
							{
								st.brush_tris++;
							}
						}
					}
				}

				// ---- terrain / patch partitions ----
				for (auto pi = 0; pi < pdata.partitionCount; pi++)
				{
					if (owned_partitions.contains(static_cast<unsigned int>(pi)))
					{
						continue;
					}

					const auto* partition = &pdata.partitions[pi];
					const auto material_it = partition_material.find(static_cast<unsigned int>(pi));
					const auto material_num = material_it == partition_material.end()
						? static_cast<unsigned short>(0) : material_it->second;

					// terrain is plain walkable world geometry
					const auto tag = out.tag_index(resolve_material(material_num), CONTENTS_SOLID);
					const auto base = 1024u * partition->firstVertSegment;

					for (auto o = 0u; o < partition->triCount; o++)
					{
						const auto* indices = &pdata.triIndices[3 * (partition->firstTri + o)];
						const auto i0 = indices[0] + base;
						const auto i1 = indices[1] + base;
						const auto i2 = indices[2] + base;
						if (i0 >= pdata.vertCount || i1 >= pdata.vertCount || i2 >= pdata.vertCount)
						{
							continue;
						}

						// REVERSED winding
						if (out.add(pdata.verts[i2], pdata.verts[i1], pdata.verts[i0], tag))
						{
							st.terrain_tris++;
						}
					}
				}

				st.total_tris = static_cast<unsigned int>(out.triangles.size());
				st.tag_count = static_cast<unsigned int>(out.tags.size());

				if (out_stats)
				{
					*out_stats = st;
				}

				if (out.triangles.empty())
				{
					ZONETOOL_WARNING("no world collision geometry found for clipmap \"%s\"", asset->name);
					return {};
				}

				unsigned int shape_contents = 0;
				for (const auto& tag : out.tags)
				{
					shape_contents |= tag.collision_filter_info;
				}

				try
				{
					hk::build_stats bstats{};
					auto blob = hk::build_world_collision_blob(out.triangles, out.tags,
						"World Entity 0", shape_contents, &bstats);

					ZONETOOL_INFO(
						"world collision: %u triangles (%u brush, %u terrain) in %u sections, "
						"%u tags, %u shared verts (%u windows), domain (%.0f %.0f %.0f)..(%.0f %.0f %.0f), %zu bytes",
						bstats.triangle_count, st.brush_tris, st.terrain_tris, bstats.section_count,
						st.tag_count, bstats.shared_vertex_count, bstats.shared_vertex_windows,
						bstats.domain_min[0], bstats.domain_min[1], bstats.domain_min[2],
						bstats.domain_max[0], bstats.domain_max[1], bstats.domain_max[2],
						blob.size());

					return blob;
				}
				catch (const std::exception& e)
				{
					ZONETOOL_ERROR("failed to encode world collision for \"%s\": %s", asset->name, e.what());
					return {};
				}
			}
		}
	}
}
#endif
