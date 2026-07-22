#pragma once

namespace zonetool::iw7
{
	namespace havok
	{
		namespace encode
		{
			// One world-collision triangle. Winding must already be in the
			// convention the game expects (see the H1 extractor - H1 brush
			// windings are emitted reversed).
			struct triangle
			{
				float v[3][3];
				unsigned int tag_index; // index into the tag table
			};

			// One HavokPhysicsShapeList::shapeTagData entry (24 bytes on disk).
			struct tag_entry
			{
				unsigned int collision_filter_info;
				unsigned int material_crc;
				unsigned short material_id;
				unsigned long long user_data;
			};

			struct build_stats
			{
				unsigned int triangle_count;
				unsigned int section_count;
				unsigned int shared_vertex_count;
				unsigned int shared_vertex_windows;
				unsigned int top_node_count;
				unsigned int bits_per_key;
				float domain_min[3];
				float domain_max[3];
			};

			// materialCRC = crc32("PM_" + name-without-PM_-prefix)
			unsigned int material_crc(const std::string& name);

			// Builds a complete hk_2014.2.5-r1 binary packfile containing
			//   HavokPhysicsShapeList -> hknpCompressedMeshShape -> hknpCompressedMeshShapeData
			// Throws std::runtime_error when the input cannot be encoded.
			std::string build_world_collision_blob(const std::vector<triangle>& tris,
				const std::vector<tag_entry>& tags, const std::string& shape_name,
				unsigned int shape_contents, build_stats* stats = nullptr);

			// Packfile holding a HavokPhysicsShapeList with every array empty.
			// MapEnts::havokEntsShapeData must not be null: the game registers it
			// (sub_140572320) into the global qword_144BDF460 and then reads
			// list+0x78 / list+0x80 unconditionally to resolve a brushmodel's
			// contents (sub_140B7B220, reached from SV_LinkEntity for every
			// brushmodel entity). A null list is an access violation; an empty one
			// fails the bounds check and returns the default contents 0xFDFFBFFF.
			std::string build_empty_shape_list_blob();
		}
	}
}
