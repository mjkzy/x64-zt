#pragma once

namespace zonetool::h1
{
	namespace converter::iw7
	{
		namespace world_collision
		{
			struct stats
			{
				unsigned int brush_count;
				unsigned int cmodel_brush_count;
				unsigned int skipped_nonsolid;
				unsigned int box_brushes;
				unsigned int partition_count;
				unsigned int cmodel_partition_count;
				unsigned int brush_tris;
				unsigned int terrain_tris;
				unsigned int total_tris;
				unsigned int tag_count;
			};

			// Reconstructs the H1 world collision (brush windings + terrain
			// partitions) and encodes it as an IW7 havok world-collision
			// packfile. Returns an empty string when nothing collidable was
			// found (or on encode failure - the error is logged).
			std::string generate(clipMap_t* asset, stats* out_stats);
		}
	}
}
