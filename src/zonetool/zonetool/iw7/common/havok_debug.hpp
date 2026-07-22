#pragma once

namespace zonetool::iw7
{
	namespace havok
	{
		namespace debug
		{
			// Parses an IW7 havok binary packfile (hk_2014.2.5-r1, fileVersion 11)
			// and writes a human-readable report next to the blob:
			//   <path>.txt  - header, sections, fixups, classnames, shapeTagData,
			//                 compressed-mesh tree layout, per-section table,
			//                 tree/AABB validation, triangle z-histogram
			//   <path>.obj  - decoded world-collision triangles (Blender/etc)
			void dump_readable(const std::string& hkx_path, const char* data, unsigned int size);

			// One-line ZONETOOL_INFO summary of a blob (root class, domain,
			// section/primitive/simd-node counts). Cheap; used on zone build.
			void log_summary(const std::string& hkx_path, const char* data, unsigned int size);
		}
	}
}
