#pragma once

namespace zonetool::iw8
{
	namespace converter::h1
	{
		namespace gfxworld
		{
			zonetool::h1::GfxWorld* convert(GfxWorld* asset, utils::memory::allocator& allocator);
			void dump(GfxWorld* asset);
		}
	}
}
