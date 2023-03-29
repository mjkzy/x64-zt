#pragma once

namespace zonetool::s1
{
	namespace converter::h1
	{
		namespace hullshader
		{
			zonetool::h1::MaterialHullShader* convert(MaterialHullShader* asset, ZoneMemory* mem);
			void dump(MaterialHullShader* asset, ZoneMemory* mem);
		}
	}
}