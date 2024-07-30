#include <std_include.hpp>
#include "loader/component_loader.hpp"

#include <utils/hook.hpp>

#include "zonetool/t7/zonetool.hpp"

namespace zonetool::t7
{
	namespace
	{
		void com_init_try_block_function_stub()
		{
			utils::hook::invoke<void>(0x140504170);

			zonetool::t7::start();

			while (1)
			{
				std::this_thread::sleep_for(5ms);
				utils::hook::invoke<void>(0x140501F90); // Com_Frame
			}
		}

		void load_gfximage_asset_stub(GfxImage** a1)
		{
			// Load_SimpleAsset
			utils::hook::invoke<void>(0x1401DB2B0, a1, ASSET_TYPE_IMAGE);
		}
	}

	class component final : public component_interface
	{
	public:
		void post_unpack() override
		{
			// forever loop after game initialization
			utils::hook::call(0x140502AA9, com_init_try_block_function_stub);

			// disable g_copyInfo shit ( freezes after dump, but at least wont crash )
			utils::hook::set<uint8_t>(0x1401D4FE0, 0xC3);
			utils::hook::set<uint8_t>(0x1401D86A0, 0xC3);

			// patch Load_XAsset nullsubs
			utils::hook::call(0x1401B1E5D, load_gfximage_asset_stub);

			zonetool::t7::initialize();
		}
	};
}

REGISTER_COMPONENT_T7(zonetool::t7::component)
