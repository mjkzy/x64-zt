#include <std_include.hpp>
#include "loader/loader.hpp"
#include "loader/component_loader.hpp"

#include <utils/string.hpp>
#include <utils/flags.hpp>
#include <utils/io.hpp>

#include "game/mode.hpp"

#define IW8_BINARY "game_dx12_ship_replay.exe"

namespace
{
	DECLSPEC_NORETURN void WINAPI exit_hook(const int code)
	{
		component_loader::pre_destroy();
		exit(code);
	}

	BOOL WINAPI system_parameters_info_a(const UINT uiAction, const UINT uiParam, const PVOID pvParam, const UINT fWinIni)
	{
		component_loader::post_unpack();
		return SystemParametersInfoA(uiAction, uiParam, pvParam, fWinIni);
	}

	namespace iw8
	{
		DWORD_PTR WINAPI set_thread_affinity_mask(HANDLE hThread, DWORD_PTR dwThreadAffinityMask)
		{
			component_loader::post_unpack();
			return SetThreadAffinityMask(hThread, dwThreadAffinityMask);
		}

		FARPROC load_binary()
		{
			loader loader;
			utils::nt::library self;

			/*
			loader.set_import_resolver([self](const std::string& library, const std::string& function) -> void*
			{
				if (function == "ExitProcess")
				{
					return exit_hook;
				}
				else if (function == "SetThreadAffinityMask")
				{
					return set_thread_affinity_mask;
				}

				return component_loader::load_import(library, function);
			});
			*/

			std::string binary = IW8_BINARY;

			std::string data;
			if (!utils::io::read_file(binary, &data))
			{
				throw std::runtime_error(utils::string::va(
					"Failed to read game binary (%s)!\nPlease copy the iw8-zonetool.exe into your Call of Duty: MW2019 installation folder and run it from there.",
					binary.data()));
			}

			return loader.load_library(binary);
		}

		void remove_crash_file()
		{
			utils::io::remove_file("__game_dx12_ship_replay");
		}
	}

	void remove_crash_file()
	{
		return iw8::remove_crash_file();
	}

	void verify_version()
	{
		switch (game::get_mode())
		{
		case game::iw6:
			return iw6::verify_ghost_version();
		}
	}

	int main_internal()
	{
		FARPROC entry_point = nullptr;

		srand(std::uint32_t(std::time(nullptr)));
		remove_crash_file();

		{
			component_loader::sort();

			auto premature_shutdown = true;
			const auto _ = gsl::finally([&premature_shutdown]()
			{
				if (premature_shutdown)
				{
					component_loader::pre_destroy();
				}
			});

			try
			{
				if(utils::flags::has_flag("unbuffered-io"))
				{
					setvbuf(stdout, NULL, _IONBF, 0);
					setvbuf(stderr, NULL, _IONBF, 0);
				}
				
				if (!component_loader::post_start())
				{
					return 0;
				}

				entry_point = iw8::load_binary();
				if (!entry_point)
				{
					throw std::runtime_error("Unable to load binary into memory");
				}

				verify_version();

				if (!component_loader::post_load())
				{
					return 0;
				}

				premature_shutdown = false;
			}
			catch (std::exception& e)
			{
				MessageBoxA(nullptr, e.what(), "ERROR", MB_ICONERROR);
				return 1;
			}
		}

		return static_cast<int>(entry_point());
	}
}

int main()
{
	if (utils::io::file_exists(IW8_BINARY))
	{
		game::set_mode(game::game_mode::iw8);
	}
	else
	{
		MessageBoxA(nullptr, "no game binary found!\nmake sure your zonetool.exe is in the game directory!", "NO BINARY", MB_ICONERROR);
	}

	return main_internal();
}

int __stdcall WinMain(HINSTANCE, HINSTANCE, PSTR, int)
{
	return main();
}
