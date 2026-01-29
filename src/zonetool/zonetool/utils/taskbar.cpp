#include <std_include.hpp>

#include "taskbar.hpp"

#include <shobjidl_core.h>

namespace zonetool::taskbar
{
	namespace
	{
		ITaskbarList3* get_taskbar_list()
		{
			static auto task_list = []()
			{
				ITaskbarList3* ptr{};
				[[ maybe_unused ]] const auto init = CoInitialize(NULL);
				[[ maybe_unused ]] const auto result =
					CoCreateInstance(CLSID_TaskbarList, NULL, CLSCTX_INPROC_SERVER, IID_ITaskbarList, reinterpret_cast<LPVOID*>(&ptr));
				return ptr;
			}();

			return task_list;
		}
	}

	void set_progress(const std::uint64_t completed, const std::uint64_t total)
	{
		const auto task_list = get_taskbar_list();
		if (task_list == NULL)
		{
			return;
		}

		task_list->SetProgressState(GetConsoleWindow(), TBPF_NORMAL);
		task_list->SetProgressValue(GetConsoleWindow(), completed, total);
	}

	void set_indeterminate()
	{
		const auto task_list = get_taskbar_list();
		if (task_list == NULL)
		{
			return;
		}

		task_list->SetProgressState(GetConsoleWindow(), TBPF_INDETERMINATE);
	}

	void set_error()
	{
		const auto task_list = get_taskbar_list();
		if (task_list == NULL)
		{
			return;
		}

		task_list->SetProgressState(GetConsoleWindow(), TBPF_ERROR);
	}

	void clear()
	{
		const auto task_list = get_taskbar_list();
		if (task_list == NULL)
		{
			return;
		}

		task_list->SetProgressState(GetConsoleWindow(), TBPF_NOPROGRESS);
		task_list->SetProgressValue(GetConsoleWindow(), 0ull, 0ull);
	}
}
