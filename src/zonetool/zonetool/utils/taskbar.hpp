#pragma once

namespace zonetool::taskbar
{
	void set_progress(const std::uint64_t completed, const std::uint64_t total);
	void set_indeterminate();
	void set_error();
	void clear();
}
