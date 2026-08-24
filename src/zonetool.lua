zonetool = {}
function zonetool:project()
    project "zonetool"
		kind "ConsoleApp"
		language "C++"

		targetname "zonetool"

		pchheader "std_include.hpp"
		pchsource "src/zonetool/std_include.cpp"

		linkoptions {"/IGNORE:4254", "/DYNAMICBASE:NO", "/SAFESEH:NO", "/LARGEADDRESSAWARE", "/LAST:.main", "/PDBCompress"}

		files {
			"./src/zonetool/**.rc", 
			"./src/zonetool/**.hpp", 
			"./src/zonetool/**.cpp", 
			"./src/zonetool/resources/**.*"
		}

		includedirs {
			"./src", 
			"./src/zonetool", 
			"./src/common", 
			"%{prj.location}/src"
		}

		resincludedirs {"$(ProjectDir)src"}

		dependson {"tlsdll"}

		links {"common"}

		prebuildcommands {"pushd %{_MAIN_SCRIPT_DIR}", "tools\\premake5 generate-buildinfo", "popd"}

		local function addcopyopt(name)
			if (_OPTIONS[name] ~= nil) then
				postbuildcommands {"copy /y \"$(TargetPath)\" \"" .. _OPTIONS[name] .. "\""}
			end
		end

		addcopyopt("copy-to")
		addcopyopt("iw6-copy-to")
		addcopyopt("s1-copy-to")
		addcopyopt("h1-copy-to")
		addcopyopt("h2-copy-to")
		addcopyopt("iw7-copy-to")
		
		dependencies.imports()
end