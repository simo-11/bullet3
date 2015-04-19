
  solution "0_BulletPlasticity"

	local osversion = os.getversion()
		print(string.format(" %d.%d.%d (%s)", 
   		osversion.majorversion, osversion.minorversion, osversion.revision,
   		osversion.description))
	

	-- Multithreaded compiling
	if _ACTION == "vs2010" or _ACTION=="vs2008" or _ACTION=="vs2013" then
		buildoptions { "/MP"  }
	end

	act = ""

    if _ACTION then
        act = _ACTION
    end

	configurations {"Release", "Debug","ReleaseDouble","DebugDouble"}
	configuration "Release"
		flags { "Optimize", "EnableSSE2","StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		defines {"_DEBUG=1"}
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}
	configuration "ReleaseDouble"
		defines {"BT_USE_DOUBLE_PRECISION"}
		flags { "Optimize", "EnableSSE2","StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "DebugDouble"
		defines {"_DEBUG=1","BT_USE_DOUBLE_PRECISION",
		"_CTR_SECURE_NO_WARNINGS","_CRT_SECURE_NO_DEPRECATE"}
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}

	if os.is("Linux") then
		if os.is64bit() then
			platforms {"x64"}
		else
			platforms {"x32"}
		end
	else
		platforms {"x32", "x64"}
	end

	configuration {"x32"}
		targetsuffix ("_" .. act)
	configuration "x64"
		targetsuffix ("_" .. act .. "_64" )
	configuration {"x64", "debug"}
		targetsuffix ("_" .. act .. "_x64_debug")
	configuration {"x64", "release"}
		targetsuffix ("_" .. act .. "_x64_release" )
	configuration {"x32", "debug"}
		targetsuffix ("_" .. act .. "_debug" )

	configuration{}

	postfix=""

	if _ACTION == "xcode4" then
			xcodebuildsettings
			{
        		'ARCHS = "$(ARCHS_STANDARD_32_BIT) $(ARCHS_STANDARD_64_BIT)"',
        		'VALID_ARCHS = "x86_64 i386"',
--			'SDKROOT = "macosx10.9"',
			}
	end

	targetdir "../bin"
	location("./" .. act .. postfix)


	projectRootDir = os.getcwd() .. "/../"
	print("Project root directory: " .. projectRootDir);

    dofile ("../build3/findOpenGLGlewGlut.lua")
	language "C++"

	include "../Demos/plasticity"
	include "../btgui/OpenGLWindow"
	include "../src/BulletSoftBody"
	include "../src/BulletDynamics"
	include "../src/BulletCollision"
	include "../src/LinearMath"


