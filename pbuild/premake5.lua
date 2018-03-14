-- Bullet-ep (elastic-plastic) extensions premake5 script.
-- https://premake.github.io/

workspace "0_Bullet-EP"
	location ( _ACTION )
	architecture "x86_64"
	configurations { "Debug", "Release" }
	
	configuration "vs*"
		defines { "_CRT_SECURE_NO_WARNINGS",
		"DONT_USE_GLUT",
		"GLEW_STATIC",
		"GWEN_COMPILE_STATIC",
		"_VARIADIC_MAX=10"
		}
		flags {
		"MultiProcessorCompile"
		}
		
	filter "configurations:Debug"
		targetdir ( _ACTION .. "/bin/Debug" )
	 	defines { "DEBUG","_DEBUG=1","CDBG_CALLBACK" }
		symbols "On"

	filter "configurations:Release"
		targetdir (  _ACTION .. "/bin/Release" )
		defines { "NDEBUG" }
		optimize "On"

project "0_ep"
	kind "ConsoleApp"
	language "C++"
	files {
	"../examples/plasticity/main.cpp",
	}
	includedirs { ".",
	"../src",
	}
	links {
	"BLIB"
	}
	configuration { "windows" }
		links { "glu32", "opengl32"}	

project "0_ep-test"
	kind "ConsoleApp"
	language "C++"
	files {
	"../test/plasticity/main.cpp",
	}
	includedirs {
	"../src",
	"../test/gtest-1.7.0/include",
	"../examples"
	}
	links {
	"BLIB"
	}
		
		
project "BLIB"
	kind "StaticLib"
	language "C++"		
	files { 
	"../examples/plasticity/**.h", 
	"../examples/plasticity/**.cpp",
	"../examples/ExampleBrowser/**.h", 
	"../examples/ExampleBrowser/**.cpp",
	"../examples/CommonInterfaces/*", 
	"../examples/RenderingExamples/*",
	"../examples/CharpyDemo/*", 
	"../examples/Utils/b3Clock.*",
	"../examples/OpenGLWindow/*.cpp",
	"../examples/OpenGLWindow/*.h",
	"../examples/OpenGLWindow/GL/*.h",
	"../examples/ThirdPartyLibs/Gwen/**.cpp",
	"../examples/ThirdPartyLibs/Gwen/**.h",
	"../examples/ThirdPartyLibs/glad/glad.c",
	"../examples/ThirdPartyLibs/glad/glad/glad.h",
	"../src/BulletCollision/**.h",
	"../src/BulletCollision/**.cpp",
	"../src/BulletDynamics/**.h",
	"../src/BulletDynamics/**.cpp",	
	"../src/LinearMath/**.h",
	"../src/LinearMath/**.cpp",	
	"../src/Bullet3Common/**.h",
	"../src/Bullet3Common/**.cpp",	
	"../test/gtest-1.7.0/src/gtest-all.cc",	
	}
	removefiles {
	"../examples/ExampleBrowser/OpenGLExampleBrowser.*",
	"../examples/ExampleBrowser/ExampleEntries.*",
	"../examples/ExampleBrowser/InProcessExampleBrowser.*",
	"../examples/*/main.*",
	"../examples/RenderingExamples/TinyRenderer*",
	"../examples/RenderingExamples/TinyVRGui*",
	"../examples/RenderingExamples/DynamicTexturedCubeDemo*",
	"../examples/OpenGLWindow/X11OpenGLWindow*",
	"../examples/OpenGLWindow/MacOpenGLWindow*",
	}
	includedirs { ".",
	"../src",
	"../examples/ThirdPartyLibs",
	"../examples/ThirdPartyLibs/glad",
	"../test/gtest-1.7.0/include",	
	"../test/gtest-1.7.0",	
	}
	configuration { "windows" }
		links { "glu32", "opengl32", "winmm" }