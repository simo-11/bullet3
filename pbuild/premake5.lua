-- Bullet-ep (elastic-plastic) extensions premake5 script.
-- https://premake.github.io/

workspace "0_Bullet-EP"
	location ( "Build/" .. _ACTION )
	architecture "x86_64"
	configurations { "Debug", "Release" }
	
	configuration "vs*"
		defines { "_CRT_SECURE_NO_WARNINGS",
		"DONT_USE_GLUT",
		"GWEN_COMPILE_STATIC"
		}	
		
	filter "configurations:Debug"
		targetdir ( "Build/" .. _ACTION .. "/bin/Debug" )
	 	defines { "DEBUG","_DEBUG=1","CDBG_CALLBACK" }
		symbols "On"

	filter "configurations:Release"
		targetdir ( "Build/" .. _ACTION .. "/bin/Release" )
		defines { "NDEBUG" }
		optimize "On"

project "ep"
	kind "ConsoleApp"
	language "C++"
	files { 
	"../examples/plasticity/**.h", 
	"../examples/plasticity/**.cpp",
	"../examples/ExampleBrowser/**.h", 
	"../examples/ExampleBrowser/**.cpp",
	"../examples/CommonInterfaces/*", 
	"../examples/CharpyDemo/*", 
	"../examples/Utils/b3Clock.*",
	"../examples/OpenGLWindow/*.c",
	"../examples/OpenGLWindow/*.h",
	"../examples/OpenGLWindow/GL/*.h",
	"../examples/ThirdPartyLibs/Gwen/**.cpp",
	"../examples/ThirdPartyLibs/Gwen/**.h",
	"../src/BulletCollision/**.h",
	"../src/BulletCollision/**.cpp",
	"../src/BulletDynamics/**.h",
	"../src/BulletDynamics/**.cpp",	
	"../src/LinearMath/**.h",
	"../src/LinearMath/**.cpp",	
	}
	excludes {
	"../examples/ExampleBrowser/OpenGLExampleBrowser.*",
	"../examples/ExampleBrowser/ExampleEntries.*",
	"../examples/ExampleBrowser/InProcessExampleBrowser.*",
	"../examples/ExampleBrowser/main.*",
	"../examples/RenderingExamples/TinyRenderer*",
	"../examples/RenderingExamples/TinyVRGui*",
	"../examples/RenderingExamples/DynamicTexturedCubeDemo*",
	"../examples/OpenGLWindow/X11OpenGLWindow*",
	"../examples/OpenGLWindow/MacOpenGLWindow*",
	}
	
	includedirs { ".",
	"../src",
	}
	configuration { "windows" }
		links { "glu32", "opengl32", "winmm" }