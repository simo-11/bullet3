	
		project "App_PlasticityExampleBrowser"

		language "C++"
				
		kind "ConsoleApp"

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }

	if _OPTIONS["lua"] then
		includedirs{"../ThirdPartyLibs/lua-5.2.3/src"}
		links {"lua-5.2.3"}
		defines {"ENABLE_LUA"}
		files {"../LuaDemo/LuaPhysicsSetup.cpp"}
	end

			
		links{"gwen", "OpenGL_Window", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}
		initOpenGL()
		initGlew()

		files {
		"**.cpp",
		"**.h",
		"../CommonInterfaces/*",
		"../CharpyDemo/*",
		"../Utils/b3Clock.*",

		}
		
if os.is("Linux") then 
	initX11()
end

if os.is("MacOSX") then
	links{"Cocoa.framework"}
end

			
