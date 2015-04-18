
  
    
function createDemos( demos, incdirs, linknames)
	for _, name in ipairs(demos) do
				
			project ( "App_" .. name )
			
			kind "ConsoleApp"
			targetdir "../.."

		configuration {}
	
	  	includedirs {incdirs}
		links {
                                linknames
                        }

		configuration { "Windows" }
			defines { "GLEW_STATIC"}
	 		links { "opengl32","glu32","winmm"}
			includedirs{	"Glut"	}
	 		libdirs {"../Glut"}
	 		files   { "../../build3/bullet.rc" }
	 		
	 		configuration {"Windows", "x32"}
				links {"glew32s","glut32"}
			configuration {"Windows", "x64"}
				links {"glew64s", "glut64"}

			
		configuration {"MacOSX"}
			--print "hello"
	 		linkoptions { "-framework Carbon -framework OpenGL -framework AGL -framework Glut" } 
		
		configuration {"not Windows", "not MacOSX"}
			links {"GL","GLU","glut","pthread"}
		configuration{}
	
		
		files     { 
		 	"./" .. name .. "/*.cpp" ,
		 	"./" .. name .. "/*.h"
		 }
	end
end

 local localdemos = {
    "CharpyDemo"
  }


createDemos(localdemos,{"../src","../OpenGL","../btgui",},{"OpenGLSupport","BulletDynamics", "BulletCollision", "LinearMath"})
include "../OpenGL"
 
 
