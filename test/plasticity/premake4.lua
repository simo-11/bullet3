
	project "Test_BulletPlasticity"
		
	kind "ConsoleApp"
	
--	defines {  }
	

	
	includedirs 
	{
		".",
		"../../src",
		"../../examples",
		"../gtest-1.7.0/include"
	
	}


	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end
	
	links {"LinearMath", "gtest"}
	
	files {
		"**.cpp",
		"**.h",
		"../../examples/plasticity/PlasticityUtils.cpp",
		"../../examples/plasticity/PlasticityUtils.h",
	}

	if os.is("Linux") then
                links {"pthread"}
        end

