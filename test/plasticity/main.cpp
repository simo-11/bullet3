/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2014 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///Original author: Erwin Coumans, October 2014
/// For Plasticity Simo Nikula, January 2017

#include <gtest/gtest.h>

#include "CharpyDemo/btElasticPlasticConstraint.h"

/** http://www.binaryhexconverter.com/binary-to-hex-converter */
TEST(btElasticPlasticConstraint, countChanges) {
	int extra = (sizeof(velDirType) > 1 ? 1 : 0);
	ASSERT_EQ(0, btElasticPlasticConstraint::countChanges(0x0));  // 00000000
	ASSERT_EQ(0+extra, btElasticPlasticConstraint::countChanges(0xff)); // 11111111
	ASSERT_EQ(1 + extra, btElasticPlasticConstraint::countChanges(0xfe)); // 11111110
	ASSERT_EQ(1 + extra, btElasticPlasticConstraint::countChanges(0xfc)); // 11111100
	ASSERT_EQ(1 + extra, btElasticPlasticConstraint::countChanges(0xf0)); // 11110000
	ASSERT_EQ(2 + extra, btElasticPlasticConstraint::countChanges(0xc7)); // 11000111
	ASSERT_EQ(2, btElasticPlasticConstraint::countChanges(0x1e)); // 00011110
	ASSERT_EQ(3 + extra, btElasticPlasticConstraint::countChanges(0x8e)); // 10001110
	ASSERT_EQ(4 + extra, btElasticPlasticConstraint::countChanges(0x99)); // 10011001
	ASSERT_EQ(7 + extra, btElasticPlasticConstraint::countChanges(0xaa)); // 10101010
}

int main(int argc, char **argv) {
#if _MSC_VER
        _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
        //void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
