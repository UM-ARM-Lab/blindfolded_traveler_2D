#include "beliefs/chs.hpp"
#include <gtest/gtest.h>

using namespace BTP;

TEST(ChsTest, voxel_grid_subtraction_gives_correct_overlap)
{
    VoxelGrid2D v1(0.1), v2(0.1);
    v1.set(0.4, 0.4, true);
    v1.set(0.2, 0.2, true);

    v2.set(0.4, 0.4, true);
    
    EXPECT_TRUE(v1.get(0.2, 0.2)) << "Voxel not properly set to true";
    EXPECT_TRUE(v1.get(0.4, 0.4)) << "Voxel not properly set to true";
    EXPECT_TRUE(v2.get(0.4, 0.4)) << "Voxel not properly set to true";

    EXPECT_EQ(v2.intersectFraction(v1), 1.0) << "Overlapping fraction not correct";
    EXPECT_EQ(v1.intersectFraction(v2), 0.5) << "Overlapping fraction not correct";

    v1.subtract(v2);

    EXPECT_TRUE(v1.get(0.2, 0.2)) << "Voxel improperly set to false after subtraction";
    EXPECT_FALSE(v1.get(0.4, 0.4)) << "Voxel not set to false after subtraction";

    EXPECT_EQ(v2.intersectFraction(v1), 0.0) << "Overlap not 0 after subtraction";
    EXPECT_EQ(v1.intersectFraction(v2), 0.0) << "Overlap not 0 after subtraction";
}



GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
