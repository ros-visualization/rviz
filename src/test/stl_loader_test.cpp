#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <rviz/ogre_helpers/stl_loader.h>

TEST( STLLoader, load )
{
  // Get the path to the directory where the meshes are located.
  boost::filesystem::path meshDir = ros::package::getPath("rviz");
  meshDir /= "src/test/meshes";

  // Load an ascii STL file. Note that only binary STL files are supported.
  ogre_tools::STLLoader loader;
  boost::filesystem::path meshFilePath = meshDir / "ascii.stl";
  EXPECT_FALSE(loader.load(meshFilePath.string()));

  // Load an invalid STL binary file (size < 84 bytes).
  meshFilePath = meshDir / "invalid_short.stl";
  EXPECT_FALSE(loader.load(meshFilePath.string()));

  // Load an invalid STL binary file (size does not match the expected size).
  meshFilePath = meshDir / "invalid.stl";
  EXPECT_FALSE(loader.load(meshFilePath.string()));

  // Load an invalid STL binary file (size does not match the expected size,
  // but does if incorrectly read as an 16-bit uint)
  meshFilePath = meshDir / "16bit_vs_32bit_should_fail.stl";
  EXPECT_FALSE(loader.load(meshFilePath.string()));

  // Load a valid STL binary file.
  meshFilePath = meshDir / "valid.stl";
  EXPECT_TRUE(loader.load(meshFilePath.string()));
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
