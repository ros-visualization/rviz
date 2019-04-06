#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <rviz/mesh_loader.h>
#include <Ogre.h>

TEST( STLLoader, load )
{
  std::unique_ptr<Ogre::Root> root(new Ogre::Root());

  // Get the path to the directory where the meshes are located.
  boost::filesystem::path meshDir = ros::package::getPath("rviz");
  meshDir /= "src/test/meshes";

  // Load an ascii STL file. Note that only binary STL files are supported.
  boost::filesystem::path meshFilePath = meshDir / "ascii.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());

  // Load an invalid STL binary file (size < 84 bytes).
  meshFilePath = meshDir / "invalid_short.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());

  // Load an invalid STL binary file (size does not match the expected size).
  meshFilePath = meshDir / "invalid.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());

  // Load an invalid STL binary file (size does not match the expected size,
  // but does if incorrectly read as an 16-bit uint)
  meshFilePath = meshDir / "16bit_vs_32bit_should_fail.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());

  // Load a valid STL binary file.
  meshFilePath = meshDir / "valid.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());

  // Load a "potentially" valid STL binary file with bigger size than the
  // expected. The extra "unexpected" data at the end of the file should be
  // ignored.
  meshFilePath = meshDir / "valid_extra.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath.string()).isNull());
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
