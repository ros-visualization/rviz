#include <gtest/gtest.h>
#include <rviz/mesh_loader.h>
#include <Ogre.h>

TEST( STLLoader, load )
{
  // Get the path to the directory where the meshes are located.
  std::string meshDir = "package://rviz/src/test/meshes/";

  // Load an ascii STL file. Note that only binary STL files are supported.
  std::string meshFilePath = meshDir + "ascii.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).isNull());

  // Load an invalid STL binary file (size < 84 bytes).
  meshFilePath = meshDir + "invalid_short.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).isNull());

  // Load an invalid STL binary file (size does not match the expected size).
  meshFilePath = meshDir + "invalid.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).isNull());

  // Load an invalid STL binary file (size does not match the expected size,
  // but does if incorrectly read as an 16-bit uint)
  meshFilePath = meshDir + "16bit_vs_32bit_should_fail.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).isNull());

  // Load a valid STL binary file.
  meshFilePath = meshDir + "valid.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).isNull());

  // Load a "potentially" valid STL binary file with bigger size than the
  // expected. The extra "unexpected" data at the end of the file should be
  // ignored.
  meshFilePath = meshDir + "valid_extra.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).isNull());
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );

  //initialize Ogre
  std::unique_ptr<Ogre::Root> root(new Ogre::Root());

  return RUN_ALL_TESTS();
}
