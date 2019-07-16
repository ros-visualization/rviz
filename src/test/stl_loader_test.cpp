#include <gtest/gtest.h>
#include <rviz/mesh_loader.h>
#include <ros/package.h>
#include <rviz/ogre_helpers/render_system.h>

TEST( STLLoader, load )
{
  // Get the path to the directory where the meshes are located.
  std::string meshDir = "package://rviz/src/test/meshes/";

  // Load a valid ascii STL file.
  std::string meshFilePath = meshDir + "valid_ascii.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load an ascii STL file. Note that this file uses "end facet" instead of "endfacet"
  // and therefore fails in assimp
  meshFilePath = meshDir + "ascii.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load an invalid STL binary file (size < 84 bytes).
  meshFilePath = meshDir + "invalid_short.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load an invalid STL binary file (size does not match the expected size).
  meshFilePath = meshDir + "invalid.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load an invalid STL binary file (size does not match the expected size,
  // but does if incorrectly read as an 16-bit uint)
  meshFilePath = meshDir + "16bit_vs_32bit_should_fail.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load a valid STL binary file.
  meshFilePath = meshDir + "valid.stl";
  EXPECT_TRUE(rviz::loadMeshFromResource(meshFilePath).get());

  // Load a "potentially" valid STL binary file with bigger size than the
  // expected. The extra "unexpected" data at the end of the file should be
  // ignored. Maybe. But is not ignored by assimp
  meshFilePath = meshDir + "valid_extra.stl";
  EXPECT_FALSE(rviz::loadMeshFromResource(meshFilePath).get());
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );

  // initialize ogre resource system
  rviz::RenderSystem::get();

  return RUN_ALL_TESTS();
}
