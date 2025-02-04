/*
This file is part of the Ristra Wonton project.
Please see the license file at the root of this repository, or at:
    https://github.com/laristra/wonton/blob/master/LICENSE
*/


#include <iostream>
#include <vector>
#include <iterator>

#include "wonton/mesh/simple/simple_mesh.h"
#include "wonton/mesh/simple/simple_mesh_wrapper.h"
#include "wonton/support/wonton.h"
#include "wonton/support/Point.h"
#include "wonton/support/Vector.h"

#include "gtest/gtest.h"

TEST(Simple_Mesh, OneCell) {
  Wonton::Simple_Mesh mesh(0.0, 0.0, 0.0,
                            1.0, 1.0, 1.0,
                            1, 1, 1);
  Wonton::Simple_Mesh_Wrapper mesh_wrapper(mesh);

  // Check basic dimensionality
  ASSERT_EQ(mesh_wrapper.space_dimension(), 3);

  // Basic ownership
  ASSERT_EQ(mesh_wrapper.num_owned_cells(), 1);
  ASSERT_EQ(mesh_wrapper.num_owned_nodes(), 8);
  ASSERT_EQ(mesh_wrapper.num_owned_faces(), 6);
  ASSERT_EQ(mesh_wrapper.num_ghost_cells(), 0);
  ASSERT_EQ(mesh_wrapper.num_ghost_nodes(), 0);
  ASSERT_EQ(mesh_wrapper.num_ghost_faces(), 0);

  // Cell information
  ASSERT_EQ(mesh_wrapper.cell_get_type(0),
            Wonton::Entity_type::PARALLEL_OWNED);
  ASSERT_EQ(mesh_wrapper.cell_get_element_type(0),
            Wonton::Element_type::HEX);

  // Connectivity information
  // Nodes for this cell; the expected are global ids and the ordering is
  // local to the cell
  std::vector<int> expectedNodes = {0, 1, 3, 2, 4, 5, 7, 6};
  std::vector<int> cellnodes;
  mesh_wrapper.cell_get_nodes(0, &cellnodes);
  for (int i(0); i < 8; ++i)
    ASSERT_EQ(expectedNodes[i], cellnodes[i]);
  // Faces and dirs; the expected are global ids
  std::vector<int> expectedFaces = {2, 5, 3, 4, 0, 1};
  std::vector<int> expectedDirs = {1, 1, -1, -1, -1, 1};
  std::vector<int> cellfaces, cellfaceDirs;
  mesh_wrapper.cell_get_faces_and_dirs(0, &cellfaces, &cellfaceDirs);
  for (int i(0); i < 6; ++i) {
    ASSERT_EQ(expectedFaces[i], cellfaces[i]);
    ASSERT_EQ(expectedDirs[i], cellfaceDirs[i]);
  }
  // Nodes of the faces; the expected are global ids
  std::vector<std::vector<int>> expectedFaceNodes = {{0, 1, 3, 2},
                                                     {4, 5, 7, 6},
                                                     {0, 1, 5, 4},
                                                     {2, 3, 7, 6},
                                                     {0, 2, 6, 4},
                                                     {1, 3, 7, 5}};
  std::vector<int> facenodes;
  for (int f(0); f < 6; ++f) {
    mesh_wrapper.face_get_nodes(f, &facenodes);
    for (int i(0); i < 4; ++i)
      ASSERT_EQ(expectedFaceNodes[f][i], facenodes[i]);
  }

  // Coordinate information
  // Ordering is the GLOBAL ordering
  std::vector<Wonton::Point<3>> expectedCoords = {{0.0, 0.0, 0.0},
                                                   {1.0, 0.0, 0.0},
                                                   {0.0, 1.0, 0.0},
                                                   {1.0, 1.0, 0.0},
                                                   {0.0, 0.0, 1.0},
                                                   {1.0, 0.0, 1.0},
                                                   {0.0, 1.0, 1.0},
                                                   {1.0, 1.0, 1.0}};
  Wonton::Point<3> nodeCoord;
  for (int i(0); i < 8; ++i) {
    mesh_wrapper.node_get_coordinates(i, &nodeCoord);
    for (int d(0); d < 3; ++d)
      ASSERT_EQ(expectedCoords[i][d], nodeCoord[d]);
  }

  // Check for nodes in cells adjacent to this
  // In a single cell setup, this should only contain the other nodes
  // of the cell.
  std::vector<int> adjnodes;
  for (int i(0); i < 8; ++i) {
    mesh_wrapper.node_get_cell_adj_nodes(i,
                                         Wonton::Entity_type::PARALLEL_OWNED,
                                         &adjnodes);
    ASSERT_EQ(adjnodes.size(), 7);
  }

  // Check for cells adjacent to this cell; adjacency determined from
  // nodes.
  // For a single cell setup, this should be empty
  std::vector<int> adjcells;
  mesh_wrapper.cell_get_node_adj_cells(0,
                                       Wonton::Entity_type::PARALLEL_OWNED,
                                       &adjcells);
  ASSERT_EQ(adjcells.size(), 0);

  // Get the tet decomposition of the cell (ask for general cell
  // decomposition with and without using the fact that its a planar
  // hex) and make sure the volumes add up to the cell volume

  double cvolume = mesh_wrapper.cell_volume(0);

  std::vector<std::array<Wonton::Point<3>, 4>> cell_tet_coords;

  // 5 tet decomposition of hex

  bool planar_hex = true;
  mesh_wrapper.decompose_cell_into_tets(0, &cell_tet_coords, planar_hex);

  int ntets = cell_tet_coords.size();
  ASSERT_EQ(5, ntets);

  double ctvolume = 0.0;
  for (int i = 0; i < ntets; i++) {
    std::array<Wonton::Point<3>, 4> & tet_coords = cell_tet_coords[i];
    Wonton::Vector<3> v0 = tet_coords[1] - tet_coords[0];
    Wonton::Vector<3> v1 = tet_coords[2] - tet_coords[0];
    Wonton::Vector<3> v2 = tet_coords[3] - tet_coords[0];

    Wonton::Vector<3> cpvec = cross(v0, v1);
    double tvolume = dot(cpvec, v2)/6.0;
    ASSERT_GT(tvolume, 0);

    ctvolume += tvolume;
  }

  ASSERT_NEAR(cvolume, ctvolume, 1.0e-12);

  // 24 tet decomposition of hex

  planar_hex = false;
  mesh_wrapper.decompose_cell_into_tets(0, &cell_tet_coords, planar_hex);

  ntets = cell_tet_coords.size();
  ASSERT_EQ(24, ntets);

  ctvolume = 0.0;
  for (int i = 0; i < ntets; i++) {
    std::array<Wonton::Point<3>, 4> & tet_coords = cell_tet_coords[i];
    Wonton::Vector<3> v0 = tet_coords[1] - tet_coords[0];
    Wonton::Vector<3> v1 = tet_coords[2] - tet_coords[0];
    Wonton::Vector<3> v2 = tet_coords[3] - tet_coords[0];

    Wonton::Vector<3> cpvec = cross(v0, v1);
    double tvolume = dot(cpvec, v2)/6.0;
    ASSERT_GT(tvolume, 0);

    ctvolume += tvolume;
  }

  ASSERT_NEAR(cvolume, ctvolume, 1.0e-12);

}



TEST(Simple_Mesh, MultiCell) {
  // Create a 2x2x2 mesh
  double xmin(0.0), ymin(0.0), zmin(0.0);
  double xmax(1.0), ymax(1.0), zmax(1.0);
  int nx(2), ny(2), nz(2);
  Wonton::Simple_Mesh mesh(xmin, ymin, zmin,
                            xmax, ymax, zmax,
                            nx, ny, nz);
  Wonton::Simple_Mesh_Wrapper mesh_wrapper(mesh);

  // Check basic dimensionality
  ASSERT_EQ(mesh_wrapper.space_dimension(), 3);

  // Basic ownership
  int ncells = mesh_wrapper.num_owned_cells();
  int nnodes = mesh_wrapper.num_owned_nodes();
  int nfaces = mesh_wrapper.num_owned_faces();

  ASSERT_EQ(ncells, nx*ny*nz);
  ASSERT_EQ(nnodes, (nx+1)*(ny+1)*(nz+1));
  ASSERT_EQ(nfaces, nx*ny*(nz+1) + nx*(ny+1)*nz + (nx+1)*ny*nz);
  ASSERT_EQ(mesh_wrapper.num_ghost_cells(), 0);
  ASSERT_EQ(mesh_wrapper.num_ghost_nodes(), 0);
  ASSERT_EQ(mesh_wrapper.num_ghost_faces(), 0);

  // Cell information
  for (int i(0); i < ncells; ++i) {
    ASSERT_EQ(mesh_wrapper.cell_get_type(i),
              Wonton::Entity_type::PARALLEL_OWNED);
    ASSERT_EQ(mesh_wrapper.cell_get_element_type(i),
              Wonton::Element_type::HEX);
  }

  // Connectivity information
  // Nodes for each cell; the expected are global ids and the ordering is
  // local to the cell
  std::vector<std::vector<int>> expectedNodes = {
    {0, 1, 4, 3, 9, 10, 13, 12},
    {1, 2, 5, 4, 10, 11, 14, 13},
    {3, 4, 7, 6, 12, 13, 16, 15},
    {4, 5, 8, 7, 13, 14, 17, 16},
    {9, 10, 13, 12, 18, 19, 22, 21},
    {10, 11, 14, 13, 19, 20, 23, 22},
    {12, 13, 16, 15, 21, 22, 25, 24},
    {13, 14, 17, 16, 22, 23, 26, 25}};
  std::vector<int> cellnodes;
  for (int i(0); i < ncells; ++i) {
    mesh_wrapper.cell_get_nodes(i, &cellnodes);
    for (int n(0); n < 8; ++n)
      ASSERT_EQ(expectedNodes[i][n], cellnodes[n]);
  }

  // Faces for each cell; the expected are global ids and the ordering is
  // local to the cell
  std::vector<std::vector<int>> expectedFaces = {
    {12, 25, 14, 24, 0, 4},
    {13, 26, 15, 25, 1, 5},
    {14, 28, 16, 27, 2, 6},
    {15, 29, 17, 28, 3, 7},
    {18, 31, 20, 30, 4, 8},
    {19, 32, 21, 31, 5, 9},
    {20, 34, 22, 33, 6, 10},
    {21, 35, 23, 34, 7, 11}
  };
  std::vector<int> expectedDirs = {1, 1, -1, -1, -1, 1};
  std::vector<int> cellfaces, cellfacedirs;
  for (int i(0); i < ncells; ++i) {
    mesh_wrapper.cell_get_faces_and_dirs(i, &cellfaces, &cellfacedirs);
    for (int f(0); f < 6; ++f) {
      ASSERT_EQ(expectedFaces[i][f], cellfaces[f]);
      ASSERT_EQ(expectedDirs[f], cellfacedirs[f]);
    }
  }

  // Nodes of the faces; the expected are global ids and the ordering is
  // local to the face
  std::vector<std::vector<int>> expectedFaceNodes = {
    // xy faces
    // z = 0 plane
    {0, 1, 4, 3},
    {1, 2, 5, 4},
    {3, 4, 7, 6},
    {4, 5, 8, 7},
    // z = 0.5 plane
    {9, 10, 13, 12},
    {10, 11, 14, 13},
    {12, 13, 16, 15},
    {13, 14, 17, 16},
    // z = 1.0 plane
    {18, 19, 22, 21},
    {19, 20, 23, 22},
    {21, 22, 25, 24},
    {22, 23, 26, 25},
    // xz faces
    // y = 0 plane bottom
    {0, 1, 10, 9},
    {1, 2, 11, 10},
    // y = 0.5 plane bottom
    {3, 4, 13, 12},
    {4, 5, 14, 13},
    // y = 1.0 plane bottom
    {6, 7, 16, 15},
    {7, 8, 17, 16},
    // y = 0 plane top
    {9, 10, 19, 18},
    {10, 11, 20, 19},
    // y = 0.5 plane top
    {12, 13, 22, 21},
    {13, 14, 23, 22},
    // y = 1.0 plane top
    {15, 16, 25, 24},
    {16, 17, 26, 25},
    // yz faces
    // x = 0 plane bottom front
    {0, 3, 12, 9},
    // x = 0.5 plane bottom front
    {1, 4, 13, 10},
    // x = 1.0 plane bottom front
    {2, 5, 14, 11},
    // x = 0 plane bottom back
    {3, 6, 15, 12},
    // x = 0.5 plane bottom back
    {4, 7, 16, 13},
    // x = 1.0 plane bottom back
    {5, 8, 17, 14},
    // x = 0 plane top front
    {9, 12, 21, 18},
    // x = 0.5 plane top front
    {10, 13, 22, 19},
    // x = 1.0 plane top front
    {11, 14, 23, 20},
    // x = 0 plane top back
    {12, 15, 24, 21},
    // x = 0.5 plane top back
    {13, 16, 25, 22},
    // x = 1.0 plane top back
    {14, 17, 26, 23}
  };
  std::vector<int> faceNodes;
  for (int f(0); f < nfaces; ++f) {
    mesh_wrapper.face_get_nodes(f, &faceNodes);
    for (int i(0); i < 4; ++i)
      ASSERT_EQ(expectedFaceNodes[f][i], faceNodes[i]);
  }

  // Coordinate information
  // Ordering is the global ordering
  Wonton::Point<3> expectedCoords, nodeCoords;
  double dx((xmax-xmin)/nx), dy((ymax-ymin)/ny), dz((zmax-zmin)/nz);
  for (int iz(0); iz < nz+1; ++iz)
    for (int iy(0); iy < ny+1; ++iy)
      for (int ix(0); ix < nx+1; ++ix) {
        expectedCoords = {ix*dx + xmin,
                          iy*dy + ymin,
                          iz*dz + zmin};
        int thisNode = ix + iy*(nx+1) + iz*(nx+1)*(ny+1);
        mesh_wrapper.node_get_coordinates(thisNode, &nodeCoords);
        for (int d(0); d < 3; ++d)
          ASSERT_EQ(expectedCoords[d], nodeCoords[d]);
      }

  // Check for nodes in cells adjacent to each node
  // Ordering is cell first, then nodes within a cell
  std::vector<std::vector<int>> adjExpectedNodes = {
    // node 0; just the others in this cell
    {1, 4, 3, 9, 10, 13, 12},
    // node 1; cells 0,1
    {0, 4, 3, 9, 10, 13, 12, 2, 5, 11, 14},
    // node 2; just the others in this cell
    {1, 5, 4, 10, 11, 14, 13},
    // node 3; cells 0,3
    {0, 1, 4, 9, 10, 13, 12, 7, 6, 16, 15},
    // node 4; cells 0,1,2,3
    {0, 1, 3, 9, 10, 13, 12, 2, 5, 11, 14, 7, 6, 16, 15, 8, 17},
    // node 5; cells 1,3
    {1, 2, 4, 10, 11, 14, 13, 8, 7, 17, 16},
    // node 6; just the others in this cell
    {3, 4, 7, 12, 13, 16, 15},
    // node 7; cells 2,3
    {3, 4, 6, 12, 13, 16, 15, 5, 8, 14, 17},
    // node 8; just the others in this cell
    {4, 5, 7, 13, 14, 17, 16},
    // node 9; cells 0,4
    {0, 1, 4, 3, 10, 13, 12, 18, 19, 22, 21},
    // node 10; cells 0,1,4,5
    {0, 1, 4, 3, 9, 13, 12, 2, 5, 11, 14, 18, 19, 22, 21, 20, 23},
    // node 11; cells 1,5
    {1, 2, 5, 4, 10, 14, 13, 19, 20, 23, 22},
    // node 12; cells 0,2,4,6
    {0, 1, 4, 3, 9, 10, 13, 7, 6, 16, 15, 18, 19, 22, 21, 25, 24},
    // node 13; everyone else
    {0, 1, 4, 3, 9, 10, 12, 2, 5, 11, 14, 7, 6, 16, 15, 8, 17,
     18, 19, 22, 21, 20, 23, 25, 24, 26},
    // node 14; cells 1,3,5,7
    {1, 2, 5, 4, 10, 11, 13, 8, 7, 17, 16, 19, 20, 23, 22, 26, 25},
    // node 15; cells 2,6
    {3, 4, 7, 6, 12, 13, 16, 21, 22, 25, 24},
    // node 16; cells 2,3,6,7
    {3, 4, 7, 6, 12, 13, 15, 5, 8, 14, 17, 21, 22, 25, 24, 23, 26},
    // node 17; cells 3,7
    {4, 5, 8, 7, 13, 14, 16, 22, 23, 26, 25},
    // node 18; just the others in this cell
    {9, 10, 13, 12, 19, 22, 21},
    // node 19; cells 4,5
    {9, 10, 13, 12, 18, 22, 21, 11, 14, 20, 23},
    // node 20; just the others in this cell
    {10, 11, 14, 13, 19, 23, 22},
    // node 21; cells 4,6
    {9, 10, 13, 12, 18, 19, 22, 16, 15, 25, 24},
    // node 22; cells 4,5,6,7
    {9, 10, 13, 12, 18, 19, 21, 11, 14, 20, 23, 16, 15, 25, 24, 17, 26},
    // node 23; cells 5,7
    {10, 11, 14, 13, 19, 20, 22, 17, 16, 26, 25},
    // node 24; just the others in this cell
    {12, 13, 16, 15, 21, 22, 25},
    // node 25; cells 6,7
    {12, 13, 16, 15, 21, 22, 24, 14, 17, 23, 26},
    // node 26; just the others in this cell
    {13, 14, 17, 16, 22, 23, 25}
  };
  std::vector<int> adjNodes;
  for (int i(0); i < nnodes; ++i) {
    int adjExpectedNumNodes = adjExpectedNodes[i].size();

    mesh_wrapper.node_get_cell_adj_nodes(i,
                                         Wonton::Entity_type::PARALLEL_OWNED,
                                         &adjNodes);
    ASSERT_EQ(adjExpectedNumNodes, adjNodes.size());

    for (int j(0); j < adjExpectedNumNodes; ++j)
      ASSERT_EQ(adjExpectedNodes[i][j], adjNodes[j]);
  }

  // Check for cells adjacent to this cell, determined by nodes
  // For this 2x2x2 case, each cell is attached to all the others
  int expectedNumAdjCells = ncells-1;
  std::vector<int> adjCells;
  for (int i(0); i < ncells; ++i) {
    mesh_wrapper.cell_get_node_adj_cells(i,
                                         Wonton::Entity_type::PARALLEL_OWNED,
                                         &adjCells);
    ASSERT_EQ(expectedNumAdjCells, adjCells.size());
  }
}


TEST(Simple_Mesh, AdjCell) {
  // Create a 2x2x3 mesh
  double xmin(0.0), ymin(0.0), zmin(0.0);
  double xmax(1.0), ymax(1.0), zmax(1.0);
  int nx(2), ny(2), nz(3);
  Wonton::Simple_Mesh mesh(xmin, ymin, zmin,
                            xmax, ymax, zmax,
                            nx, ny, nz);
  Wonton::Simple_Mesh_Wrapper mesh_wrapper(mesh);

  int ncells = mesh_wrapper.num_owned_cells();
  ASSERT_EQ(ncells, nx*ny*nz);

  // Check for cells adjacent to each cell; adjacency determined from nodes
  std::vector<std::vector<int>> expectedAdjCells = {
    // bottom plane
    {1, 2, 3, 4, 5, 6, 7},
    {0, 3, 2, 4, 5, 7, 6},
    {0, 1, 3, 4, 6, 5, 7},
    {0, 1, 2, 4, 5, 6, 7},
    // middle plane
    {0, 1, 5, 2, 3, 6, 7, 8, 9, 10, 11},
    {0, 1, 4, 3, 7, 2, 6, 8, 9, 11, 10},
    {0, 2, 4, 1, 3, 5, 7, 8, 10, 9, 11},
    {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11},
    // top plane
    {4, 5, 9, 6, 7, 10, 11},
    {4, 5, 8, 7, 11, 6, 10},
    {4, 6, 8, 5, 7, 9, 11},
    {4, 5, 6, 7, 8, 9, 10}
  };
  std::vector<int> adjCells;
  for (int i(0); i < ncells; ++i) {
    int expectedNumAdjCells = expectedAdjCells[i].size();
    mesh_wrapper.cell_get_node_adj_cells(i,
                                         Wonton::Entity_type::PARALLEL_OWNED,
                                         &adjCells);
    ASSERT_EQ(expectedNumAdjCells, adjCells.size());
    for (int j(0); j < expectedNumAdjCells; ++j)
      ASSERT_EQ(expectedAdjCells[i][j], adjCells[j]);
  }
}

TEST(Simple_Mesh, GlobalID) {
  Wonton::Simple_Mesh mesh(0.0, 0.0, 0.0,
                            1.0, 1.0, 1.0,
                            10, 10, 10);
  Wonton::Simple_Mesh_Wrapper mesh_wrapper(mesh);

  int ncells = mesh_wrapper.num_owned_cells();

  for (int c(0); c < ncells; ++c) {
    ASSERT_EQ(c, mesh_wrapper.get_global_id(c, Wonton::Entity_kind::CELL));
  }
}


// Check that we can facetize cells and dual cells in 3D correctly

TEST(Simple_Mesh, MultiCell_Facetization) {
  // Create a 2x2x2 mesh
  double xmin(0.0), ymin(0.0), zmin(0.0);
  double xmax(2.0), ymax(2.0), zmax(2.0);
  int nx(2), ny(2), nz(2);
  Wonton::Simple_Mesh mesh(xmin, ymin, zmin,
                            xmax, ymax, zmax,
                            nx, ny, nz);
  Wonton::Simple_Mesh_Wrapper mesh_wrapper(mesh);

  // The volume of any cell in the mesh is known to be 1
  double cvolume = 1.0;

  // Sum of areas of the cell faces is known to be 6
  double cfarea = 6.0;

  // Get the facetization of any one cell and check surface area and
  // volume (by divergence theorem). By definition, each facet is
  // planar and has only one normal

  std::vector<Wonton::Point<3>> fctpoints;
  std::vector<std::vector<int>> facets;
  mesh_wrapper.cell_get_facetization(6, &facets, &fctpoints);

  int nfacets = facets.size();
  ASSERT_EQ(24, nfacets);
  double cfctvolume = 0.0, cfctarea = 0.0;
  for (int i = 0; i < nfacets; i++) {
    int nfpoints = facets[i].size();
    for (int j = 1; j < nfpoints-1; j++) {
      Wonton::Point<3> p0 = fctpoints[facets[i][0]];
      Wonton::Point<3> p1 = fctpoints[facets[i][j]];
      Wonton::Point<3> p2 = fctpoints[facets[i][j+1]];

      Wonton::Vector<3> v0 = p1 - p0;
      Wonton::Vector<3> v1 = p2 - p0;
      Wonton::Vector<3> cpvec = cross(v0, v1)/2.0;

      cfctarea += cpvec.norm();

      // We are using the fact the cell is an axis aligned cube to do
      // a short cut form for integral of X.N where X is the
      // coordinate vector at any point on the face and N is the area
      // weighted normal of the face

      Wonton::Vector<3> p0vec(p0[0], p0[1], p0[2]);
      cfctvolume += dot(p0vec, cpvec)/3.0;
    }
  }

  ASSERT_NEAR(cfarea, cfctarea, 1.0e-12);
  ASSERT_NEAR(cvolume, cfctvolume, 1.0e-12);

  // Get the facetization of the control volume around an interior
  // node and check surface area and volume. The volume and surface
  // area should be the same as a cell
  
  // Find index of interior node without assuming any numbering scheme
  int nnodes = mesh_wrapper.num_owned_nodes();
  int inode = -1;
  Wonton::Point<3> cen(1.0, 1.0, 1.0);
  for (int n = 0; n < nnodes; n++) {
    Wonton::Point<3> nxyz;
    mesh_wrapper.node_get_coordinates(n, &nxyz);
    if (Wonton::approxEq(cen, nxyz, 1.0e-12)) {
      inode = n;
      break;
    }
  }
  ASSERT_NE(-1, inode);
  mesh_wrapper.dual_cell_get_facetization(inode, &facets, &fctpoints);

  nfacets = facets.size();
  ASSERT_EQ(48, nfacets);
  cfctvolume = 0.0;
  cfctarea = 0.0;
  for (int i = 0; i < nfacets; i++) {
    int nfpoints = facets[i].size();
    for (int j = 1; j < nfpoints-1; j++) {
      Wonton::Point<3> p0 = fctpoints[facets[i][0]];
      Wonton::Point<3> p1 = fctpoints[facets[i][j]];
      Wonton::Point<3> p2 = fctpoints[facets[i][j+1]];

      Wonton::Vector<3> v0 = p1 - p0;
      Wonton::Vector<3> v1 = p2 - p0;
      Wonton::Vector<3> cpvec = cross(v0, v1)/2.0;

      cfctarea += cpvec.norm();

      // We are using the fact the cell is an axis aligned cube to do
      // a short cut form for integral of X.N where X is the
      // coordinate vector at any point on the face and N is the area
      // weighted normal of the face

      Wonton::Vector<3> p0vec(p0[0], p0[1], p0[2]);
      cfctvolume += dot(p0vec, cpvec)/3.0;
    }
  }

  ASSERT_NEAR(cfarea, cfctarea, 1.0e-12);
  ASSERT_NEAR(cvolume, cfctvolume, 1.0e-12);


  // Get the facetization of the control volume around a corner node
  // (0) and check surface area and volume. The volume should be 1/8th
  // the volume of a cell and the surface area should be 1/4th the
  // surface area of a cell. WE ARE NOT TESTING TWO OTHER CASES - NODE
  // ON AN EXTERIOR EDGE OF THE DOMAIN AND NODE ON AN EXTERIOR FACE OF
  // THE DOMAIN AS THESE TWO CASES WILL CAPTURE THE POSSIBLE FAILURE
  // MODES
  
  mesh_wrapper.dual_cell_get_facetization(0, &facets, &fctpoints);

  nfacets = facets.size();
  ASSERT_EQ(12, nfacets);
  cfctvolume = 0.0;
  cfctarea = 0.0;
  for (int i = 0; i < nfacets; i++) {
    int nfpoints = facets[i].size();
    for (int j = 1; j < nfpoints-1; j++) {
      Wonton::Point<3> p0 = fctpoints[facets[i][0]];
      Wonton::Point<3> p1 = fctpoints[facets[i][j]];
      Wonton::Point<3> p2 = fctpoints[facets[i][j+1]];

      Wonton::Vector<3> v0 = p1 - p0;
      Wonton::Vector<3> v1 = p2 - p0;
      Wonton::Vector<3> cpvec = cross(v0, v1)/2.0;

      cfctarea += cpvec.norm();

      // We are using the fact the cell is an axis aligned cube to do
      // a short cut form for integral of X.N where X is the
      // coordinate vector at any point on the face and N is the area
      // weighted normal of the face

      Wonton::Vector<3> p0vec(p0[0], p0[1], p0[2]);
      cfctvolume += dot(p0vec, cpvec)/3.0;
    }
  }

  ASSERT_NEAR(cfarea/4.0, cfctarea, 1.0e-12);
  ASSERT_NEAR(cvolume/8.0, cfctvolume, 1.0e-12);

}

