/*
This file is part of the Ristra Wonton project.
Please see the license file at the root of this repository, or at:
    https://github.com/laristra/wonton/blob/master/LICENSE
*/

#include <memory>
#include <vector>
#include <string>

#include "wonton/state/simple/simple_state_wrapper.h"

#include "gtest/gtest.h"

#include "wonton/support/wonton.h"
#include "wonton/mesh/simple/simple_mesh.h"
#include "wonton/mesh/simple/simple_mesh_wrapper.h"
#include "wonton/state/simple/simple_state.h"


TEST(Simple_State_Wrapper, WrapperTest) {
  Wonton::Simple_Mesh mymesh(0.0, 0.0, 0.0,
                              1.0, 1.0, 1.0,
                              2, 2, 2);
  Wonton::Simple_State mystate(std::make_shared<Wonton::Simple_Mesh>(mymesh));

  // Wrappers
  Wonton::Simple_Mesh_Wrapper mymesh_wrapper(mymesh);
  Wonton::Simple_State_Wrapper mystate_wrapper(mystate);

  auto numcells = mymesh_wrapper.num_owned_cells();
  auto numnodes = mymesh_wrapper.num_owned_nodes();

  std::vector<double> cellvar(numcells);
  for (int i(0); i < numcells; ++i)
    cellvar[i] = i;
  mystate.add("cellvar1", Wonton::Entity_kind::CELL, &(cellvar[0]));

  double* celldata;
  mystate_wrapper.mesh_get_data(Wonton::Entity_kind::CELL, "cellvar1",
                           &celldata);
  for (int i(0); i < numcells; ++i) {
    ASSERT_EQ(cellvar[i], celldata[i]);
  }


  std::vector<double> nodevar(numnodes);
  for (int i(0); i < numnodes; ++i)
    nodevar[i] = i;
  mystate.add("nodevar1", Wonton::Entity_kind::NODE, &(nodevar[0]));

  double* nodedata;
  mystate_wrapper.mesh_get_data(Wonton::Entity_kind::NODE, "nodevar1",
                           &nodedata);
  for (int i(0); i < numnodes; ++i) {
    ASSERT_EQ(nodevar[i], nodedata[i]);
  }

  // Check get_entity
  std::vector<std::string> names = {"cellvar1", "nodevar1"};
  std::vector<Wonton::Entity_kind>
      expected_kinds = {Wonton::Entity_kind::CELL,
                        Wonton::Entity_kind::NODE};
  Wonton::Entity_kind kind;
  for (int i(0); i < names.size(); ++i) {
    kind = mystate_wrapper.get_entity(names[i]);
    ASSERT_EQ(expected_kinds[i], kind);
  }

  // Check add_data
  std::vector<double> nodevar2(numnodes);
  for (int i(0); i < numnodes; ++i)
    nodevar2[i] = i*i;
  const double *testa = &(nodevar2[0]), *testg;
  mystate_wrapper.mesh_add_data(Wonton::Entity_kind::NODE, "nodevar2", &testa);
  mystate_wrapper.mesh_get_data(Wonton::Entity_kind::NODE, "nodevar2", &testg);
  for (size_t i = 0; i < numnodes; i++) ASSERT_EQ(testg[i], nodevar2[i]);

  // Check names
  size_t index = 0;
  std::vector<std::string>::iterator iter0 = mystate_wrapper.names_begin();
  std::vector<std::string>::iterator iter1 = mystate_wrapper.names_end();
  std::vector<std::string> names_test = mystate_wrapper.names();

  ASSERT_EQ(names_test.size(), 3);
  ASSERT_EQ(names_test[0], "cellvar1");
  ASSERT_EQ(names_test[1], "nodevar1");
  ASSERT_EQ(names_test[2], "nodevar2");
  for (std::vector<std::string>::iterator iter=iter0; iter != iter1; ++iter) {
    ASSERT_EQ(names_test[index], *iter);
    index++;
  }
}
