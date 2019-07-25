#ifndef AMANZI_MESH_WRAPPER_H_
#define AMANZI_MESH_WRAPPER_H_

#include <cassert>
#include <algorithm>
#include <vector>
#include <array>
#include <utility>

#include "Mesh.hh" // Amanzi mesh header

#include "wonton/mesh/AuxMeshTopology.h"
#include "wonton/support/wonton.h"
#include "wonton/support/Point.h"

namespace Wonton {
  template<int D>
  Point<D> toWontonPoint(Amanzi::AmanziGeometry::Point const & jp) {
    Point<D> pp;
    assert(jp.dim() == D);
    for (int d = 0; d < D; ++d) pp[d] = jp[d];
    return pp;
  }
  class Amanzi_Mesh_Wrapper : public AuxMeshTopology<Amanzi_Mesh_Wrapper> {
  private:
    Wonton::Entity_type amanzi2wonton_type(Amanzi::AmanziMesh::Parallel_type etype) const {
      static Wonton::Entity_type amanzi2wonton[] = { DELETED, PARALLEL_OWNED, PARALLEL_GHOST, ALL };
      return amanzi2wonton[static_cast<int>(etype)];
    }
    Amanzi::AmanziMesh::Parallel_type wonton2amanzi_type(Wonton::Entity_type etype) const {
      if (etype == PARALLEL_OWNED) return Amanzi::AmanziMesh::Parallel_type::OWNED;
      if (etype == PARALLEL_GHOST) return Amanzi::AmanziMesh::Parallel_type::GHOST;
      if (etype == ALL) return Amanzi::AmanziMesh::Parallel_type::ALL;
      if (etype == TYPE_UNKNOWN) return Amanzi::AmanziMesh::Parallel_type::PTYPE_UNKNOWN;
      throw std::invalid_argument("__ function __: type is not supported");
    }
    Wonton::Element_type amanzi2wonton_elemtype(Amanzi::AmanziMesh::Cell_type elemtype) const {
      static Wonton::Element_type amanzi2wonton_elemtype[] = {UNKNOWN_TOPOLOGY, TRI, QUAD, POLYGON, TET, PRISM, PYRAMID, HEX, POLYHEDRON};
      return amanzi2wonton_elemtype[static_cast<int>(elemtype)];
    }
    Amanzi::AmanziMesh::Mesh const & amanzi_mesh_;
  public:
    explicit Amanzi_Mesh_Wrapper(Amanzi::AmanziMesh::Mesh const & mesh, bool request_sides = true, bool request_wedges = true, bool request_corners = true)
      : amanzi_mesh_(mesh)
      , AuxMeshTopology<Amanzi_Mesh_Wrapper>(request_sides, request_wedges, request_corners) {
      AuxMeshTopology<Amanzi_Mesh_Wrapper>::build_aux_entities(); 
    }
    Amanzi_Mesh_Wrapper(Amanzi_Mesh_Wrapper const &) = delete;
    Amanzi_Mesh_Wrapper & operator=(Amanzi_Mesh_Wrapper const &) = delete;
    ~Amanzi_Mesh_Wrapper() {}
    int space_dimension() const {
      return amanzi_mesh_.space_dimension();
    }
    int num_owned_cells() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::CELL, Amanzi::AmanziMesh::Parallel_type::OWNED);
    }
    int num_owned_faces() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::FACE, Amanzi::AmanziMesh::Parallel_type::OWNED);
    }
    int num_owned_nodes() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::NODE, Amanzi::AmanziMesh::Parallel_type::OWNED);
    }
    int num_ghost_cells() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::CELL, Amanzi::AmanziMesh::Parallel_type::GHOST);
    }
    int num_ghost_faces() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::FACE, Amanzi::AmanziMesh::Parallel_type::GHOST);
    }
    int num_ghost_nodes() const {
      return amanzi_mesh_.num_entities(Amanzi::AmanziMesh::NODE, Amanzi::AmanziMesh::Parallel_type::GHOST);
    }
    Wonton::Entity_type cell_get_type(int const cellid) const {
      auto etype = amanzi_mesh_.entity_get_ptype(Amanzi::AmanziMesh::CELL, cellid);
      return amanzi2wonton_type(etype);
    }
    Wonton::Entity_type node_get_type(int const nodeid) const {
      auto etype = amanzi_mesh_.entity_get_ptype(Amanzi::AmanziMesh::NODE, nodeid);
      return amanzi2wonton_type(etype);
    }
    Wonton::Element_type cell_get_element_type(int const cellid) const {
      auto ctype = amanzi_mesh_.cell_get_type(cellid);
      return amanzi2wonton_elemtype(ctype);
    }
    void cell_get_faces_and_dirs(int const cellid, std::vector<int> *cfaces, std::vector<int> *cfdirs) const {
      amanzi_mesh_.cell_get_faces_and_dirs(cellid, cfaces, cfdirs);
    }
    void cell_get_nodes(int cellid, std::vector<int> *nodes) const {
      amanzi_mesh_.cell_get_nodes(cellid, nodes);
    }
    void cell_get_node_adj_cells(int const cellid, Entity_type const ptype, std::vector<int> *adjcells) const {
      amanzi_mesh_.cell_get_node_adj_cells(cellid, wonton2amanzi_type(ptype), adjcells);
    }
    void face_get_nodes(int const faceid, std::vector<int> *fnodes) const {
      amanzi_mesh_.face_get_nodes(faceid, fnodes);
    }
    void node_get_cells(int const nodeid, Entity_type const ptype, std::vector<int> *nodecells) const {
      amanzi_mesh_.node_get_cells(nodeid, wonton2amanzi_type(ptype), nodecells);
    }
    //! Get global id
    GID_t get_global_id(int const id, Entity_kind const kind) const {
      return amanzi_mesh_.GID(id, (Amanzi::AmanziMesh::Entity_kind) kind);
    }
    template <int D>
    void node_get_coordinates(int const nodeid, Point<D>* pp) const {
      Amanzi::AmanziGeometry::Point jp;
      amanzi_mesh_.node_get_coordinates(nodeid, &jp);
      assert(jp.dim() == D);
      *pp = toWontonPoint<D>(jp);
    }
  };  // class Amanzi_Mesh_Wrapper
}  // end namespace Wonton

#endif // AMANZI_MESH_WRAPPER_H_
