//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RRTTree.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "RRTTree.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "NeighborSearcher.h"
#include "SqrtApproxNeighborSearcher.h"
#include "eml_setop.h"
#include "minOrMax.h"
#include "pathToTrajectory.h"
#include "planTrajectory.h"
#include "planTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "stack1.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions

//
// Arguments    : double childId
// Return Type  : void
//
namespace mgen
{
  void f_matlabshared_planning_interna::c_rectifyDownstreamCostsNoRecur(double
    childId)
  {
    coder::array<unsigned int, 2U> r;
    short size_tmp_idx_0;
    coder_internal_stack stack;
    int edgeId_tmp;
    int n;
    int i;
    this->get_Edges(r);
    size_tmp_idx_0 = static_cast<short>(r.size(0));
    stack.init();
    edgeId_tmp = size_tmp_idx_0;
    for (n = 0; n < edgeId_tmp; n++) {
      this->get_Edges(r);
      if (r[n] == childId) {
        if (stack.n == stack.d.size(0)) {
          i = stack.d.size(0);
          stack.d.set_size((stack.d.size(0) + 1));
          stack.d[i] = static_cast<double>(n) + 1.0;
        } else {
          stack.d[stack.n] = static_cast<double>(n) + 1.0;
        }

        stack.n++;
      }
    }

    while (stack.n != 0) {
      unsigned int parentId;
      unsigned int b_childId;
      edgeId_tmp = stack.n - 1;
      stack.n = edgeId_tmp;
      this->get_Edges(r);
      edgeId_tmp = static_cast<int>(stack.d[edgeId_tmp]) - 1;
      parentId = r[edgeId_tmp];
      this->get_Edges(r);
      b_childId = r[edgeId_tmp + r.size(0)];
      this->CostBuffer[edgeId_tmp] = this->costTo(parentId) + this->edgeCost
        (parentId, b_childId);
      edgeId_tmp = size_tmp_idx_0;
      for (n = 0; n < edgeId_tmp; n++) {
        this->get_Edges(r);
        if (r[n] == b_childId) {
          if (stack.n == stack.d.size(0)) {
            i = stack.d.size(0);
            stack.d.set_size((stack.d.size(0) + 1));
            stack.d[i] = static_cast<double>(n) + 1.0;
          } else {
            stack.d[stack.n] = static_cast<double>(n) + 1.0;
          }

          stack.n++;
        }
      }
    }
  }

  //
  // Arguments    : unsigned int id
  // Return Type  : double
  //
  double f_matlabshared_planning_interna::costTo(unsigned int id) const
  {
    double cost;
    if (id < 2U) {
      cost = 0.0;
    } else {
      cost = this->CostBuffer[static_cast<int>(id) - 2];
    }

    return cost;
  }

  //
  // Arguments    : const double id_data[]
  //                const int id_size[2]
  //                double cost_data[]
  //                int cost_size[2]
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::costTo(const double id_data[], const int
    id_size[2], double cost_data[], int cost_size[2]) const
  {
    int loop_ub_tmp;
    int k;
    boolean_T y;
    boolean_T x_data[1];
    loop_ub_tmp = id_size[0] * id_size[1];
    for (k = 0; k < loop_ub_tmp; k++) {
      x_data[k] = (id_data[k] < 2.0);
    }

    y = ((id_size[0] != 0) && (id_size[1] != 0));
    if (y) {
      boolean_T exitg1;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= loop_ub_tmp - 1)) {
        if (!x_data[0]) {
          y = false;
          exitg1 = true;
        } else {
          k = 1;
        }
      }
    }

    if (y) {
      cost_size[0] = 1;
      cost_size[1] = 1;
      cost_data[0] = 0.0;
    } else {
      cost_size[0] = id_size[0];
      cost_size[1] = id_size[1];
      for (k = 0; k < loop_ub_tmp; k++) {
        cost_data[k] = this->CostBuffer[static_cast<int>(id_data[k] - 1.0) - 1];
      }
    }
  }

  //
  // Arguments    : unsigned int fromId
  //                unsigned int toId
  // Return Type  : double
  //
  double f_matlabshared_planning_interna::edgeCost(unsigned int fromId, unsigned
    int toId) const
  {
    double b_this[3];
    double c_this[3];
    b_this[0] = this->NodeBuffer[static_cast<int>(fromId) - 1];
    c_this[0] = this->NodeBuffer[static_cast<int>(toId) - 1];
    b_this[1] = this->NodeBuffer[static_cast<int>(fromId) + 10000];
    c_this[1] = this->NodeBuffer[static_cast<int>(toId) + 10000];
    b_this[2] = this->NodeBuffer[static_cast<int>(fromId) + 20001];
    c_this[2] = this->NodeBuffer[static_cast<int>(toId) + 20001];
    return this->NeighborSearcher->distance(b_this, c_this);
  }

  //
  // Arguments    : double fromId
  //                double toId
  // Return Type  : double
  //
  double f_matlabshared_planning_interna::edgeCost(double fromId, double toId)
    const
  {
    int this_tmp;
    double b_this[3];
    int b_this_tmp;
    double c_this[3];
    this_tmp = static_cast<int>(fromId);
    b_this[0] = this->NodeBuffer[this_tmp - 1];
    b_this_tmp = static_cast<int>(toId);
    c_this[0] = this->NodeBuffer[b_this_tmp - 1];
    b_this[1] = this->NodeBuffer[this_tmp + 10000];
    c_this[1] = this->NodeBuffer[b_this_tmp + 10000];
    b_this[2] = this->NodeBuffer[this_tmp + 20001];
    c_this[2] = this->NodeBuffer[b_this_tmp + 20001];
    return this->NeighborSearcher->distance(b_this, c_this);
  }

  //
  // Arguments    : coder::array<unsigned int, 2U> &edges
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::get_Edges(coder::array<unsigned int, 2U>
    &edges) const
  {
    double d;
    int loop_ub;
    int i;
    d = this->EdgeIndex - 1.0;
    if (1.0 > d) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }

    edges.set_size(loop_ub, 2);
    for (i = 0; i < loop_ub; i++) {
      edges[i] = this->EdgeBuffer[i];
    }

    for (i = 0; i < loop_ub; i++) {
      edges[i + edges.size(0)] = this->EdgeBuffer[i + 10001];
    }
  }

  //
  // Arguments    : double fromId
  //                double toId
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::addEdge(double fromId, double toId)
  {
    double edgeId;
    double d;
    unsigned int u;
    int i;
    int i1;
    edgeId = this->EdgeIndex;
    d = rt_roundd_snf(fromId);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    i = static_cast<int>(edgeId);
    i1 = i - 1;
    this->EdgeBuffer[i1] = u;
    d = rt_roundd_snf(toId);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    this->EdgeBuffer[i + 10000] = u;
    this->CostBuffer[i1] = this->costTo(fromId) + this->edgeCost(fromId, toId);
    this->EdgeIndex = edgeId + 1.0;
  }

  //
  // Arguments    : const double node[3]
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::addNode(const double node[3])
  {
    double id;
    int i;
    id = this->NodeIndex;
    i = static_cast<int>(id);
    this->NodeBuffer[i - 1] = node[0];
    this->NodeBuffer[i + 10000] = node[1];
    this->NodeBuffer[i + 20001] = node[2];
    this->NodeIndex = id + 1.0;
  }

  //
  // Arguments    : const double node[3]
  // Return Type  : double
  //
  double f_matlabshared_planning_interna::b_addNode(const double node[3])
  {
    double id;
    int i;
    id = this->NodeIndex;
    i = static_cast<int>(id);
    this->NodeBuffer[i - 1] = node[0];
    this->NodeBuffer[i + 10000] = node[1];
    this->NodeBuffer[i + 20001] = node[2];
    this->NodeIndex = id + 1.0;
    return id;
  }

  //
  // Arguments    : const double node_data[]
  //                const int node_size[2]
  //                double nearestNode[3]
  //                double *nearestId
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::b_nearest(const double node_data[],
    const int node_size[2], double nearestNode[3], double *nearestId)
  {
    this->NeighborSearcher->b_nearest(this->NodeBuffer, (this->NodeIndex - 1.0),
      node_data, node_size, nearestNode, nearestId);
  }

  //
  // Arguments    : double id
  // Return Type  : double
  //
  double f_matlabshared_planning_interna::costTo(double id) const
  {
    double cost;
    if (id < 2.0) {
      cost = 0.0;
    } else {
      cost = this->CostBuffer[static_cast<int>(id - 1.0) - 1];
    }

    return cost;
  }

  //
  // Arguments    : coder::array<double, 2U> &nodes
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::get_Nodes(coder::array<double, 2U>
    &nodes) const
  {
    double d;
    int loop_ub;
    d = this->NodeIndex - 1.0;
    if (1.0 > d) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }

    nodes.set_size(loop_ub, 3);
    for (int i = 0; i < 3; i++) {
      for (int i1 = 0; i1 < loop_ub; i1++) {
        nodes[i1 + nodes.size(0) * i] = this->NodeBuffer[i1 + 10001 * i];
      }
    }
  }

  //
  // Arguments    : e_matlabshared_planning_interna *neighborSearcher
  // Return Type  : f_matlabshared_planning_interna *
  //
  f_matlabshared_planning_interna *f_matlabshared_planning_interna::init
    (e_matlabshared_planning_interna *neighborSearcher)
  {
    f_matlabshared_planning_interna *this_;
    int i;
    this_ = this;
    this_->NeighborSearcher = neighborSearcher;
    for (i = 0; i < 30003; i++) {
      this_->NodeBuffer[i] = 0.0;
    }

    for (i = 0; i < 20002; i++) {
      this_->EdgeBuffer[i] = 0U;
    }

    for (i = 0; i < 10001; i++) {
      this_->CostBuffer[i] = 0.0;
    }

    this_->NodeIndex = 1.0;
    this_->EdgeIndex = 1.0;
    return this_;
  }

  //
  // Arguments    : const double node[3]
  //                coder::array<double, 2U> &nearNodes
  //                coder::array<double, 1U> &nearIds
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::near(const double node[3], coder::array<
    double, 2U> &nearNodes, coder::array<double, 1U> &nearIds)
  {
    this->NeighborSearcher->near(this->NodeBuffer, (this->NodeIndex - 1.0), node,
      std::ceil(32.61938194150855 * std::log((this->NodeIndex - 1.0) + 1.0)),
      nearNodes, nearIds);
  }

  //
  // Arguments    : double childId
  //                double newParentId
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::replaceParent(double childId, double
    newParentId)
  {
    double d;
    unsigned int u;
    int i;
    d = rt_roundd_snf(newParentId);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    i = static_cast<int>(childId - 1.0) - 1;
    this->EdgeBuffer[i] = u;
    this->CostBuffer[i] = this->costTo(newParentId) + this->edgeCost(newParentId,
      childId);
    this->c_rectifyDownstreamCostsNoRecur(childId);
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::reset()
  {
    this->NodeIndex = 1.0;
    this->EdgeIndex = 1.0;
    this->NeighborSearcher->reset();
  }

  //
  // Arguments    : const double childId_data[]
  //                const int childId_size[2]
  //                const coder::array<double, 2U> &goalNodeIds
  //                unsigned int path_data[]
  //                int path_size[2]
  //                double totalCost_data[]
  //                int totalCost_size[2]
  // Return Type  : void
  //
  void f_matlabshared_planning_interna::shortestPathFromRoot(const double
    childId_data[], const int childId_size[2], const coder::array<double, 2U>
    &goalNodeIds, unsigned int path_data[], int path_size[2], double
    totalCost_data[], int totalCost_size[2]) const
  {
    int b_childId_size[2];
    int loop_ub;
    int i;
    double b_childId_data[1];
    double edgeId_data[1];
    int edgeId_size[2];
    int edgeId_idx_0_tmp;
    unsigned int parentId_data[1];
    boolean_T x_data[1];
    int k;
    int i1;
    int b_path_size[2];
    int c_path_size[2];
    int b_loop_ub;
    static unsigned int b_path_data[10001];
    static unsigned int c_path_data[10002];
    static int ia_data[10001];
    int ia_size[1];
    static int ib_data[10001];
    int ib_size[1];
    b_childId_size[0] = childId_size[0];
    b_childId_size[1] = childId_size[1];
    loop_ub = childId_size[0] * childId_size[1];
    for (i = 0; i < loop_ub; i++) {
      b_childId_data[i] = childId_data[i] - 1.0;
    }

    maximum2(b_childId_data, b_childId_size, edgeId_data, edgeId_size);
    edgeId_idx_0_tmp = edgeId_size[0] * edgeId_size[1];
    for (i = 0; i < edgeId_idx_0_tmp; i++) {
      parentId_data[i] = this->EdgeBuffer[static_cast<int>(edgeId_data[i]) - 1];
    }

    path_size[1] = 2;
    for (i = 0; i < edgeId_idx_0_tmp; i++) {
      path_data[i] = this->EdgeBuffer[static_cast<int>(edgeId_data[i]) + 10000];
    }

    if (0 <= edgeId_idx_0_tmp - 1) {
      std::memcpy(&path_data[1], &parentId_data[0], edgeId_idx_0_tmp * sizeof
                  (unsigned int));
    }

    int exitg1;
    do {
      boolean_T y;
      exitg1 = 0;
      for (i = 0; i < edgeId_idx_0_tmp; i++) {
        x_data[i] = (parentId_data[i] != 1U);
      }

      y = (edgeId_idx_0_tmp != 0);
      if (y) {
        boolean_T exitg2;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= edgeId_idx_0_tmp - 1)) {
          if (!x_data[0]) {
            y = false;
            exitg2 = true;
          } else {
            k = 1;
          }
        }
      }

      if (y) {
        short input_sizes_idx_1;
        signed char b_input_sizes_idx_1;
        for (i = 0; i < edgeId_idx_0_tmp; i++) {
          unsigned int qY;
          qY = parentId_data[i] - 1U;
          if (qY > parentId_data[i]) {
            qY = 0U;
          }

          parentId_data[i] = this->EdgeBuffer[static_cast<int>(qY) - 1];
        }

        if (path_size[1] != 0) {
          input_sizes_idx_1 = static_cast<short>(path_size[1]);
        } else {
          input_sizes_idx_1 = 0;
        }

        b_input_sizes_idx_1 = static_cast<signed char>(edgeId_idx_0_tmp != 0);
        if (path_size[1] != 0) {
          k = static_cast<short>(path_size[1]);
        } else {
          k = 0;
        }

        c_path_size[1] = input_sizes_idx_1 + b_input_sizes_idx_1;
        loop_ub = input_sizes_idx_1;
        if (0 <= loop_ub - 1) {
          std::memcpy(&b_path_data[0], &path_data[0], loop_ub * sizeof(unsigned
            int));
        }

        loop_ub = b_input_sizes_idx_1;
        if (0 <= loop_ub - 1) {
          b_path_data[k] = parentId_data[0];
        }

        path_size[1] = c_path_size[1];
        loop_ub = c_path_size[1];
        if (0 <= loop_ub - 1) {
          std::memcpy(&path_data[0], &b_path_data[0], loop_ub * sizeof(unsigned
            int));
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (1 > path_size[1]) {
      i = 0;
      k = 1;
      i1 = -1;
    } else {
      i = path_size[1] - 1;
      k = -1;
      i1 = 0;
    }

    b_path_size[0] = 1;
    loop_ub = div_s32_floor(i1 - i, k);
    b_loop_ub = loop_ub + 1;
    b_path_size[1] = b_loop_ub;
    for (i1 = 0; i1 <= loop_ub; i1++) {
      edgeId_idx_0_tmp = i + k * i1;
      c_path_data[i1] = path_data[edgeId_idx_0_tmp];
      b_path_data[i1] = path_data[edgeId_idx_0_tmp];
    }

    path_size[0] = 1;
    path_size[1] = b_loop_ub;
    if (0 <= b_loop_ub - 1) {
      std::memcpy(&path_data[0], &b_path_data[0], b_loop_ub * sizeof(unsigned
        int));
    }

    do_vectors(c_path_data, b_path_size, goalNodeIds, b_path_data, c_path_size,
               ia_data, ia_size, ib_data, ib_size);
    if (ia_size[0] != 0) {
      if (1 > ia_data[0]) {
        loop_ub = 0;
      } else {
        loop_ub = ia_data[0];
      }

      path_size[0] = 1;
      path_size[1] = loop_ub;
      if (0 <= loop_ub - 1) {
        std::memcpy(&path_data[0], &c_path_data[0], loop_ub * sizeof(unsigned
          int));
      }
    }

    this->costTo(childId_data, childId_size, totalCost_data, totalCost_size);
  }
}

//
// File trailer for RRTTree.cpp
//
// [EOF]
//
