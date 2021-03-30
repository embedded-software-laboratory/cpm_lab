//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sort.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "sort.h"
#include "DubinsPathSegment.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "unique.h"
#include <string.h>

// Function Definitions

//
// Arguments    : coder::array<double, 1U> *x
//                coder::array<int, 1U> *idx
// Return Type  : void
//
namespace mgen
{
  void b_sort(coder::array<double, 1U> &x, coder::array<int, 1U> &idx)
  {
    int dim;
    int i;
    int vlen;
    coder::array<double, 1U> vwork;
    int vstride;
    int k;
    coder::array<int, 1U> iidx;
    dim = 0;
    if (x.size(0) != 1) {
      dim = -1;
    }

    if (dim + 2 <= 1) {
      i = x.size(0);
    } else {
      i = 1;
    }

    vlen = i - 1;
    vwork.set_size(i);
    idx.set_size(x.size(0));
    vstride = 1;
    for (k = 0; k <= dim; k++) {
      vstride *= x.size(0);
    }

    for (dim = 0; dim < vstride; dim++) {
      for (k = 0; k <= vlen; k++) {
        vwork[k] = x[dim + k * vstride];
      }

      d_sortIdx(vwork, iidx);
      for (k = 0; k <= vlen; k++) {
        i = dim + k * vstride;
        x[i] = vwork[k];
        idx[i] = iidx[k];
      }
    }
  }

  //
  // Arguments    : int x_data[]
  //                const int x_size[1]
  //                int idx_data[]
  //                int idx_size[1]
  // Return Type  : void
  //
  void sort(int x_data[], const int x_size[1], int idx_data[], int idx_size[1])
  {
    int dim;
    int i;
    int vlen;
    int vwork_size[1];
    int vstride;
    int k;
    int vwork_data[10001];
    int iidx_data[10001];
    int iidx_size[1];
    dim = 0;
    if (x_size[0] != 1) {
      dim = -1;
    }

    if (dim + 2 <= 1) {
      i = x_size[0];
    } else {
      i = 1;
    }

    vlen = i - 1;
    vwork_size[0] = i;
    idx_size[0] = x_size[0];
    vstride = 1;
    for (k = 0; k <= dim; k++) {
      vstride *= x_size[0];
    }

    for (dim = 0; dim < vstride; dim++) {
      for (k = 0; k <= vlen; k++) {
        vwork_data[k] = x_data[dim + k * vstride];
      }

      c_sortIdx(vwork_data, vwork_size, iidx_data, iidx_size);
      for (k = 0; k <= vlen; k++) {
        i = dim + k * vstride;
        x_data[i] = vwork_data[k];
        idx_data[i] = iidx_data[k];
      }
    }
  }
}

//
// File trailer for sort.cpp
//
// [EOF]
//
