//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: extremeKElements.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "extremeKElements.h"
#include "DubinsPathSegment.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "sortedInsertion.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const coder::array<double, 1U> *a
//                int k
//                int idx_data[]
//                int idx_size[1]
//                coder::array<double, 1U> *b
// Return Type  : void
//
namespace mgen
{
  void exkib(const coder::array<double, 1U> &a, int k, int idx_data[], int
             idx_size[1], coder::array<double, 1U> &b)
  {
    int n;
    int i;
    int itmp_data[10002];
    int itmp_size[1];
    int unusedU1;
    n = a.size(0);
    idx_size[0] = k;
    b.set_size(k);
    for (i = 0; i < k; i++) {
      idx_data[i] = 0;
      b[i] = 0.0;
    }

    if (k != 0) {
      if ((k > 64) && (k > (a.size(0) >> 6))) {
        sortIdx(a, itmp_data, itmp_size);
        idx_size[0] = k;
        b.set_size(k);
        for (int j = 0; j < k; j++) {
          idx_data[j] = itmp_data[j];
          b[j] = a[itmp_data[j] - 1];
        }
      } else {
        int j;
        for (j = 0; j < k; j++) {
          unusedU1 = j;
          sortedInsertion(a[j], j + 1, b, &unusedU1, k, idx_data);
        }

        i = k + 1;
        for (j = i; j <= n; j++) {
          unusedU1 = k;
          sortedInsertion(a[j - 1], j, b, &unusedU1, k, idx_data);
        }
      }
    }
  }
}

//
// File trailer for extremeKElements.cpp
//
// [EOF]
//
