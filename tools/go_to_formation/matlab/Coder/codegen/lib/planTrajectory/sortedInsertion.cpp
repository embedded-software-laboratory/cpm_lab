//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortedInsertion.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 15-Feb-2021 16:41:56
//

// Include Files
#include "sortedInsertion.h"
#include "DubinsPathSegment.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : double x
//                int ix
//                coder::array<double, 1U> *b
//                int *nb
//                int blen
//                int idx_data[]
// Return Type  : void
//
namespace mgen
{
  void sortedInsertion(double x, int ix, coder::array<double, 1U> &b, int *nb,
                       int blen, int idx_data[])
  {
    if (*nb == 0) {
      *nb = 1;
      idx_data[0] = ix;
      b[0] = x;
    } else if ((x >= b[0]) || rtIsNaN(x)) {
      if ((*nb > 1) && (!(x >= b[*nb - 1])) && (!rtIsNaN(x))) {
        int ja;
        int jc;
        int jb;
        ja = 1;
        jb = *nb;
        while (ja < jb) {
          jc = ja + ((jb - ja) >> 1);
          if (jc == ja) {
            ja = jb;
          } else if ((x >= b[jc - 1]) || rtIsNaN(x)) {
            ja = jc;
          } else {
            jb = jc;
          }
        }

        if (*nb < blen) {
          (*nb)++;
        }

        jb = ja + 1;
        for (jc = *nb; jc >= jb; jc--) {
          b[jc - 1] = b[jc - 2];
          idx_data[jc - 1] = idx_data[jc - 2];
        }

        b[ja - 1] = x;
        idx_data[ja - 1] = ix;
      } else {
        if (*nb < blen) {
          (*nb)++;
          b[*nb - 1] = x;
          idx_data[*nb - 1] = ix;
        }
      }
    } else {
      if (*nb < blen) {
        (*nb)++;
      }

      for (int jc = *nb; jc >= 2; jc--) {
        idx_data[jc - 1] = idx_data[jc - 2];
        b[jc - 1] = b[jc - 2];
      }

      b[0] = x;
      idx_data[0] = ix;
    }
  }
}

//
// File trailer for sortedInsertion.cpp
//
// [EOF]
//
