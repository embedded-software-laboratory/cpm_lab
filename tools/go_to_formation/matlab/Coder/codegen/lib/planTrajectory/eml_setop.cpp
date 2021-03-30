//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_setop.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "eml_setop.h"
#include "DubinsBuiltins.h"
#include "Path1.h"
#include "mergesort.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "sortIdx.h"
#include <cmath>
#include <cstring>
#include <math.h>
#include <string.h>

// Function Definitions

//
// Arguments    : const unsigned int a_data[]
//                const int a_size[2]
//                const coder::array<double, 2U> *b
//                unsigned int c_data[]
//                int c_size[2]
//                int ia_data[]
//                int ia_size[1]
//                int ib_data[]
//                int ib_size[1]
// Return Type  : void
//
namespace mgen
{
  void do_vectors(const unsigned int a_data[], const int a_size[2], const coder::
                  array<double, 2U> &b, unsigned int c_data[], int c_size[2],
                  int ia_data[], int ia_size[1], int ib_data[], int ib_size[1])
  {
    int iafirst;
    int ncmax;
    int ialast;
    static int aperm_data[10001];
    coder::array<int, 2U> bperm;
    int nc;
    int ibfirst;
    int iblast;
    int iidx_size[1];
    static int b_ib_data[10001];
    int exponent;
    int b_exponent;
    iafirst = a_size[1];
    ncmax = b.size(1);
    if (iafirst < ncmax) {
      ncmax = iafirst;
    }

    c_size[0] = 1;
    c_size[1] = ncmax;
    ia_size[0] = static_cast<short>(ncmax);
    ialast = a_size[1];
    if (0 <= ialast - 1) {
      std::memset(&aperm_data[0], 0, ialast * sizeof(int));
    }

    if (a_size[1] != 0) {
      b_mergesort(aperm_data, a_data, a_size[1]);
    }

    b_sortIdx(b, bperm);
    nc = 0;
    iafirst = 0;
    ialast = 1;
    ibfirst = 0;
    iblast = 1;
    while ((ialast <= a_size[1]) && (iblast <= b.size(1))) {
      int b_ialast;
      unsigned int ak;
      int b_iblast;
      double bk;
      boolean_T exitg1;
      double absx;
      b_ialast = ialast;
      ak = a_data[aperm_data[ialast - 1] - 1];
      while ((b_ialast < a_size[1]) && (a_data[aperm_data[b_ialast] - 1] == ak))
      {
        b_ialast++;
      }

      ialast = b_ialast;
      b_iblast = iblast;
      bk = b[bperm[iblast - 1] - 1];
      exitg1 = false;
      while ((!exitg1) && (b_iblast < b.size(1))) {
        double d;
        absx = std::abs(bk / 2.0);
        if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
          if (absx <= 2.2250738585072014E-308) {
            absx = 4.94065645841247E-324;
          } else {
            frexp(absx, &exponent);
            absx = std::ldexp(1.0, exponent - 53);
          }
        } else {
          absx = rtNaN;
        }

        d = b[bperm[b_iblast] - 1];
        if ((std::abs(bk - d) < absx) || (rtIsInf(d) && rtIsInf(bk) && ((d > 0.0)
              == (bk > 0.0)))) {
          b_iblast++;
        } else {
          exitg1 = true;
        }
      }

      iblast = b_iblast;
      absx = std::abs(bk / 2.0);
      if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
        if (absx <= 2.2250738585072014E-308) {
          absx = 4.94065645841247E-324;
        } else {
          frexp(absx, &b_exponent);
          absx = std::ldexp(1.0, b_exponent - 53);
        }
      } else {
        absx = rtNaN;
      }

      if (std::abs(bk - static_cast<double>(ak)) < absx) {
        nc++;
        ia_data[nc - 1] = aperm_data[iafirst];
        ib_data[nc - 1] = bperm[ibfirst];
        ialast = b_ialast + 1;
        iafirst = b_ialast;
        iblast = b_iblast + 1;
        ibfirst = b_iblast;
      } else if (rtIsNaN(bk) || (ak < bk)) {
        ialast = b_ialast + 1;
        iafirst = b_ialast;
      } else {
        iblast = b_iblast + 1;
        ibfirst = b_iblast;
      }
    }

    if (ncmax > 0) {
      if (1 > nc) {
        ia_size[0] = 0;
      } else {
        ia_size[0] = nc;
      }
    }

    sort(ia_data, ia_size, aperm_data, iidx_size);
    for (iafirst = 0; iafirst < nc; iafirst++) {
      c_data[iafirst] = a_data[ia_data[iafirst] - 1];
    }

    iafirst = iidx_size[0];
    ialast = iidx_size[0];
    for (ibfirst = 0; ibfirst < ialast; ibfirst++) {
      b_ib_data[ibfirst] = ib_data[aperm_data[ibfirst] - 1];
    }

    ib_size[0] = iidx_size[0];
    if (0 <= iafirst - 1) {
      std::memcpy(&ib_data[0], &b_ib_data[0], iafirst * sizeof(int));
    }

    if (ncmax > 0) {
      if (1 > nc) {
        c_size[1] = 0;
      } else {
        c_size[1] = nc;
      }
    }
  }
}

//
// File trailer for eml_setop.cpp
//
// [EOF]
//
