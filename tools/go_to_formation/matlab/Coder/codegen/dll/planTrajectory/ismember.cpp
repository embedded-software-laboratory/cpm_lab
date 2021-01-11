//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ismember.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "ismember.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "Path1.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Function Declarations
namespace mgen
{
  static int b_bsearchni(int k, const coder::array<double, 2U> &x, const coder::
    array<double, 2U> &s);
  static int bsearchni(int k, const coder::array<double, 2U> &x, const coder::
                       array<double, 1U> &s);
}

// Function Definitions

//
// Arguments    : int k
//                const coder::array<double, 2U> &x
//                const coder::array<double, 2U> &s
// Return Type  : int
//
namespace mgen
{
  static int b_bsearchni(int k, const coder::array<double, 2U> &x, const coder::
    array<double, 2U> &s)
  {
    int idx;
    double b_x;
    int ihi;
    int ilo;
    boolean_T exitg1;
    b_x = x[k - 1];
    ihi = s.size(0) * 3;
    idx = 0;
    ilo = 1;
    exitg1 = false;
    while ((!exitg1) && (ihi >= ilo)) {
      int imid;
      imid = ((ilo >> 1) + (ihi >> 1)) - 1;
      if (((ilo & 1) == 1) && ((ihi & 1) == 1)) {
        imid++;
      }

      if (b_x == s[imid]) {
        idx = imid + 1;
        exitg1 = true;
      } else {
        boolean_T p;
        if (rtIsNaN(s[imid])) {
          p = !rtIsNaN(b_x);
        } else {
          p = ((!rtIsNaN(b_x)) && (b_x < s[imid]));
        }

        if (p) {
          ihi = imid;
        } else {
          ilo = imid + 2;
        }
      }
    }

    if (idx > 0) {
      idx--;
      while ((idx > 0) && (b_x == s[idx - 1])) {
        idx--;
      }

      idx++;
    }

    return idx;
  }

  //
  // Arguments    : int k
  //                const coder::array<double, 2U> &x
  //                const coder::array<double, 1U> &s
  // Return Type  : int
  //
  static int bsearchni(int k, const coder::array<double, 2U> &x, const coder::
                       array<double, 1U> &s)
  {
    int idx;
    double b_x;
    int ihi;
    int ilo;
    boolean_T exitg1;
    b_x = x[k - 1];
    ihi = s.size(0);
    idx = 0;
    ilo = 1;
    exitg1 = false;
    while ((!exitg1) && (ihi >= ilo)) {
      int imid;
      imid = ((ilo >> 1) + (ihi >> 1)) - 1;
      if (((ilo & 1) == 1) && ((ihi & 1) == 1)) {
        imid++;
      }

      if (b_x == s[imid]) {
        idx = imid + 1;
        exitg1 = true;
      } else {
        boolean_T p;
        if (rtIsNaN(s[imid])) {
          p = !rtIsNaN(b_x);
        } else {
          p = ((!rtIsNaN(b_x)) && (b_x < s[imid]));
        }

        if (p) {
          ihi = imid;
        } else {
          ilo = imid + 2;
        }
      }
    }

    if (idx > 0) {
      idx--;
      while ((idx > 0) && (b_x == s[idx - 1])) {
        idx--;
      }

      idx++;
    }

    return idx;
  }

  //
  // Arguments    : const coder::array<double, 2U> *a
  //                const coder::array<double, 2U> *s
  //                coder::array<boolean_T, 2U> *tf
  // Return Type  : void
  //
  void local_ismember(const coder::array<double, 2U> &a, const coder::array<
                      double, 2U> &s, coder::array<boolean_T, 2U> &tf)
  {
    int na;
    int ns;
    int n;
    int pmax;
    boolean_T guard1 = false;
    boolean_T exitg1;
    int pow2p;
    double absx;
    int exponent;
    coder::array<double, 1U> ss;
    coder::array<int, 1U> b_ss;
    int b_n;
    int subs[2];
    na = a.size(0) * 3 - 1;
    ns = s.size(0) * 3;
    n = a.size(0);
    tf.set_size(n, 3);
    pmax = n * 3;
    for (n = 0; n < pmax; n++) {
      tf[n] = false;
    }

    guard1 = false;
    if (ns <= 4) {
      guard1 = true;
    } else {
      int pmin;
      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        n = (pmin + pmax) >> 1;
        pow2p = 1 << n;
        if (pow2p == ns) {
          pmax = n;
          exitg1 = true;
        } else if (pow2p > ns) {
          pmax = n;
        } else {
          pmin = n;
        }
      }

      if (na + 1 <= pmax + 4) {
        guard1 = true;
      } else {
        boolean_T y;
        y = true;
        pmax = 2;
        if (s.size(0) * 3 != 1) {
          pmax = 1;
        }

        if (s.size(0) * 3 != 0) {
          if (pmax <= 1) {
            ns = s.size(0) * 3;
          } else {
            ns = 1;
          }

          if (ns != 1) {
            if (pmax == 2) {
              pmin = -1;
            } else {
              pmin = 0;
            }

            pow2p = 0;
            exitg1 = false;
            while ((!exitg1) && (pow2p <= pmin)) {
              boolean_T exitg2;
              if (pmax == 1) {
                n = s.size(0) * 3 - 1;
              } else {
                n = s.size(0) * 3;
              }

              pow2p = 0;
              exitg2 = false;
              while ((!exitg2) && (pow2p <= n - 1)) {
                subs[0] = pow2p + 1;
                subs[1] = 1;
                subs[pmax - 1]++;
                absx = s[subs[0] - 1];
                if ((!(s[pow2p] <= absx)) && (!rtIsNaN(absx))) {
                  y = false;
                }

                if (!y) {
                  exitg2 = true;
                } else {
                  pow2p++;
                }
              }

              if (!y) {
                exitg1 = true;
              } else {
                pow2p = 1;
              }
            }
          }
        }

        if (!y) {
          ss.set_size((s.size(0) * 3));
          pmax = s.size(0) * 3;
          for (n = 0; n < pmax; n++) {
            ss[n] = s[n];
          }

          b_sort(ss, b_ss);

#pragma omp parallel for \
 num_threads(omp_get_max_threads()) \
 private(b_n)

          for (int k = 0; k <= na; k++) {
            b_n = bsearchni(k + 1, a, ss);
            if (b_n > 0) {
              tf[k] = true;
            }
          }
        } else {

#pragma omp parallel for \
 num_threads(omp_get_max_threads()) \
 private(b_n)

          for (int k = 0; k <= na; k++) {
            b_n = b_bsearchni(k + 1, a, s);
            if (b_n > 0) {
              tf[k] = true;
            }
          }
        }
      }
    }

    if (guard1) {
      for (n = 0; n <= na; n++) {
        pow2p = 0;
        exitg1 = false;
        while ((!exitg1) && (pow2p <= ns - 1)) {
          absx = std::abs(s[pow2p] / 2.0);
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

          if ((std::abs(s[pow2p] - a[n]) < absx) || (rtIsInf(a[n]) && rtIsInf
               (s[pow2p]) && ((a[n] > 0.0) == (s[pow2p] > 0.0)))) {
            tf[n] = true;
            exitg1 = true;
          } else {
            pow2p++;
          }
        }
      }
    }
  }
}

//
// File trailer for ismember.cpp
//
// [EOF]
//
