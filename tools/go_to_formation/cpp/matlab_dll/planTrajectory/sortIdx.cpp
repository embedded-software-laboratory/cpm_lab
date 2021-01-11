//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortIdx.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
//

// Include Files
#include "sortIdx.h"
#include "DubinsBuiltins.h"
#include "DubinsPathSegment.h"
#include "Path1.h"
#include "eml_setop.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include <cstring>
#include <string.h>

// Function Declarations
namespace mgen
{
  static void b_merge(coder::array<int, 1U> &idx, coder::array<double, 1U> &x,
                      int offset, int np, int nq, coder::array<int, 1U> &iwork,
                      coder::array<double, 1U> &xwork);
  static void b_merge_block(coder::array<int, 1U> &idx, coder::array<double, 1U>
    &x, int offset, int n, int preSortLevel, coder::array<int, 1U> &iwork, coder::
    array<double, 1U> &xwork);
  static void merge(int idx_data[], int x_data[], int offset, int np, int nq,
                    int iwork_data[], int xwork_data[]);
  static void merge_block(int idx_data[], int x_data[], int offset, int n, int
    preSortLevel, int iwork_data[], int xwork_data[]);
  static void merge_pow2_block(int idx_data[], int x_data[], int offset);
}

// Function Definitions

//
// Arguments    : coder::array<int, 1U> &idx
//                coder::array<double, 1U> &x
//                int offset
//                int np
//                int nq
//                coder::array<int, 1U> &iwork
//                coder::array<double, 1U> &xwork
// Return Type  : void
//
namespace mgen
{
  static void b_merge(coder::array<int, 1U> &idx, coder::array<double, 1U> &x,
                      int offset, int np, int nq, coder::array<int, 1U> &iwork,
                      coder::array<double, 1U> &xwork)
  {
    if (nq != 0) {
      int n_tmp;
      int j;
      int p;
      int iout;
      int q;
      n_tmp = np + nq;
      for (j = 0; j < n_tmp; j++) {
        iout = offset + j;
        iwork[j] = idx[iout];
        xwork[j] = x[iout];
      }

      p = 0;
      q = np;
      iout = offset - 1;
      int exitg1;
      do {
        exitg1 = 0;
        iout++;
        if (xwork[p] <= xwork[q]) {
          idx[iout] = iwork[p];
          x[iout] = xwork[p];
          if (p + 1 < np) {
            p++;
          } else {
            exitg1 = 1;
          }
        } else {
          idx[iout] = iwork[q];
          x[iout] = xwork[q];
          if (q + 1 < n_tmp) {
            q++;
          } else {
            q = iout - p;
            for (j = p + 1; j <= np; j++) {
              iout = q + j;
              idx[iout] = iwork[j - 1];
              x[iout] = xwork[j - 1];
            }

            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }

  //
  // Arguments    : coder::array<int, 1U> &idx
  //                coder::array<double, 1U> &x
  //                int offset
  //                int n
  //                int preSortLevel
  //                coder::array<int, 1U> &iwork
  //                coder::array<double, 1U> &xwork
  // Return Type  : void
  //
  static void b_merge_block(coder::array<int, 1U> &idx, coder::array<double, 1U>
    &x, int offset, int n, int preSortLevel, coder::array<int, 1U> &iwork, coder::
    array<double, 1U> &xwork)
  {
    int nPairs;
    int bLen;
    nPairs = n >> preSortLevel;
    bLen = 1 << preSortLevel;
    while (nPairs > 1) {
      int tailOffset;
      int nTail;
      if ((nPairs & 1) != 0) {
        nPairs--;
        tailOffset = bLen * nPairs;
        nTail = n - tailOffset;
        if (nTail > bLen) {
          b_merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
        }
      }

      tailOffset = bLen << 1;
      nPairs >>= 1;
      for (nTail = 0; nTail < nPairs; nTail++) {
        b_merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
      }

      bLen = tailOffset;
    }

    if (n > bLen) {
      b_merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
    }
  }

  //
  // Arguments    : int idx_data[]
  //                int x_data[]
  //                int offset
  //                int np
  //                int nq
  //                int iwork_data[]
  //                int xwork_data[]
  // Return Type  : void
  //
  static void merge(int idx_data[], int x_data[], int offset, int np, int nq,
                    int iwork_data[], int xwork_data[])
  {
    if (nq != 0) {
      int n_tmp;
      int j;
      int p;
      int iout;
      int q;
      n_tmp = np + nq;
      for (j = 0; j < n_tmp; j++) {
        iout = offset + j;
        iwork_data[j] = idx_data[iout];
        xwork_data[j] = x_data[iout];
      }

      p = 0;
      q = np;
      iout = offset - 1;
      int exitg1;
      do {
        exitg1 = 0;
        iout++;
        if (xwork_data[p] <= xwork_data[q]) {
          idx_data[iout] = iwork_data[p];
          x_data[iout] = xwork_data[p];
          if (p + 1 < np) {
            p++;
          } else {
            exitg1 = 1;
          }
        } else {
          idx_data[iout] = iwork_data[q];
          x_data[iout] = xwork_data[q];
          if (q + 1 < n_tmp) {
            q++;
          } else {
            q = iout - p;
            for (j = p + 1; j <= np; j++) {
              iout = q + j;
              idx_data[iout] = iwork_data[j - 1];
              x_data[iout] = xwork_data[j - 1];
            }

            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }

  //
  // Arguments    : int idx_data[]
  //                int x_data[]
  //                int offset
  //                int n
  //                int preSortLevel
  //                int iwork_data[]
  //                int xwork_data[]
  // Return Type  : void
  //
  static void merge_block(int idx_data[], int x_data[], int offset, int n, int
    preSortLevel, int iwork_data[], int xwork_data[])
  {
    int nPairs;
    int bLen;
    nPairs = n >> preSortLevel;
    bLen = 1 << preSortLevel;
    while (nPairs > 1) {
      int tailOffset;
      int nTail;
      if ((nPairs & 1) != 0) {
        nPairs--;
        tailOffset = bLen * nPairs;
        nTail = n - tailOffset;
        if (nTail > bLen) {
          merge(idx_data, x_data, offset + tailOffset, bLen, nTail - bLen,
                iwork_data, xwork_data);
        }
      }

      tailOffset = bLen << 1;
      nPairs >>= 1;
      for (nTail = 0; nTail < nPairs; nTail++) {
        merge(idx_data, x_data, offset + nTail * tailOffset, bLen, bLen,
              iwork_data, xwork_data);
      }

      bLen = tailOffset;
    }

    if (n > bLen) {
      merge(idx_data, x_data, offset, bLen, n - bLen, iwork_data, xwork_data);
    }
  }

  //
  // Arguments    : int idx_data[]
  //                int x_data[]
  //                int offset
  // Return Type  : void
  //
  static void merge_pow2_block(int idx_data[], int x_data[], int offset)
  {
    int iout;
    int iwork[256];
    int xwork[256];
    for (int b = 0; b < 6; b++) {
      int bLen;
      int bLen2;
      int nPairs;
      bLen = 1 << (b + 2);
      bLen2 = bLen << 1;
      nPairs = 256 >> (b + 3);
      for (int k = 0; k < nPairs; k++) {
        int blockOffset;
        int j;
        int p;
        int q;
        blockOffset = offset + k * bLen2;
        for (j = 0; j < bLen2; j++) {
          iout = blockOffset + j;
          iwork[j] = idx_data[iout];
          xwork[j] = x_data[iout];
        }

        p = 0;
        q = bLen;
        iout = blockOffset - 1;
        int exitg1;
        do {
          exitg1 = 0;
          iout++;
          if (xwork[p] <= xwork[q]) {
            idx_data[iout] = iwork[p];
            x_data[iout] = xwork[p];
            if (p + 1 < bLen) {
              p++;
            } else {
              exitg1 = 1;
            }
          } else {
            idx_data[iout] = iwork[q];
            x_data[iout] = xwork[q];
            if (q + 1 < bLen2) {
              q++;
            } else {
              iout -= p;
              for (j = p + 1; j <= bLen; j++) {
                q = iout + j;
                idx_data[q] = iwork[j - 1];
                x_data[q] = xwork[j - 1];
              }

              exitg1 = 1;
            }
          }
        } while (exitg1 == 0);
      }
    }
  }

  //
  // Arguments    : const coder::array<double, 2U> *x
  //                coder::array<int, 2U> *idx
  // Return Type  : void
  //
  void b_sortIdx(const coder::array<double, 2U> &x, coder::array<int, 2U> &idx)
  {
    int n;
    int i;
    int b_i;
    coder::array<int, 1U> iwork;
    int k;
    int qEnd;
    n = x.size(1) + 1;
    idx.set_size(1, x.size(1));
    i = x.size(1);
    for (b_i = 0; b_i < i; b_i++) {
      idx[b_i] = 0;
    }

    if (x.size(1) != 0) {
      double d;
      iwork.set_size(x.size(1));
      b_i = x.size(1) - 1;
      for (k = 1; k <= b_i; k += 2) {
        d = x[k];
        if ((x[k - 1] <= d) || rtIsNaN(d)) {
          idx[k - 1] = k;
          idx[k] = k + 1;
        } else {
          idx[k - 1] = k + 1;
          idx[k] = k;
        }
      }

      if ((x.size(1) & 1) != 0) {
        idx[x.size(1) - 1] = x.size(1);
      }

      i = 2;
      while (i < n - 1) {
        int i2;
        int j;
        i2 = i << 1;
        j = 1;
        for (int pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
          int p;
          int q;
          int kEnd;
          p = j;
          q = pEnd - 1;
          qEnd = j + i2;
          if (qEnd > n) {
            qEnd = n;
          }

          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            d = x[idx[q] - 1];
            b_i = idx[p - 1];
            if ((x[b_i - 1] <= d) || rtIsNaN(d)) {
              iwork[k] = b_i;
              p++;
              if (p == pEnd) {
                while (q + 1 < qEnd) {
                  k++;
                  iwork[k] = idx[q];
                  q++;
                }
              }
            } else {
              iwork[k] = idx[q];
              q++;
              if (q + 1 == qEnd) {
                while (p < pEnd) {
                  k++;
                  iwork[k] = idx[p - 1];
                  p++;
                }
              }
            }

            k++;
          }

          for (k = 0; k < kEnd; k++) {
            idx[(j + k) - 1] = iwork[k];
          }

          j = qEnd;
        }

        i = i2;
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
  void c_sortIdx(int x_data[], const int x_size[1], int idx_data[], int
                 idx_size[1])
  {
    short unnamed_idx_0;
    int i3;
    int x4[4];
    short idx4[4];
    int iwork_data[10001];
    int xwork_data[10001];
    signed char perm[4];
    unnamed_idx_0 = static_cast<short>(x_size[0]);
    idx_size[0] = unnamed_idx_0;
    i3 = unnamed_idx_0;
    if (0 <= i3 - 1) {
      std::memset(&idx_data[0], 0, i3 * sizeof(int));
    }

    if (x_size[0] != 0) {
      int n;
      int nQuartets;
      int i4;
      int nLeft;
      int k;
      int i1;
      int i2;
      n = x_size[0];
      x4[0] = 0;
      idx4[0] = 0;
      x4[1] = 0;
      idx4[1] = 0;
      x4[2] = 0;
      idx4[2] = 0;
      x4[3] = 0;
      idx4[3] = 0;
      i3 = unnamed_idx_0;
      if (0 <= i3 - 1) {
        std::memset(&iwork_data[0], 0, i3 * sizeof(int));
      }

      i3 = x_size[0];
      if (0 <= i3 - 1) {
        std::memset(&xwork_data[0], 0, i3 * sizeof(int));
      }

      nQuartets = x_size[0] >> 2;
      for (int j = 0; j < nQuartets; j++) {
        int i;
        i = j << 2;
        idx4[0] = static_cast<short>(i + 1);
        idx4[1] = static_cast<short>(i + 2);
        idx4[2] = static_cast<short>(i + 3);
        idx4[3] = static_cast<short>(i + 4);
        x4[0] = x_data[i];
        i3 = x_data[i + 1];
        x4[1] = i3;
        i4 = x_data[i + 2];
        x4[2] = i4;
        nLeft = x_data[i + 3];
        x4[3] = nLeft;
        if (x_data[i] <= i3) {
          i1 = 1;
          i2 = 2;
        } else {
          i1 = 2;
          i2 = 1;
        }

        if (i4 <= nLeft) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        nLeft = x4[i1 - 1];
        k = x4[i3 - 1];
        if (nLeft <= k) {
          nLeft = x4[i2 - 1];
          if (nLeft <= k) {
            perm[0] = static_cast<signed char>(i1);
            perm[1] = static_cast<signed char>(i2);
            perm[2] = static_cast<signed char>(i3);
            perm[3] = static_cast<signed char>(i4);
          } else if (nLeft <= x4[i4 - 1]) {
            perm[0] = static_cast<signed char>(i1);
            perm[1] = static_cast<signed char>(i3);
            perm[2] = static_cast<signed char>(i2);
            perm[3] = static_cast<signed char>(i4);
          } else {
            perm[0] = static_cast<signed char>(i1);
            perm[1] = static_cast<signed char>(i3);
            perm[2] = static_cast<signed char>(i4);
            perm[3] = static_cast<signed char>(i2);
          }
        } else {
          k = x4[i4 - 1];
          if (nLeft <= k) {
            if (x4[i2 - 1] <= k) {
              perm[0] = static_cast<signed char>(i3);
              perm[1] = static_cast<signed char>(i1);
              perm[2] = static_cast<signed char>(i2);
              perm[3] = static_cast<signed char>(i4);
            } else {
              perm[0] = static_cast<signed char>(i3);
              perm[1] = static_cast<signed char>(i1);
              perm[2] = static_cast<signed char>(i4);
              perm[3] = static_cast<signed char>(i2);
            }
          } else {
            perm[0] = static_cast<signed char>(i3);
            perm[1] = static_cast<signed char>(i4);
            perm[2] = static_cast<signed char>(i1);
            perm[3] = static_cast<signed char>(i2);
          }
        }

        i1 = perm[0] - 1;
        idx_data[i] = idx4[i1];
        i2 = perm[1] - 1;
        idx_data[i + 1] = idx4[i2];
        i3 = perm[2] - 1;
        idx_data[i + 2] = idx4[i3];
        i4 = perm[3] - 1;
        idx_data[i + 3] = idx4[i4];
        x_data[i] = x4[i1];
        x_data[i + 1] = x4[i2];
        x_data[i + 2] = x4[i3];
        x_data[i + 3] = x4[i4];
      }

      i4 = nQuartets << 2;
      nLeft = (x_size[0] - i4) - 1;
      if (nLeft + 1 > 0) {
        for (k = 0; k <= nLeft; k++) {
          i3 = i4 + k;
          idx4[k] = static_cast<short>(i3 + 1);
          x4[k] = x_data[i3];
        }

        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (nLeft + 1 == 1) {
          perm[0] = 1;
        } else if (nLeft + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }

        for (k = 0; k <= nLeft; k++) {
          i1 = perm[k] - 1;
          i2 = i4 + k;
          idx_data[i2] = idx4[i1];
          x_data[i2] = x4[i1];
        }
      }

      i3 = 2;
      if (n > 1) {
        if (n >= 256) {
          i3 = n >> 8;
          for (i4 = 0; i4 < i3; i4++) {
            merge_pow2_block(idx_data, x_data, i4 << 8);
          }

          i3 <<= 8;
          i4 = n - i3;
          if (i4 > 0) {
            merge_block(idx_data, x_data, i3, i4, 2, iwork_data, xwork_data);
          }

          i3 = 8;
        }

        merge_block(idx_data, x_data, 0, n, i3, iwork_data, xwork_data);
      }
    }
  }

  //
  // Arguments    : coder::array<double, 1U> *x
  //                coder::array<int, 1U> *idx
  // Return Type  : void
  //
  void d_sortIdx(coder::array<double, 1U> &x, coder::array<int, 1U> &idx)
  {
    int i1;
    int ib;
    double x4[4];
    int idx4[4];
    coder::array<int, 1U> iwork;
    coder::array<double, 1U> xwork;
    signed char perm[4];
    int b_iwork[256];
    double b_xwork[256];
    i1 = x.size(0);
    idx.set_size(i1);
    for (ib = 0; ib < i1; ib++) {
      idx[ib] = 0;
    }

    if (x.size(0) != 0) {
      int n;
      int b_n;
      int bLen;
      int k;
      int i4;
      int idx_tmp;
      int quartetOffset;
      int i3;
      int nNonNaN;
      n = x.size(0);
      b_n = x.size(0);
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      iwork.set_size(i1);
      for (ib = 0; ib < i1; ib++) {
        iwork[ib] = 0;
      }

      i1 = x.size(0);
      xwork.set_size(i1);
      for (ib = 0; ib < i1; ib++) {
        xwork[ib] = 0.0;
      }

      bLen = 0;
      ib = -1;
      for (k = 0; k < b_n; k++) {
        if (rtIsNaN(x[k])) {
          idx_tmp = (b_n - bLen) - 1;
          idx[idx_tmp] = k + 1;
          xwork[idx_tmp] = x[k];
          bLen++;
        } else {
          ib++;
          idx4[ib] = k + 1;
          x4[ib] = x[k];
          if (ib + 1 == 4) {
            double d;
            double d1;
            quartetOffset = k - bLen;
            if (x4[0] <= x4[1]) {
              i1 = 1;
              ib = 2;
            } else {
              i1 = 2;
              ib = 1;
            }

            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }

            d = x4[i1 - 1];
            d1 = x4[i3 - 1];
            if (d <= d1) {
              d = x4[ib - 1];
              if (d <= d1) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(ib);
                perm[2] = static_cast<signed char>(i3);
                perm[3] = static_cast<signed char>(i4);
              } else if (d <= x4[i4 - 1]) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(ib);
                perm[3] = static_cast<signed char>(i4);
              } else {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(i4);
                perm[3] = static_cast<signed char>(ib);
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[ib - 1] <= d1) {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(ib);
                  perm[3] = static_cast<signed char>(i4);
                } else {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(i4);
                  perm[3] = static_cast<signed char>(ib);
                }
              } else {
                perm[0] = static_cast<signed char>(i3);
                perm[1] = static_cast<signed char>(i4);
                perm[2] = static_cast<signed char>(i1);
                perm[3] = static_cast<signed char>(ib);
              }
            }

            idx_tmp = perm[0] - 1;
            idx[quartetOffset - 3] = idx4[idx_tmp];
            i3 = perm[1] - 1;
            idx[quartetOffset - 2] = idx4[i3];
            ib = perm[2] - 1;
            idx[quartetOffset - 1] = idx4[ib];
            i1 = perm[3] - 1;
            idx[quartetOffset] = idx4[i1];
            x[quartetOffset - 3] = x4[idx_tmp];
            x[quartetOffset - 2] = x4[i3];
            x[quartetOffset - 1] = x4[ib];
            x[quartetOffset] = x4[i1];
            ib = -1;
          }
        }
      }

      i4 = (b_n - bLen) - 1;
      if (ib + 1 > 0) {
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (ib + 1 == 1) {
          perm[0] = 1;
        } else if (ib + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }

        for (k = 0; k <= ib; k++) {
          idx_tmp = perm[k] - 1;
          i3 = (i4 - ib) + k;
          idx[i3] = idx4[idx_tmp];
          x[i3] = x4[idx_tmp];
        }
      }

      ib = (bLen >> 1) + 1;
      for (k = 0; k <= ib - 2; k++) {
        i1 = (i4 + k) + 1;
        i3 = idx[i1];
        idx_tmp = (b_n - k) - 1;
        idx[i1] = idx[idx_tmp];
        idx[idx_tmp] = i3;
        x[i1] = xwork[idx_tmp];
        x[idx_tmp] = xwork[i1];
      }

      if ((bLen & 1) != 0) {
        ib += i4;
        x[ib] = xwork[ib];
      }

      nNonNaN = n - bLen;
      i1 = 2;
      if (nNonNaN > 1) {
        if (n >= 256) {
          int nBlocks;
          nBlocks = nNonNaN >> 8;
          if (nBlocks > 0) {
            for (int b = 0; b < nBlocks; b++) {
              quartetOffset = (b << 8) - 1;
              for (int b_b = 0; b_b < 6; b_b++) {
                bLen = 1 << (b_b + 2);
                b_n = bLen << 1;
                n = 256 >> (b_b + 3);
                for (k = 0; k < n; k++) {
                  i3 = (quartetOffset + k * b_n) + 1;
                  for (i1 = 0; i1 < b_n; i1++) {
                    ib = i3 + i1;
                    b_iwork[i1] = idx[ib];
                    b_xwork[i1] = x[ib];
                  }

                  i4 = 0;
                  i1 = bLen;
                  ib = i3 - 1;
                  int exitg1;
                  do {
                    exitg1 = 0;
                    ib++;
                    if (b_xwork[i4] <= b_xwork[i1]) {
                      idx[ib] = b_iwork[i4];
                      x[ib] = b_xwork[i4];
                      if (i4 + 1 < bLen) {
                        i4++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      idx[ib] = b_iwork[i1];
                      x[ib] = b_xwork[i1];
                      if (i1 + 1 < b_n) {
                        i1++;
                      } else {
                        ib -= i4;
                        for (i1 = i4 + 1; i1 <= bLen; i1++) {
                          idx_tmp = ib + i1;
                          idx[idx_tmp] = b_iwork[i1 - 1];
                          x[idx_tmp] = b_xwork[i1 - 1];
                        }

                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }

            i1 = nBlocks << 8;
            ib = nNonNaN - i1;
            if (ib > 0) {
              b_merge_block(idx, x, i1, ib, 2, iwork, xwork);
            }

            i1 = 8;
          }
        }

        b_merge_block(idx, x, 0, nNonNaN, i1, iwork, xwork);
      }
    }
  }

  //
  // Arguments    : const coder::array<double, 1U> *x
  //                int idx_data[]
  //                int idx_size[1]
  // Return Type  : void
  //
  void sortIdx(const coder::array<double, 1U> &x, int idx_data[], int idx_size[1])
  {
    int loop_ub;
    int i;
    int k;
    int b_i;
    int qEnd;
    int iwork_data[10002];
    idx_size[0] = static_cast<short>(x.size(0));
    loop_ub = static_cast<short>(x.size(0));
    if (0 <= loop_ub - 1) {
      std::memset(&idx_data[0], 0, loop_ub * sizeof(int));
    }

    loop_ub = x.size(0) + 1;
    i = x.size(0) - 1;
    for (k = 1; k <= i; k += 2) {
      if ((x[k - 1] <= x[k]) || rtIsNaN(x[k])) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }

    if ((x.size(0) & 1) != 0) {
      idx_data[x.size(0) - 1] = x.size(0);
    }

    b_i = 2;
    while (b_i < loop_ub - 1) {
      int i2;
      int j;
      i2 = b_i << 1;
      j = 1;
      for (int pEnd = b_i + 1; pEnd < loop_ub; pEnd = qEnd + b_i) {
        int p;
        int q;
        int kEnd;
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > loop_ub) {
          qEnd = loop_ub;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          double d;
          d = x[idx_data[q] - 1];
          i = idx_data[p - 1];
          if ((x[i - 1] <= d) || rtIsNaN(d)) {
            iwork_data[k] = i;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                k++;
                iwork_data[k] = idx_data[p - 1];
                p++;
              }
            }
          }

          k++;
        }

        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }

        j = qEnd;
      }

      b_i = i2;
    }
  }
}

//
// File trailer for sortIdx.cpp
//
// [EOF]
//
