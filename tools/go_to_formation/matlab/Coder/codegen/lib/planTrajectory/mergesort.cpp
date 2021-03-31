//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mergesort.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "mergesort.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : int idx_data[]
//                const unsigned int x_data[]
//                int n
// Return Type  : void
//
namespace mgen
{
  void b_mergesort(int idx_data[], const unsigned int x_data[], int n)
  {
    int i;
    int k;
    int b_i;
    int qEnd;
    int iwork_data[10001];
    i = n - 1;
    for (k = 1; k <= i; k += 2) {
      if (x_data[k - 1] <= x_data[k]) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }

    if ((n & 1) != 0) {
      idx_data[n - 1] = n;
    }

    b_i = 2;
    while (b_i < n) {
      int i2;
      int j;
      i2 = b_i << 1;
      j = 1;
      for (int pEnd = b_i + 1; pEnd < n + 1; pEnd = qEnd + b_i) {
        int p;
        int q;
        int kEnd;
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n + 1) {
          qEnd = n + 1;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          int i1;
          i = idx_data[q - 1];
          i1 = idx_data[p - 1];
          if (x_data[i1 - 1] <= x_data[i - 1]) {
            iwork_data[k] = i1;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[k] = i;
            q++;
            if (q == qEnd) {
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
// File trailer for mergesort.cpp
//
// [EOF]
//
