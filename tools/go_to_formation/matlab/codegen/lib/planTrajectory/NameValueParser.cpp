//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: NameValueParser.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//

// Include Files
#include "NameValueParser.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : mgen::d_matlabshared_autonomous_core_ *
//
namespace mgen
{
  d_matlabshared_autonomous_core_ *d_matlabshared_autonomous_core_::init()
  {
    d_matlabshared_autonomous_core_ *obj;
    cell_wrap_8 r;
    static const char cv[7] = { 'o', 'p', 't', 'i', 'm', 'a', 'l' };

    obj = this;
    for (int i = 0; i < 7; i++) {
      r.f1[i] = cv[i];
    }

    obj->Defaults[0] = r;
    return obj;
  }

  //
  // Arguments    : void
  // Return Type  : c_matlabshared_autonomous_core_ *
  //
  c_matlabshared_autonomous_core_ *c_matlabshared_autonomous_core_::init()
  {
    c_matlabshared_autonomous_core_ *obj;
    obj = this;
    obj->Defaults.f1 = 1.0;
    return obj;
  }

  //
  // Arguments    : char value[7]
  // Return Type  : void
  //
  void d_matlabshared_autonomous_core_::parameterValue(char value[7]) const
  {
    for (int i = 0; i < 7; i++) {
      value[i] = this->ParsedResults[0].f1[i];
    }
  }

  //
  // Arguments    : void
  // Return Type  : double
  //
  double c_matlabshared_autonomous_core_::parameterValue() const
  {
    return this->ParsedResults.f1;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void d_matlabshared_autonomous_core_::parse()
  {
    for (int i = 0; i < 7; i++) {
      this->ParsedResults[0].f1[i] = this->Defaults[0].f1[i];
    }
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  void c_matlabshared_autonomous_core_::parse()
  {
    this->ParsedResults.f1 = this->Defaults.f1;
  }
}

//
// File trailer for NameValueParser.cpp
//
// [EOF]
//
