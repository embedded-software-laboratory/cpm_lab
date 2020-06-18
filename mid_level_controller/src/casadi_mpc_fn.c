// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) casadi_mpc_fn_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sign CASADI_PREFIX(sign)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[11] = {1, 4, 0, 1, 2, 3, 4, 0, 0, 0, 0};
static const casadi_int casadi_s1[7] = {1, 2, 0, 1, 2, 0, 0};
static const casadi_int casadi_s2[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s3[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s4[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s5[5] = {1, 1, 0, 1, 0};

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_sign(casadi_real x) { return x<0 ? -1 : x>0 ? 1 : x;}

//Suppress warning for unused parameters (iw, w, mem)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/* casadi_mpc_fn:(var_x0[1x4],var_u0[1x2],var_u[3x3],var_momentum[3x3],var_params[10],var_reference_trajectory_x[6],var_reference_trajectory_y[6],var_learning_rate,var_momentum_rate)->(trajectory_x[6],trajectory_y[6],objective,var_momentum_next[3x3],var_u_next[3x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[0] ? arg[0][0] : 0;
  a1=5.0000000000000003e-02;
  a2=arg[4] ? arg[4][0] : 0;
  a3=arg[0] ? arg[0][3] : 0;
  a4=(a2*a3);
  a5=1.;
  a6=arg[4] ? arg[4][1] : 0;
  a7=arg[2] ? arg[2][3] : 0;
  a8=arg[4] ? arg[4][8] : 0;
  a9=(a7+a8);
  a10=casadi_sq(a9);
  a10=(a6*a10);
  a10=(a5+a10);
  a10=(a4*a10);
  a11=arg[0] ? arg[0][2] : 0;
  a12=arg[4] ? arg[4][2] : 0;
  a13=(a12*a9);
  a13=(a11+a13);
  a14=arg[4] ? arg[4][9] : 0;
  a13=(a13+a14);
  a15=cos(a13);
  a16=(a10*a15);
  a16=(a1*a16);
  a0=(a0+a16);
  if (res[0]!=0) res[0][0]=a0;
  a16=arg[4] ? arg[4][4] : 0;
  a17=(a16*a3);
  a18=arg[4] ? arg[4][5] : 0;
  a19=arg[4] ? arg[4][6] : 0;
  a20=arg[2] ? arg[2][6] : 0;
  a21=(a19*a20);
  a21=(a18+a21);
  a22=arg[2] ? arg[2][0] : 0;
  a23=casadi_sign(a22);
  a21=(a21*a23);
  a23=fabs(a22);
  a24=arg[4] ? arg[4][7] : 0;
  a25=pow(a23,a24);
  a25=(a21*a25);
  a17=(a17+a25);
  a17=(a1*a17);
  a17=(a3+a17);
  a25=(a2*a17);
  a26=(a7+a8);
  a27=casadi_sq(a26);
  a27=(a6*a27);
  a27=(a5+a27);
  a28=(a25*a27);
  a29=arg[4] ? arg[4][3] : 0;
  a30=(a29*a3);
  a31=(a30*a9);
  a31=(a1*a31);
  a31=(a11+a31);
  a32=(a12*a26);
  a32=(a31+a32);
  a32=(a32+a14);
  a33=cos(a32);
  a34=(a28*a33);
  a34=(a1*a34);
  a34=(a0+a34);
  if (res[0]!=0) res[0][1]=a34;
  a35=(a16*a17);
  a36=(a19*a20);
  a36=(a18+a36);
  a37=casadi_sign(a22);
  a36=(a36*a37);
  a37=fabs(a22);
  a38=pow(a37,a24);
  a38=(a36*a38);
  a35=(a35+a38);
  a35=(a1*a35);
  a35=(a17+a35);
  a38=(a2*a35);
  a39=arg[2] ? arg[2][4] : 0;
  a40=(a39+a8);
  a41=casadi_sq(a40);
  a41=(a6*a41);
  a41=(a5+a41);
  a42=(a38*a41);
  a43=(a29*a17);
  a44=(a43*a26);
  a44=(a1*a44);
  a44=(a31+a44);
  a45=(a12*a40);
  a45=(a44+a45);
  a45=(a45+a14);
  a46=cos(a45);
  a47=(a42*a46);
  a47=(a1*a47);
  a47=(a34+a47);
  if (res[0]!=0) res[0][2]=a47;
  a48=(a16*a35);
  a49=arg[2] ? arg[2][7] : 0;
  a50=(a19*a49);
  a50=(a18+a50);
  a51=arg[2] ? arg[2][1] : 0;
  a52=casadi_sign(a51);
  a50=(a50*a52);
  a52=fabs(a51);
  a53=pow(a52,a24);
  a53=(a50*a53);
  a48=(a48+a53);
  a48=(a1*a48);
  a48=(a35+a48);
  a53=(a2*a48);
  a54=(a39+a8);
  a55=casadi_sq(a54);
  a55=(a6*a55);
  a55=(a5+a55);
  a56=(a53*a55);
  a57=(a29*a35);
  a58=(a57*a40);
  a58=(a1*a58);
  a58=(a44+a58);
  a59=(a12*a54);
  a59=(a58+a59);
  a59=(a59+a14);
  a60=cos(a59);
  a61=(a56*a60);
  a61=(a1*a61);
  a61=(a47+a61);
  if (res[0]!=0) res[0][3]=a61;
  a62=(a16*a48);
  a63=(a19*a49);
  a63=(a18+a63);
  a64=casadi_sign(a51);
  a63=(a63*a64);
  a64=fabs(a51);
  a65=pow(a64,a24);
  a65=(a63*a65);
  a62=(a62+a65);
  a62=(a1*a62);
  a62=(a48+a62);
  a65=(a2*a62);
  a66=arg[2] ? arg[2][5] : 0;
  a67=(a66+a8);
  a68=casadi_sq(a67);
  a68=(a6*a68);
  a68=(a5+a68);
  a69=(a65*a68);
  a70=(a29*a48);
  a71=(a70*a54);
  a71=(a1*a71);
  a71=(a58+a71);
  a72=(a12*a67);
  a72=(a71+a72);
  a72=(a72+a14);
  a73=cos(a72);
  a74=(a69*a73);
  a74=(a1*a74);
  a74=(a61+a74);
  if (res[0]!=0) res[0][4]=a74;
  a75=(a16*a62);
  a76=arg[2] ? arg[2][8] : 0;
  a19=(a19*a76);
  a18=(a18+a19);
  a19=arg[2] ? arg[2][2] : 0;
  a77=casadi_sign(a19);
  a18=(a18*a77);
  a77=fabs(a19);
  a78=pow(a77,a24);
  a78=(a18*a78);
  a75=(a75+a78);
  a75=(a1*a75);
  a75=(a62+a75);
  a78=(a2*a75);
  a8=(a66+a8);
  a79=casadi_sq(a8);
  a79=(a6*a79);
  a79=(a5+a79);
  a80=(a78*a79);
  a81=(a29*a62);
  a82=(a81*a67);
  a82=(a1*a82);
  a82=(a71+a82);
  a83=(a12*a8);
  a83=(a82+a83);
  a83=(a83+a14);
  a84=cos(a83);
  a85=(a80*a84);
  a85=(a1*a85);
  a85=(a74+a85);
  if (res[0]!=0) res[0][5]=a85;
  a86=arg[0] ? arg[0][1] : 0;
  a3=(a2*a3);
  a87=casadi_sq(a9);
  a87=(a6*a87);
  a87=(a5+a87);
  a87=(a3*a87);
  a88=(a12*a9);
  a11=(a11+a88);
  a11=(a11+a14);
  a88=sin(a11);
  a89=(a87*a88);
  a89=(a1*a89);
  a86=(a86+a89);
  if (res[1]!=0) res[1][0]=a86;
  a17=(a2*a17);
  a89=casadi_sq(a26);
  a89=(a6*a89);
  a89=(a5+a89);
  a90=(a17*a89);
  a91=(a12*a26);
  a31=(a31+a91);
  a31=(a31+a14);
  a91=sin(a31);
  a92=(a90*a91);
  a92=(a1*a92);
  a92=(a86+a92);
  if (res[1]!=0) res[1][1]=a92;
  a35=(a2*a35);
  a93=casadi_sq(a40);
  a93=(a6*a93);
  a93=(a5+a93);
  a94=(a35*a93);
  a95=(a12*a40);
  a44=(a44+a95);
  a44=(a44+a14);
  a95=sin(a44);
  a96=(a94*a95);
  a96=(a1*a96);
  a96=(a92+a96);
  if (res[1]!=0) res[1][2]=a96;
  a48=(a2*a48);
  a97=casadi_sq(a54);
  a97=(a6*a97);
  a97=(a5+a97);
  a98=(a48*a97);
  a99=(a12*a54);
  a58=(a58+a99);
  a58=(a58+a14);
  a99=sin(a58);
  a100=(a98*a99);
  a100=(a1*a100);
  a100=(a96+a100);
  if (res[1]!=0) res[1][3]=a100;
  a62=(a2*a62);
  a101=casadi_sq(a67);
  a101=(a6*a101);
  a101=(a5+a101);
  a102=(a62*a101);
  a103=(a12*a67);
  a71=(a71+a103);
  a71=(a71+a14);
  a103=sin(a71);
  a104=(a102*a103);
  a104=(a1*a104);
  a104=(a100+a104);
  if (res[1]!=0) res[1][4]=a104;
  a75=(a2*a75);
  a105=casadi_sq(a8);
  a105=(a6*a105);
  a105=(a5+a105);
  a106=(a75*a105);
  a107=(a12*a8);
  a82=(a82+a107);
  a82=(a82+a14);
  a14=sin(a82);
  a107=(a106*a14);
  a107=(a1*a107);
  a107=(a104+a107);
  if (res[1]!=0) res[1][5]=a107;
  a108=arg[5] ? arg[5][0] : 0;
  a0=(a0-a108);
  a108=casadi_sq(a0);
  a109=arg[5] ? arg[5][1] : 0;
  a34=(a34-a109);
  a109=casadi_sq(a34);
  a108=(a108+a109);
  a109=arg[5] ? arg[5][2] : 0;
  a47=(a47-a109);
  a109=casadi_sq(a47);
  a108=(a108+a109);
  a109=arg[5] ? arg[5][3] : 0;
  a61=(a61-a109);
  a109=casadi_sq(a61);
  a108=(a108+a109);
  a109=arg[5] ? arg[5][4] : 0;
  a74=(a74-a109);
  a109=casadi_sq(a74);
  a108=(a108+a109);
  a109=arg[5] ? arg[5][5] : 0;
  a85=(a85-a109);
  a109=casadi_sq(a85);
  a108=(a108+a109);
  a109=arg[6] ? arg[6][0] : 0;
  a86=(a86-a109);
  a109=casadi_sq(a86);
  a110=arg[6] ? arg[6][1] : 0;
  a92=(a92-a110);
  a110=casadi_sq(a92);
  a109=(a109+a110);
  a110=arg[6] ? arg[6][2] : 0;
  a96=(a96-a110);
  a110=casadi_sq(a96);
  a109=(a109+a110);
  a110=arg[6] ? arg[6][3] : 0;
  a100=(a100-a110);
  a110=casadi_sq(a100);
  a109=(a109+a110);
  a110=arg[6] ? arg[6][4] : 0;
  a104=(a104-a110);
  a110=casadi_sq(a104);
  a109=(a109+a110);
  a110=arg[6] ? arg[6][5] : 0;
  a107=(a107-a110);
  a110=casadi_sq(a107);
  a109=(a109+a110);
  a108=(a108+a109);
  a109=5.0000000000000000e-01;
  a110=arg[1] ? arg[1][0] : 0;
  a110=(a22-a110);
  a111=casadi_sq(a110);
  a112=(a51-a22);
  a113=casadi_sq(a112);
  a111=(a111+a113);
  a113=(a19-a51);
  a114=casadi_sq(a113);
  a111=(a111+a114);
  a111=(a109*a111);
  a108=(a108+a111);
  a111=1.0000000000000000e-02;
  a114=arg[1] ? arg[1][1] : 0;
  a114=(a7-a114);
  a115=casadi_sq(a114);
  a116=(a39-a7);
  a117=casadi_sq(a116);
  a115=(a115+a117);
  a117=(a66-a39);
  a118=casadi_sq(a117);
  a115=(a115+a118);
  a115=(a111*a115);
  a108=(a108+a115);
  if (res[2]!=0) res[2][0]=a108;
  a108=arg[8] ? arg[8][0] : 0;
  a115=arg[3] ? arg[3][0] : 0;
  a115=(a108*a115);
  a110=(a110+a110);
  a110=(a109*a110);
  a112=(a112+a112);
  a112=(a109*a112);
  a110=(a110-a112);
  a118=casadi_sign(a22);
  a119=(a24-a5);
  a37=pow(a37,a119);
  a37=(a24*a37);
  a107=(a107+a107);
  a104=(a104+a104);
  a104=(a107+a104);
  a100=(a100+a100);
  a100=(a104+a100);
  a96=(a96+a96);
  a96=(a100+a96);
  a119=(a1*a96);
  a95=(a95*a119);
  a93=(a93*a95);
  a93=(a2*a93);
  a58=cos(a58);
  a100=(a1*a100);
  a98=(a98*a100);
  a58=(a58*a98);
  a71=cos(a71);
  a104=(a1*a104);
  a102=(a102*a104);
  a71=(a71*a102);
  a82=cos(a82);
  a107=(a1*a107);
  a106=(a106*a107);
  a82=(a82*a106);
  a83=sin(a83);
  a85=(a85+a85);
  a106=(a1*a85);
  a80=(a80*a106);
  a83=(a83*a80);
  a80=(a82-a83);
  a102=(a71+a80);
  a72=sin(a72);
  a74=(a74+a74);
  a85=(a85+a74);
  a74=(a1*a85);
  a69=(a69*a74);
  a72=(a72*a69);
  a102=(a102-a72);
  a69=(a58+a102);
  a59=sin(a59);
  a61=(a61+a61);
  a85=(a85+a61);
  a61=(a1*a85);
  a56=(a56*a61);
  a59=(a59*a56);
  a69=(a69-a59);
  a56=(a1*a69);
  a98=(a40*a56);
  a98=(a29*a98);
  a93=(a93+a98);
  a99=(a99*a100);
  a97=(a97*a99);
  a97=(a2*a97);
  a102=(a1*a102);
  a100=(a54*a102);
  a100=(a29*a100);
  a97=(a97+a100);
  a103=(a103*a104);
  a101=(a101*a103);
  a101=(a2*a101);
  a80=(a1*a80);
  a104=(a67*a80);
  a104=(a29*a104);
  a101=(a101+a104);
  a14=(a14*a107);
  a105=(a105*a14);
  a105=(a2*a105);
  a84=(a84*a106);
  a79=(a79*a84);
  a79=(a2*a79);
  a105=(a105+a79);
  a101=(a101+a105);
  a105=(a1*a105);
  a79=(a16*a105);
  a101=(a101+a79);
  a73=(a73*a74);
  a68=(a68*a73);
  a68=(a2*a68);
  a101=(a101+a68);
  a97=(a97+a101);
  a101=(a1*a101);
  a68=(a16*a101);
  a97=(a97+a68);
  a60=(a60*a61);
  a55=(a55*a60);
  a55=(a2*a55);
  a97=(a97+a55);
  a93=(a93+a97);
  a97=(a1*a97);
  a55=(a16*a97);
  a93=(a93+a55);
  a47=(a47+a47);
  a85=(a85+a47);
  a47=(a1*a85);
  a46=(a46*a47);
  a41=(a41*a46);
  a41=(a2*a41);
  a93=(a93+a41);
  a41=(a1*a93);
  a36=(a36*a41);
  a37=(a37*a36);
  a118=(a118*a37);
  a110=(a110+a118);
  a118=casadi_sign(a22);
  a37=(a24-a5);
  a23=pow(a23,a37);
  a23=(a24*a23);
  a92=(a92+a92);
  a96=(a96+a92);
  a92=(a1*a96);
  a91=(a91*a92);
  a89=(a89*a91);
  a89=(a2*a89);
  a44=cos(a44);
  a94=(a94*a119);
  a44=(a44*a94);
  a69=(a44+a69);
  a45=sin(a45);
  a42=(a42*a47);
  a45=(a45*a42);
  a69=(a69-a45);
  a42=(a1*a69);
  a47=(a26*a42);
  a29=(a29*a47);
  a89=(a89+a29);
  a89=(a89+a93);
  a16=(a16*a41);
  a89=(a89+a16);
  a34=(a34+a34);
  a85=(a85+a34);
  a34=(a1*a85);
  a33=(a33*a34);
  a27=(a27*a33);
  a2=(a2*a27);
  a89=(a89+a2);
  a89=(a1*a89);
  a21=(a21*a89);
  a23=(a23*a21);
  a118=(a118*a23);
  a110=(a110+a118);
  a115=(a115-a110);
  if (res[3]!=0) res[3][0]=a115;
  a110=arg[3] ? arg[3][1] : 0;
  a110=(a108*a110);
  a113=(a113+a113);
  a109=(a109*a113);
  a112=(a112-a109);
  a113=casadi_sign(a51);
  a118=(a24-a5);
  a64=pow(a64,a118);
  a64=(a24*a64);
  a63=(a63*a101);
  a64=(a64*a63);
  a113=(a113*a64);
  a112=(a112+a113);
  a113=casadi_sign(a51);
  a64=(a24-a5);
  a52=pow(a52,a64);
  a52=(a24*a52);
  a50=(a50*a97);
  a52=(a52*a50);
  a113=(a113*a52);
  a112=(a112+a113);
  a110=(a110-a112);
  if (res[3]!=0) res[3][1]=a110;
  a112=arg[3] ? arg[3][2] : 0;
  a112=(a108*a112);
  a113=casadi_sign(a19);
  a5=(a24-a5);
  a77=pow(a77,a5);
  a24=(a24*a77);
  a18=(a18*a105);
  a24=(a24*a18);
  a113=(a113*a24);
  a109=(a109+a113);
  a112=(a112-a109);
  if (res[3]!=0) res[3][2]=a112;
  a109=arg[3] ? arg[3][3] : 0;
  a109=(a108*a109);
  a114=(a114+a114);
  a114=(a111*a114);
  a116=(a116+a116);
  a116=(a111*a116);
  a114=(a114-a116);
  a31=cos(a31);
  a90=(a90*a92);
  a31=(a31*a90);
  a90=(a12*a31);
  a92=(a26+a26);
  a17=(a17*a91);
  a17=(a6*a17);
  a92=(a92*a17);
  a90=(a90+a92);
  a43=(a43*a42);
  a90=(a90+a43);
  a32=sin(a32);
  a28=(a28*a34);
  a32=(a32*a28);
  a28=(a12*a32);
  a90=(a90-a28);
  a26=(a26+a26);
  a25=(a25*a33);
  a25=(a6*a25);
  a26=(a26*a25);
  a90=(a90+a26);
  a114=(a114+a90);
  a11=cos(a11);
  a86=(a86+a86);
  a96=(a96+a86);
  a96=(a1*a96);
  a87=(a87*a96);
  a11=(a11*a87);
  a11=(a12*a11);
  a87=(a9+a9);
  a88=(a88*a96);
  a3=(a3*a88);
  a3=(a6*a3);
  a87=(a87*a3);
  a11=(a11+a87);
  a31=(a31+a69);
  a31=(a31-a32);
  a31=(a1*a31);
  a30=(a30*a31);
  a11=(a11+a30);
  a13=sin(a13);
  a0=(a0+a0);
  a85=(a85+a0);
  a1=(a1*a85);
  a10=(a10*a1);
  a13=(a13*a10);
  a13=(a12*a13);
  a11=(a11-a13);
  a9=(a9+a9);
  a15=(a15*a1);
  a4=(a4*a15);
  a4=(a6*a4);
  a9=(a9*a4);
  a11=(a11+a9);
  a114=(a114+a11);
  a109=(a109-a114);
  if (res[3]!=0) res[3][3]=a109;
  a114=arg[3] ? arg[3][4] : 0;
  a114=(a108*a114);
  a117=(a117+a117);
  a111=(a111*a117);
  a116=(a116-a111);
  a58=(a12*a58);
  a117=(a54+a54);
  a48=(a48*a99);
  a48=(a6*a48);
  a117=(a117*a48);
  a58=(a58+a117);
  a70=(a70*a102);
  a58=(a58+a70);
  a59=(a12*a59);
  a58=(a58-a59);
  a54=(a54+a54);
  a53=(a53*a60);
  a53=(a6*a53);
  a54=(a54*a53);
  a58=(a58+a54);
  a116=(a116+a58);
  a44=(a12*a44);
  a58=(a40+a40);
  a35=(a35*a95);
  a35=(a6*a35);
  a58=(a58*a35);
  a44=(a44+a58);
  a57=(a57*a56);
  a44=(a44+a57);
  a45=(a12*a45);
  a44=(a44-a45);
  a40=(a40+a40);
  a38=(a38*a46);
  a38=(a6*a38);
  a40=(a40*a38);
  a44=(a44+a40);
  a116=(a116+a44);
  a114=(a114-a116);
  if (res[3]!=0) res[3][4]=a114;
  a116=arg[3] ? arg[3][5] : 0;
  a116=(a108*a116);
  a82=(a12*a82);
  a44=(a8+a8);
  a75=(a75*a14);
  a75=(a6*a75);
  a44=(a44*a75);
  a82=(a82+a44);
  a83=(a12*a83);
  a82=(a82-a83);
  a8=(a8+a8);
  a78=(a78*a84);
  a78=(a6*a78);
  a8=(a8*a78);
  a82=(a82+a8);
  a111=(a111+a82);
  a71=(a12*a71);
  a82=(a67+a67);
  a62=(a62*a103);
  a62=(a6*a62);
  a82=(a82*a62);
  a71=(a71+a82);
  a81=(a81*a80);
  a71=(a71+a81);
  a12=(a12*a72);
  a71=(a71-a12);
  a67=(a67+a67);
  a65=(a65*a73);
  a6=(a6*a65);
  a67=(a67*a6);
  a71=(a71+a67);
  a111=(a111+a71);
  a116=(a116-a111);
  if (res[3]!=0) res[3][5]=a116;
  a111=arg[3] ? arg[3][6] : 0;
  a111=(a108*a111);
  if (res[3]!=0) res[3][6]=a111;
  a71=arg[3] ? arg[3][7] : 0;
  a71=(a108*a71);
  if (res[3]!=0) res[3][7]=a71;
  a67=arg[3] ? arg[3][8] : 0;
  a108=(a108*a67);
  if (res[3]!=0) res[3][8]=a108;
  a67=arg[7] ? arg[7][0] : 0;
  a115=(a67*a115);
  a22=(a22+a115);
  if (res[4]!=0) res[4][0]=a22;
  a110=(a67*a110);
  a51=(a51+a110);
  if (res[4]!=0) res[4][1]=a51;
  a112=(a67*a112);
  a19=(a19+a112);
  if (res[4]!=0) res[4][2]=a19;
  a109=(a67*a109);
  a7=(a7+a109);
  if (res[4]!=0) res[4][3]=a7;
  a114=(a67*a114);
  a39=(a39+a114);
  if (res[4]!=0) res[4][4]=a39;
  a116=(a67*a116);
  a66=(a66+a116);
  if (res[4]!=0) res[4][5]=a66;
  a111=(a67*a111);
  a20=(a20+a111);
  if (res[4]!=0) res[4][6]=a20;
  a71=(a67*a71);
  a49=(a49+a71);
  if (res[4]!=0) res[4][7]=a49;
  a67=(a67*a108);
  a76=(a76+a67);
  if (res[4]!=0) res[4][8]=a76;
  return 0;
}

#pragma GCC diagnostic pop

CASADI_SYMBOL_EXPORT int casadi_mpc_fn(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void casadi_mpc_fn_incref(void) {
}

CASADI_SYMBOL_EXPORT void casadi_mpc_fn_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int casadi_mpc_fn_n_in(void) { return 9;}

CASADI_SYMBOL_EXPORT casadi_int casadi_mpc_fn_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT const char* casadi_mpc_fn_name_in(casadi_int i){
  switch (i) {
    case 0: return "var_x0";
    case 1: return "var_u0";
    case 2: return "var_u";
    case 3: return "var_momentum";
    case 4: return "var_params";
    case 5: return "var_reference_trajectory_x";
    case 6: return "var_reference_trajectory_y";
    case 7: return "var_learning_rate";
    case 8: return "var_momentum_rate";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* casadi_mpc_fn_name_out(casadi_int i){
  switch (i) {
    case 0: return "trajectory_x";
    case 1: return "trajectory_y";
    case 2: return "objective";
    case 3: return "var_momentum_next";
    case 4: return "var_u_next";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* casadi_mpc_fn_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    case 6: return casadi_s4;
    case 7: return casadi_s5;
    case 8: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* casadi_mpc_fn_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int casadi_mpc_fn_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 9;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
