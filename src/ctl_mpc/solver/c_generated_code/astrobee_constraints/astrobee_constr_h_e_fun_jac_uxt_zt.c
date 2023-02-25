/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) astrobee_constr_h_e_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_mtimes CASADI_PREFIX(mtimes)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
#define casadi_s14 CASADI_PREFIX(s14)
#define casadi_s15 CASADI_PREFIX(s15)
#define casadi_s16 CASADI_PREFIX(s16)
#define casadi_s17 CASADI_PREFIX(s17)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)

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

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_mtimes(const casadi_real* x, const casadi_int* sp_x, const casadi_real* y, const casadi_int* sp_y, casadi_real* z, const casadi_int* sp_z, casadi_real* w, casadi_int tr) {
  casadi_int ncol_x, ncol_y, ncol_z, cc;
  const casadi_int *colind_x, *row_x, *colind_y, *row_y, *colind_z, *row_z;
  ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y + 2 + ncol_y+1;
  ncol_z = sp_z[1];
  colind_z = sp_z+2; row_z = sp_z + 2 + ncol_z+1;
  if (tr) {
    for (cc=0; cc<ncol_z; ++cc) {
      casadi_int kk;
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        w[row_y[kk]] = y[kk];
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_z[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          z[kk] += x[kk1] * w[row_x[kk1]];
        }
      }
    }
  } else {
    for (cc=0; cc<ncol_y; ++cc) {
      casadi_int kk;
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        w[row_z[kk]] = z[kk];
      }
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_y[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          w[row_x[kk1]] += x[kk1]*y[kk];
        }
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        z[kk] = w[row_z[kk]];
      }
    }
  }
}

static const casadi_int casadi_s0[132] = {128, 1, 0, 128, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127};
static const casadi_int casadi_s1[5] = {12, 1, 0, 1, 0};
static const casadi_int casadi_s2[1551] = {128, 12, 0, 128, 256, 384, 512, 640, 768, 896, 1024, 1152, 1280, 1408, 1536, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127};
static const casadi_int casadi_s3[5] = {12, 1, 0, 1, 1};
static const casadi_int casadi_s4[5] = {12, 1, 0, 1, 2};
static const casadi_int casadi_s5[5] = {12, 1, 0, 1, 3};
static const casadi_int casadi_s6[5] = {12, 1, 0, 1, 4};
static const casadi_int casadi_s7[5] = {12, 1, 0, 1, 5};
static const casadi_int casadi_s8[5] = {12, 1, 0, 1, 6};
static const casadi_int casadi_s9[5] = {12, 1, 0, 1, 7};
static const casadi_int casadi_s10[5] = {12, 1, 0, 1, 8};
static const casadi_int casadi_s11[5] = {12, 1, 0, 1, 9};
static const casadi_int casadi_s12[5] = {12, 1, 0, 1, 10};
static const casadi_int casadi_s13[5] = {12, 1, 0, 1, 11};
static const casadi_int casadi_s14[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s15[3] = {0, 0, 0};
static const casadi_int casadi_s16[1667] = {12, 128, 0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204, 216, 228, 240, 252, 264, 276, 288, 300, 312, 324, 336, 348, 360, 372, 384, 396, 408, 420, 432, 444, 456, 468, 480, 492, 504, 516, 528, 540, 552, 564, 576, 588, 600, 612, 624, 636, 648, 660, 672, 684, 696, 708, 720, 732, 744, 756, 768, 780, 792, 804, 816, 828, 840, 852, 864, 876, 888, 900, 912, 924, 936, 948, 960, 972, 984, 996, 1008, 1020, 1032, 1044, 1056, 1068, 1080, 1092, 1104, 1116, 1128, 1140, 1152, 1164, 1176, 1188, 1200, 1212, 1224, 1236, 1248, 1260, 1272, 1284, 1296, 1308, 1320, 1332, 1344, 1356, 1368, 1380, 1392, 1404, 1416, 1428, 1440, 1452, 1464, 1476, 1488, 1500, 1512, 1524, 1536, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s17[3] = {128, 0, 0};

static const casadi_real casadi_c0[1536] = {1., 0., 0., -1., 0., 0., -2.1262119797839307e-01, 2.1262119797839307e-01, 0., 0., 0., 0., -2.0329448910436734e-01, 2.0329448910436734e-01, 0., 0., 0., 0., -1.9381170733169620e-01, 1.9381170733169620e-01, 0., 0., 0., 0., -1.8413316750951103e-01, 1.8413316750951103e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0., 9.8219986825223626e-01, 0., -9.8219986825223626e-01, 0., 0., 0., 9.3884904236993239e-01, 0., -9.3884904236993239e-01, 0., 0., 0., 8.8393110476168069e-01, 0., -8.8393110476168069e-01, 0., 0., 0., 8.2690592381590600e-01, 0., -8.2690592381590600e-01, 0., 7.7269010877274169e-01, 0., -7.7269010877274169e-01, 0., 7.2325575793686403e-01, 0., -7.2325575793686403e-01, 0., 6.7902894160845695e-01, 0., -6.7902894160845695e-01, 0., 6.3973822763407340e-01, 0., -6.3973822763407340e-01, 0., 6.0485352993817232e-01, 0., -6.0485352993817232e-01, 0., 5.7379149131131246e-01, 0., -5.7379149131131246e-01, 0., 5.4600232994164688e-01, 0., -5.4600232994164688e-01, 0., 5.2100041636635508e-01, 0., -5.2100041636635508e-01, 0., 4.9836969010444571e-01, 0., -4.9836969010444571e-01, 0., 4.7775877375526293e-01, 0., -4.7775877375526293e-01, 0., 4.5887262438996207e-01, 0., -4.5887262438996207e-01, 0., 4.4146372143651752e-01, 0., -4.4146372143651752e-01, 0., 4.2532399135949900e-01, 0., -4.2532399135949900e-01, 0., 4.1027785011622131e-01, 0., -4.1027785011622131e-01, 0., 3.9617638900653296e-01, 0., -3.9617638900653296e-01, 0., 3.8289259046383711e-01, 0., -3.8289259046383711e-01, 0., 3.7031741935674251e-01, 0., -3.7031741935674251e-01, 0., 3.5835663651640542e-01, 0., -3.5835663651640542e-01, 0., 3.4692819837525263e-01, 0., -3.4692819837525263e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0., 9.8219986825223626e-01, 0., -9.8219986825223626e-01, 0., 0., 0., 9.3884904236993239e-01, 0., -9.3884904236993239e-01, 0., 0., 0., 8.8393110476168069e-01, 0., -8.8393110476168069e-01, 0., 0., 0., 8.2690592381590600e-01, 0., -8.2690592381590600e-01, 0., 7.7269010877274169e-01, 0., -7.7269010877274169e-01, 0., 7.2325575793686403e-01, 0., -7.2325575793686403e-01, 0., 6.7902894160845695e-01, 0., -6.7902894160845695e-01, 0., 6.3973822763407340e-01, 0., -6.3973822763407340e-01, 0., 6.0485352993817232e-01, 0., -6.0485352993817232e-01, 0., 5.7379149131131246e-01, 0., -5.7379149131131246e-01, 0., 5.4600232994164688e-01, 0., -5.4600232994164688e-01, 0., 5.2100041636635508e-01, 0., -5.2100041636635508e-01, 0., 4.9836969010444571e-01, 0., -4.9836969010444571e-01, 0., 4.7775877375526293e-01, 0., -4.7775877375526293e-01, 0., 4.5887262438996207e-01, 0., -4.5887262438996207e-01, 0., 4.4146372143651752e-01, 0., -4.4146372143651752e-01, 0., 4.2532399135949900e-01, 0., -4.2532399135949900e-01, 0., 4.1027785011622131e-01, 0., -4.1027785011622131e-01, 0., 3.9617638900653296e-01, 0., -3.9617638900653296e-01, 0., 3.8289259046383711e-01, 0., -3.8289259046383711e-01, 0., 3.7031741935674251e-01, 0., -3.7031741935674251e-01, 0., 3.5835663651640542e-01, 0., -3.5835663651640542e-01, 0., 3.4692819837525263e-01, 0., -3.4692819837525263e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -9.7713470216251819e-01, 9.7713470216251819e-01, 0., 0., 0., 0., -9.7911763884621861e-01, 9.7911763884621861e-01, 0., 0., 0., 0., -9.8103874648312084e-01, 9.8103874648312084e-01, 0., 0., 0., 0., -9.8290130563699762e-01, 9.8290130563699762e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.8783881070026434e-01, 0., -1.8783881070026434e-01, 0., 0., 0., 3.4432902236242191e-01, 0., -3.4432902236242191e-01, 0., 0., 0., 4.6761715327262610e-01, 0., -4.6761715327262610e-01, 0., 0., 0., 5.6234028235416600e-01, 0., -5.6234028235416600e-01, 0., 6.3478342433051038e-01, 0., -6.3478342433051038e-01, 0., 6.9058026949165896e-01, 0., -6.9058026949165896e-01, 0., 7.3411150137979653e-01, 0., -7.3411150137979653e-01, 0., 7.6859287018786115e-01, 0., -7.6859287018786115e-01, 0., 7.9633674241575259e-01, 0., -7.9633674241575259e-01, 0., 8.1900141910667057e-01, 0., -8.1900141910667057e-01, 0., 8.3778365685795830e-01, 0., -8.3778365685795830e-01, 0., 8.5355642235653328e-01, 0., -8.5355642235653328e-01, 0., 8.6696461980013850e-01, 0., -8.6696461980013850e-01, 0., 8.7849106660219800e-01, 0., -8.7849106660219800e-01, 0., 8.8850206222915940e-01, 0., -8.8850206222915940e-01, 0., 8.9727909964259212e-01, 0., -8.9727909964259212e-01, 0., 9.0504116059659134e-01, 0., -9.0504116059659134e-01, 0., 9.1196057245037265e-01, 0., -9.1196057245037265e-01, 0., 9.1817442177058295e-01, 0., -9.1817442177058295e-01, 0., 9.2379286864961907e-01, 0., -9.2379286864961907e-01, 0., 9.2890527446072924e-01, 0., -9.2890527446072924e-01, 0., 9.3358476908347698e-01, 0., -9.3358476908347698e-01, 0., 9.3789169159988917e-01, 0., -9.3789169159988917e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.8783881070026434e-01, 0., -1.8783881070026434e-01, 0., 0., 0., 3.4432902236242191e-01, 0., -3.4432902236242191e-01, 0., 0., 0., 4.6761715327262610e-01, 0., -4.6761715327262610e-01, 0., 0., 0., 5.6234028235416600e-01, 0., -5.6234028235416600e-01, 0., 6.3478342433051038e-01, 0., -6.3478342433051038e-01, 0., 6.9058026949165896e-01, 0., -6.9058026949165896e-01, 0., 7.3411150137979653e-01, 0., -7.3411150137979653e-01, 0., 7.6859287018786115e-01, 0., -7.6859287018786115e-01, 0., 7.9633674241575259e-01, 0., -7.9633674241575259e-01, 0., 8.1900141910667057e-01, 0., -8.1900141910667057e-01, 0., 8.3778365685795830e-01, 0., -8.3778365685795830e-01, 0., 8.5355642235653328e-01, 0., -8.5355642235653328e-01, 0., 8.6696461980013850e-01, 0., -8.6696461980013850e-01, 0., 8.7849106660219800e-01, 0., -8.7849106660219800e-01, 0., 8.8850206222915940e-01, 0., -8.8850206222915940e-01, 0., 8.9727909964259212e-01, 0., -8.9727909964259212e-01, 0., 9.0504116059659134e-01, 0., -9.0504116059659134e-01, 0., 9.1196057245037265e-01, 0., -9.1196057245037265e-01, 0., 9.1817442177058295e-01, 0., -9.1817442177058295e-01, 0., 9.2379286864961907e-01, 0., -9.2379286864961907e-01, 0., 9.2890527446072924e-01, 0., -9.2890527446072924e-01, 0., 9.3358476908347698e-01, 0., -9.3358476908347698e-01, 0., 9.3789169159988917e-01, 0., -9.3789169159988917e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -6.1770369490697963e-01, 0., 0., 6.1770369490697963e-01, 0., 0., -9.6320723558443688e-01, 0., 0., 9.6320723558443688e-01, 0., 0., -9.9815695904209201e-01, 0., 9.9815695904209201e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -6.1998447093438169e-01, 0., 0., 6.1998447093438169e-01, 0., 0., -9.7271988711069124e-01, 0., 0., 9.7271988711069124e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -6.1579017547350190e-01, 0., 0., 6.1579017547350190e-01, 0., 0., -9.5423002048368200e-01, 0., 0., 9.5423002048368200e-01, 0., -9.9869447781069298e-01, 0., 9.9869447781069298e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., -7.8641092648707844e-01, 0., 0., 7.8641092648707844e-01, 0., 0., 2.6875978366896192e-01, 0., 0., -2.6875978366896192e-01, 0., 0., -6.0685130928781722e-02, 0., 6.0685130928781722e-02, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., -7.8461408080674588e-01, 0., 0., 7.8461408080674588e-01, 0., 0., 2.3198280371476729e-01, 0., 0., -2.3198280371476729e-01, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., -7.8791018510380595e-01, 0., 0., 7.8791018510380595e-01, 0., 0., 2.9907368324162531e-01, 0., 0., -2.9907368324162531e-01, 0., -5.1081699173257493e-02, 0., 5.1081699173257493e-02};

/* astrobee_constr_h_e_fun_jac_uxt_zt:(i0[12],i1[],i2[],i3[12])->(o0[128],o1[12x128],o2[128x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+128, *w1=w+256, *w2=w+1792, *w3=w+1804, *w4=w+1816, w5;
  /* #0: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #1: @1 = 
  [[1, 0, 0, ..., 0, 0, 0], 
   [0, 1, 0, ..., 0, 0, 0], 
   [0, 0, 1, ..., 0, 0, 0], 
   ...,
   [0, 0, 0, ..., 0, 0, -0.0510817], 
   [0, 0, 0, ..., 0.0606851, 0, 0], 
   [0, 0, 0, ..., 0, 0, 0.0510817]] */
  casadi_copy(casadi_c0, 1536, w1);
  /* #2: @2 = input[0][0] */
  casadi_copy(arg[0], 12, w2);
  /* #3: @3 = input[3][0] */
  casadi_copy(arg[3], 12, w3);
  /* #4: @2 = (@2-@3) */
  for (i=0, rr=w2, cs=w3; i<12; ++i) (*rr++) -= (*cs++);
  /* #5: @0 = mac(@1,@2,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<128; ++j, ++rr) for (k=0, ss=w1+j, tt=w2+i*12; k<12; ++k) *rr += ss[k*128]**tt++;
  /* #6: output[0][0] = @0 */
  casadi_copy(w0, 128, res[0]);
  /* #7: @4 = zeros(12x128) */
  casadi_clear(w4, 1536);
  /* #8: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #9: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #10: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s1, w0, casadi_s0, w, 0);
  /* #11: (@4[:1536:12] = @0) */
  for (rr=w4+0, ss=w0; rr!=w4+1536; rr+=12) *rr = *ss++;
  /* #12: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #13: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #14: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s3, w0, casadi_s0, w, 0);
  /* #15: (@4[1:1537:12] = @0) */
  for (rr=w4+1, ss=w0; rr!=w4+1537; rr+=12) *rr = *ss++;
  /* #16: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #17: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #18: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s4, w0, casadi_s0, w, 0);
  /* #19: (@4[2:1538:12] = @0) */
  for (rr=w4+2, ss=w0; rr!=w4+1538; rr+=12) *rr = *ss++;
  /* #20: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #21: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #22: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s5, w0, casadi_s0, w, 0);
  /* #23: (@4[3:1539:12] = @0) */
  for (rr=w4+3, ss=w0; rr!=w4+1539; rr+=12) *rr = *ss++;
  /* #24: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #25: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #26: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s6, w0, casadi_s0, w, 0);
  /* #27: (@4[4:1540:12] = @0) */
  for (rr=w4+4, ss=w0; rr!=w4+1540; rr+=12) *rr = *ss++;
  /* #28: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #29: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #30: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s7, w0, casadi_s0, w, 0);
  /* #31: (@4[5:1541:12] = @0) */
  for (rr=w4+5, ss=w0; rr!=w4+1541; rr+=12) *rr = *ss++;
  /* #32: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #33: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #34: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s8, w0, casadi_s0, w, 0);
  /* #35: (@4[6:1542:12] = @0) */
  for (rr=w4+6, ss=w0; rr!=w4+1542; rr+=12) *rr = *ss++;
  /* #36: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #37: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #38: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s9, w0, casadi_s0, w, 0);
  /* #39: (@4[7:1543:12] = @0) */
  for (rr=w4+7, ss=w0; rr!=w4+1543; rr+=12) *rr = *ss++;
  /* #40: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #41: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #42: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s10, w0, casadi_s0, w, 0);
  /* #43: (@4[8:1544:12] = @0) */
  for (rr=w4+8, ss=w0; rr!=w4+1544; rr+=12) *rr = *ss++;
  /* #44: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #45: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #46: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s11, w0, casadi_s0, w, 0);
  /* #47: (@4[9:1545:12] = @0) */
  for (rr=w4+9, ss=w0; rr!=w4+1545; rr+=12) *rr = *ss++;
  /* #48: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #49: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #50: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s12, w0, casadi_s0, w, 0);
  /* #51: (@4[10:1546:12] = @0) */
  for (rr=w4+10, ss=w0; rr!=w4+1546; rr+=12) *rr = *ss++;
  /* #52: @0 = zeros(128x1) */
  casadi_clear(w0, 128);
  /* #53: @5 = ones(12x1,1nz) */
  w5 = 1.;
  /* #54: @0 = mac(@1,@5,@0) */
  casadi_mtimes(w1, casadi_s2, (&w5), casadi_s13, w0, casadi_s0, w, 0);
  /* #55: (@4[11:1547:12] = @0) */
  for (rr=w4+11, ss=w0; rr!=w4+1547; rr+=12) *rr = *ss++;
  /* #56: output[1][0] = @4 */
  casadi_copy(w4, 1536, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int astrobee_constr_h_e_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int astrobee_constr_h_e_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int astrobee_constr_h_e_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void astrobee_constr_h_e_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int astrobee_constr_h_e_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void astrobee_constr_h_e_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void astrobee_constr_h_e_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void astrobee_constr_h_e_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int astrobee_constr_h_e_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int astrobee_constr_h_e_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real astrobee_constr_h_e_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* astrobee_constr_h_e_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* astrobee_constr_h_e_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* astrobee_constr_h_e_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s14;
    case 1: return casadi_s15;
    case 2: return casadi_s15;
    case 3: return casadi_s14;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* astrobee_constr_h_e_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s16;
    case 2: return casadi_s17;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int astrobee_constr_h_e_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 3353;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
