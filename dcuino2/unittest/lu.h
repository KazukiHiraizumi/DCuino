#ifndef LUHEADERH
#define LUHEADERH

//#include <math.h>
//#include <stdint.h>
//#include <stddef.h>

#define MEPS_LU (1e-10)

int LU_decomposition(int dim, int *pivot, float *matrix) {
  int row, col, k;
  int ip, ip_tmp;
  float tmp_val, max_val;

  for (k = 0; k < dim; k++) pivot[k] = k;
  for (k = 0; k < dim - 1; k++) {
    max_val = fabs(matrix[k * dim + k]);
    ip = k;

    for (row = k + 1; row < dim; row++) {
      tmp_val = fabs(matrix[row * dim + k]);

      if (max_val < tmp_val) {
        max_val = tmp_val;
        ip = row;
      }
    }

    if (max_val < (double)MEPS_LU) {
      return -1; //Error
    }
    else if (ip != k) {
      for (col = k; col < dim; col++) {
        tmp_val = matrix[ip * dim + col];
        matrix[ip * dim + col] = matrix[k * dim + col];
        matrix[k * dim + col] = tmp_val;
      }
      ip_tmp = pivot[ip];
      pivot[ip] = pivot[k];
      pivot[k] = ip_tmp;

      for (col = 0; col < k; col++) {
        tmp_val = matrix[k * dim + col];
        matrix[k * dim + col] = matrix[ip * dim + col];
        matrix[ip * dim + col] = tmp_val;
      }
    }

    for (row = k + 1; row < dim; row++) {
      tmp_val = matrix[row * dim + k] / matrix[k * dim + k];
      matrix[row * dim + k] = tmp_val;

      for (col = k + 1; col < dim; col++) {
        matrix[row * dim + col] -= tmp_val * matrix[k * dim + col];
      }
    }
  }
  return 0;
}

int LU_solver(int dim, int *pivot, float *matrix, float *vec, float *y) {
  int row, col;
  float sum;

  for (row = 0; row < dim; row++) {
    sum = vec[pivot[row]];

    for (col = 0; col < row; col++) {
      sum -= matrix[row * dim + col] * y[col];
    }
    y[row] = sum;
  }

  for (row = dim - 1; row >= 0; row--) {
    sum = y[row];
    y[row] = vec[row];

    for (col = row + 1; col < dim; col++) {
//      sum -= matrix[row * dim + col] * vec[col];
      sum -= matrix[row * dim + col] * y[col];
    }
//    vec[row] = sum / matrix[row * dim + row];
    y[row] = sum / matrix[row * dim + row];
  }
  return 0;
}

#endif

