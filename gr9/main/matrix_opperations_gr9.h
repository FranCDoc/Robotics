/*!
 * \file matrix_opperations_gr9.h
 * \brief opperations on matrix
 */

#ifndef _MATRIX_OPPERATIONS_GR9_H_
#define _MATRIX_OPPERATIONS_GR9_H_

#include "namespace_ctrl.h"
#include <stdlib.h>
#include <stdio.h>

#define DEBUG
#define SIZE_M 3 // max matrix size

NAMESPACE_INIT(ctrlGr9);

typedef struct Matrix{
    double** m;
    int n_row;
    int n_col;
} Matrix;

/*
 * Allocate a matrix with n row and m column
 free a matrix with n row and m column
 */
Matrix* init_mat(int row, int col);
void free_mat(Matrix *mat);

/*
 * print_Matrix : print the matrix in the console
 */
void print_Matrix(Matrix *mat);

/*
 * zeros : fill in a matrix with 0;
 * eye  : create an eye matrix
 * fill_matrix  : set an element of the matrix
 */
void zeros(Matrix *mat);
void eye(Matrix *mat);
void fill_matrix(Matrix *mat, int row, int column, double x);


/*
 * copy  : copy the matrix in order to realize operation without modifiying it
 */
Matrix *copy(Matrix *mat);


/*
 * mat_const_product : multiply a matrix by a constant
 * mat_product : multiply to matrix : result = A * B
 * transpose : transpose the matrix A
 * mat_sum : sum = sum of *A and *B
 */
void mat_const_product(Matrix *mat, double cst);
void mat_product(Matrix *result, Matrix *A, Matrix *B);
void transpose(Matrix *trans, Matrix *mat);
void mat_sum(Matrix *sum, Matrix *A, Matrix *B);

/*
 * Cholesky : Cholesky decomposition of a positive-definite matrix (*result is a lower triangular matrix)
 * inverse : gives in *result the invere of matrix *mat if it is symetrix and positive-definite
 */
void cholesky(Matrix *result, Matrix *mat);
void inverse(Matrix *result, Matrix *mat);


NAMESPACE_CLOSE();

#endif

