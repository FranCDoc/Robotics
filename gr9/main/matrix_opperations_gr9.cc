#include "matrix_opperations_gr9.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr9); // Code from Antoine Genard

// MATRIX OBJECT CREATION

// initialise matrix
Matrix* init_mat(int row, int col){
#ifdef DEBUG
    if(row>SIZE_M || col >SIZE_M){
        fprintf(stderr, "matrix initialisation error, bigger than 3x3");
        exit(EXIT_FAILURE);
    }
#endif
    
    Matrix *mat = (Matrix*) malloc(sizeof(Matrix));
    double **m = (double**) malloc(SIZE_M*sizeof(double*));
    for(int i=0; i<SIZE_M; i++){
        m[i] = (double*) malloc(SIZE_M*sizeof(double));
    }
    mat->m = m;
    mat->n_row = row;
    mat->n_col = col;
    return mat;
}

// free matrix
void free_mat(Matrix *mat){
    for(int i=0; i<SIZE_M; i++){
        free(mat->m[i]);
    }
    free(mat->m);
    free(mat);
}

// print matrix
void print_Matrix(Matrix *mat){
    for(int i=0;i<mat->n_row;i++){
        for(int j=0;j<mat->n_col;j++){
            printf("  %lf  ",mat->m[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

// MATRIX INITIALIZATION

// matrix containing zeros
void zeros(Matrix *mat){
    for(int i=0;i<mat->n_row;i++)
        for(int j=0;j<mat->n_col;j++)
            mat->m[i][j] = 0.0;
}

// eye matrix
void eye(Matrix *mat){
    for(int i=0;i<mat->n_row;i++)
        for(int j=0;j<mat->n_col;j++)
            if (i==j){mat->m[i][j] = 1.0;}
            else mat->m[i][j] = 0.0;
}

// fill in a specific matrix cell
void fill_matrix(Matrix *mat, int nbr_row, int nbr_column, double x){
#ifdef DEBUG
    if (nbr_row>mat->n_row || nbr_column>mat->n_col){
        fprintf(stderr, "matrix filling error, invalid index");
        exit(EXIT_FAILURE);
    }
#endif
    mat-> m[nbr_row-1][nbr_column-1] = x;
}

//MATRIX OPPERATIONS

// matrix product
void mat_product(Matrix *product, Matrix *A, Matrix *B){
#ifdef DEBUG
    if(A->n_col != B->n_row){
        fprintf(stderr, "matrix multiplication problem: sizes do not correspond\n");
        printf("row1 =  %d\n", A->n_row);
        printf("row2 =  %d\n", B->n_row);
        printf("col1 =  %d\n", A->n_col);
        printf("col2 =  %d\n", B->n_col);
        exit(EXIT_FAILURE);
    }
#endif
    product->n_row = A->n_row;
    product->n_col = B->n_col;
    
    Matrix *Abis = copy(A);
    Matrix *Bbis = copy(B);
    
    for(int i=0;i<product->n_row;i++){
        for(int j=0;j<product->n_col;j++){
            product->m[i][j] = 0.0;
            for(int k=0;k<A->n_col;k++)
                product->m[i][j] += Abis->m[i][k] * Bbis->m[k][j];
        }
    }
    
    free_mat(Abis);
    free_mat(Bbis);
}

//transpose of a matrix
void transpose(Matrix *trans, Matrix *mat){
    Matrix *mat2 = copy(mat);
    
    trans->n_row = mat->n_col;
    trans->n_col = mat->n_row;
    
    for(int i=0;i<trans->n_row;i++)
        for(int j=0;j<trans->n_col;j++)
            trans->m[i][j] = mat2->m[j][i];
    
    free_mat(mat2);
}

//sum of matrix
void mat_sum(Matrix *sum, Matrix *A, Matrix *B){
#ifdef DEBUG
    if(B->n_row != A->n_row || A->n_col != B->n_col){
        fprintf(stderr, "Matrix addition problem: sizes do not correspond\n");
        printf("row1 =  %d\n", A->n_row);
        printf("row2 =  %d\n", B->n_row);
        printf("col1 =  %d\n", A->n_col);
        printf("col2 =  %d\n", B->n_col);
        exit(EXIT_FAILURE);
    }
#endif
    Matrix *Abis = copy(A);
    Matrix *Bbis = copy(B);
    
    sum->n_col = A->n_col;
    sum->n_row = A->n_row;
    for(int i=0;i<sum->n_row;i++){
        for(int j=0;j<sum->n_col;j++){
            sum->m[i][j] = Abis->m[i][j] + Bbis->m[i][j];
        }
    }
    free_mat(Abis);
    free_mat(Bbis);
}

// mustiplication by constant
void mat_const_product(Matrix *mat, double cst){
    for(int i=0;i<mat->n_row;i++)
        for(int j=0;j<mat->n_col;j++)
            mat->m[i][j] = cst * mat->m[i][j];
}

//OTHER OPPERATIONS
Matrix* copy(Matrix *mat){
    Matrix *mat_copy = init_mat(mat->n_row, mat->n_col);
    for(int i=0;i<mat->n_row;i++)
        for(int j=0;j<mat->n_col;j++)
            mat_copy->m[i][j] = mat->m[i][j];
    
    return mat_copy;
}

// Cholesky decomposition (lower matrix decomposition)
void cholesky(Matrix *result, Matrix *mat){
    zeros(result);
    int n = mat->n_row;
    double **m = mat->m;
    
#ifdef DEBUG
    if(mat->n_row != mat->n_col){
        fprintf(stderr, "Cholesky problem: not square matrix\n");
        printf("row =  %d\n", mat->n_row);
        printf("col =  %d\n", mat->n_col);
        exit(EXIT_FAILURE);
    }
#endif
    
    for (int j=0; j<n; ++j){
        for (int i=j; i<n; ++i){
            double sum = 0.0;
            double sum2 = 0.0;
            if (i==0 && j==0){
                result->m[i][j] = sqrt(m[0][0]);
            }
            else if (j==0){
                result->m[i][j] = m[i][j]/result->m[0][0];
            }
            else{
                if (i==j){
                    for(int k=0; k<j; ++k){
                        sum += pow(result->m[i][k],2.0);
                    }
                    result->m[i][j] = sqrt(m[i][j]-sum);
                }
                else {
                    for (int k=0; k<j; ++k){
                        sum2 += result->m[i][k]*result->m[j][k];
                    }
                    result->m[i][j] = (m[i][j] - sum2)/(result->m[j][j]);
                }
            }
        }
    }
}

// inverse symetric positive-definite matrix  (with cholesky)
void inverse(Matrix *result, Matrix *mat){
    Matrix *chol = init_mat(3,3);
    Matrix *result_pre = init_mat(3,3);
    zeros(result_pre);
    cholesky(chol, mat);
    int n = chol->n_row;
    double **m = chol->m;
    for (int i=0; i<n; ++i){
        for (int j=0; j<i+1; ++j){
            double sum = 0.0;
            if (i==j){
                result_pre->m[i][i] = 1/m[i][i];
            }
            else {
                for (int k = j; k<i+1; ++k){
                    sum += m[i][k]*result_pre->m[k][j];
                }
                result_pre->m[i][j] = -sum/m[i][i];
            }
        }
    }
    Matrix *transposed = init_mat(3,3);
    transpose(transposed, result_pre);
    mat_product(result, transposed,result_pre);
    

}

NAMESPACE_CLOSE();

