#include <iostream>
#include <fstream>
#include <complex>
#include <vector>
using namespace std;

void optionSelect(int option);
void answerSelect();
int userInput();
int IEEEStandardBusSystems(int numberOfBuses, std::ifstream &infile, std::ifstream &lineData, int numLines);
void createYbusSparse(int *fromBusArr, int *toBusArr, int *yrow, int *ycol, complex<double> *y, complex<double> *Z, complex<double> *B, int numLines, int numberOfBuses);
void powerEqn(double *Psparse, double *Qsparse, double *Vm, double *theta, int *yrow, int *ycol, complex<double> *y, int ynnz);
void createJacobianSparse(vector<int>& jacRowIdx1, vector<int>& jacColIdx1, vector<int>& yForJ22, vector<double>& jac1, int slackBus, int ynnz, int N_p, int N_g, int *yrow, int *ycol, complex<double> *y, double *Vm, double *theta, int *PQindex, double *Psparse, double *Qsparse, int jacSize);
void createJ11(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, int &count, int ynnz, int slackBus);
void createJ12_J21(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, int *PQbuses, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int &count, int ynnz, int N_p, int slackBus, int numberOfBuses);
void createJ22(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Qsparse, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int &count, int ynnz);
void createJacobianSparse2(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, int *PQbuses, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int ynnz, int N_p, int slackBus, int numberOfBuses);

double dot(double*, double*, int);
double norm(double*, int);
void biCG(double* x, double* b, int n, double* csrValA, int* csrRowPtrA, int* csrColIndA);
void axpy(double, double*, double*, int);
void spMVcsr(double csrValA[], int csrRowPtrA[], int csrColIndA[], double x[], double y[], int rows);
void dense2csr(double *a, int nnz, int rows, int cols, double csrValA[], int csrRowPtrA[], int csrColIndA[]);
int findNNZ(double* A, int n);
void triangularSolve(double *csrValL, int* csrRowPtrL, int* csrColIndL, double *csrValU, int* csrRowPtrU, int* csrColIndU, double* Udiag, double* x, double *b, int n);
void printMatrix(double*, int);
void printVector(double*, int);
//void pbiCGStab(double* x, double* b, int n, double* csrValA, int* csrRowPtrA, int* csrColIndA, double* csrValL, int* csrRowPtrL, int* csrColIndL, double* csrValU, int* csrRowPtrU, int* csrColIndU, double* Udiag);
void pbiCGStab(double* x, double* b, int n, double* csrValA, int* csrRowPtrA, int* csrColIndA, double* csrValL, int* csrRowPtrL, int* csrColIndL, double* csrValU, int* csrRowPtrU, int* csrColIndU);
void findDiag(double *A, int n, double* diag);
void ILUsparse(int numRows, int numEl, int &countU, int &countL, int *row, int *col, double *val, double* &U, double* &Udiag, int* &Urow, int* &Ucol, double* &L, int* &Lrow, int* &Lcol);
void coo2csr(double *cooVal, int *cooRowIdx, int *cooColIdx,double *csrVal, int *csrColIdx, int *csrRowPtr, int nnz, int numRows);
int QSort(int *v, double* x, int base_ptr, int total_elems);
void csrILU(double* val, int* col, int* rowPtr, double* &lval, int* &lcol, int* lptr, double* &uval, int* &ucol, int* uptr, int numRows, int &lnnz, int &unnz);
void lowerTriangularSolve(int m, double *val, int* indx, int* pntr, double* b, double* c);
void upperTriangularSolve(int m, double *val, int* indx, int*pntr, double* b, double* c);
