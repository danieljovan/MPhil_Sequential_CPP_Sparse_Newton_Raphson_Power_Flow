#include <string>
#include "cpuFunctions.h"
#include "TimingFunctions.h"
using namespace std;

#define CMP(A,B) ((A) < (B))

// swap two items
//
static inline void SWAP_double(double &A, double &B)
{
    double tmp = A; A = B; B = tmp;
}
 
static inline void swap_int(int &a, int &b)
{
    int tmp=a; a=b; b=tmp;
}

// This should be replaced by a standard ANSI macro.
#define BYTES_PER_WORD 8

/* The next 4 #defines implement a very fast in-line stack abstraction. */
#define STACK_SIZE (BYTES_PER_WORD * sizeof (long))
#define PUSH(LOW,HIGH) do {top->lo = LOW;top++->hi = HIGH;} while (0)
#define POP(LOW,HIGH)  do {LOW = (--top)->lo;HIGH = top->hi;} while (0)
#define STACK_NOT_EMPTY (stack < top) 
#define MAX_THRESH 4

typedef struct {
  int lo;
  int hi;
} stack_node;

#define Tol ((double) 0.0001)
#define max_it 2500
ofstream times("exec time.txt");

void optionSelect(int option) {
	ifstream busdata, linedata;
	int numLines=0;
	string line;
	if ((option<1) || (option>7)) {
		cout<<"Not a valid option"<<endl;
		answerSelect();
	}
	else if (option==1) {
		cout<<"----IEEE 14 bus system----\n"<<endl;
		linedata.open("linedata.txt");
		while (getline (linedata, line)) {
			++numLines;
		}
		linedata.close(); //pointer is at EOF. Need to close and reopen to stream data into variables
		busdata.open("busdata.txt");
		linedata.open("linedata.txt");
		IEEEStandardBusSystems(14, busdata, linedata, numLines); //calls function to execute data intialization for 14 bus system - N.B. read from text file (for now)
		answerSelect();
	}
	else if (option==2){
		cout<<"----IEEE 118 bus system----\n"<<endl;
		linedata.open("118Bus_LineData.txt");
		while (getline (linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("118BusData.txt");
		linedata.open("118Bus_LineData.txt");
		IEEEStandardBusSystems(118, busdata, linedata, numLines);
		answerSelect();
	}
		
	else if (option==3) {
		cout<<"----IEEE 300 bus system----"<<endl;
		linedata.open("300Bus_LineData.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("300BusData.txt");
		linedata.open("300Bus_LineData.txt");
		IEEEStandardBusSystems(300, busdata, linedata, numLines);
		answerSelect();
	}
	else if (option==4) {
		cout<<"----Polish Winter Peak 2383 bus system----"<<endl;
		linedata.open("2383LineData.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("2383BusData.txt");
		linedata.open("2383LineData.txt");
		IEEEStandardBusSystems(2383, busdata, linedata, numLines);
		answerSelect();
	}
	else if (option==5) {
		cout<<"----Polish Summer Peak 3120 bus system----"<<endl;
		linedata.open("3120LineData.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("3120BusData.txt");
		linedata.open("3120LineData.txt");
		IEEEStandardBusSystems(3120, busdata, linedata, numLines);
		answerSelect();
	}
	else if (option==6) {
		cout<<"----PEGASE 9241 bus system----"<<endl;
		linedata.open("9241linedata.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("9241busdata.txt");
		linedata.open("9241linedata.txt");
		IEEEStandardBusSystems(9241, busdata, linedata, numLines);
		answerSelect();
	}
	/*else if (option==7) {
		cout<<"----PEGASE 13659 bus system----"<<endl;
		linedata.open("13659linedata.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("13659busdata.txt");
		linedata.open("13659linedata.txt");
		IEEEStandardBusSystems(13659, busdata, linedata, numLines);
		answerSelect();
	}*/
	else if (option==7) {
		cout<<"----Case 6 bus system----"<<endl;
		linedata.open("6BusLineData.txt");
		while (getline(linedata, line)) {
			++numLines;
		}
		linedata.close();
		busdata.open("6BusData.txt");
		linedata.open("6BusLineData.txt");
		IEEEStandardBusSystems(6, busdata, linedata, numLines);
		answerSelect();
	}
}

void answerSelect() {
	char answer;
	int option;
	cout<<"\nDo you want to perform another simulation (y/n)?"<<endl;
	cin>>answer;
	if ((answer=='y') || (answer=='Y')) {
		system("CLS"); //clears text on command line interface 
		cout<<"\nPlease select one of the following options for simulation:"<<endl;
		cout<<"1. IEEE 14 bus system\n2. IEEE 118 bus system\n3. IEEE 300 bus system\n4. Polish Winter Peak 2383 Bus System\n5. Polish Summer Peak 3120 bus system\n6. PEGASE 9241 bus system\n7. PEGASE 13659 bus system"<<endl;
		cout<<"Your option: ";
		cin>>option;
		optionSelect(option);
	}
	else if((answer=='n') || (answer=='N')) {
		cout<<"Thank you for using this program"<<endl;
		exit(0); //exits program 
	}
	else {
		cout<<"Invalid response"<<endl;
		answerSelect();
	}
}

int IEEEStandardBusSystems(int numberOfBuses, ifstream &busData, ifstream &lineData, int numLines) 
{
//-------------------------------------------------------------VARIABLE DECLARATION SECTION---------------------------------------------------------------------------
	//bus data ifstream variables
	int bus_i, bustype, busDataIdx=0, lineDataIdx=0, N_g=0, N_p=0, index=0, jacSize=0, slackBus, numSlackLines=0;
	double P,Q, Vmag, Vang, VMax, VMin;
	//line data ifstream variables
	int fromBus, toBus;
	double r, x, b;

	//dynamic arrays to hold bus data
	int *busNum = new int[numberOfBuses], *busType = new int[numberOfBuses], *tempBusNum = new int[numberOfBuses];
	double *Pd = new double[numberOfBuses], *Qd = new double[numberOfBuses], *Vm=new double[numberOfBuses],
		*Va=new double[numberOfBuses], *Vmax=new double[numberOfBuses], *Vmin=new double[numberOfBuses], *theta = new double[numberOfBuses];
	
	//dynamic arrays to hold line data
	int *fromBusArr = new int[numLines], *toBusArr = new int[numLines], *PQindex;
	double *R = new double[numLines], *X = new double[numLines], *Bact = new double[numLines];
	complex<double> *B = new complex<double>[numLines]; 
	complex<double> *Z = new complex<double>[numLines];

	//Vectors needed for push_back() operations
	vector<double> Pval_spec, Qval_spec, Pval_calc, Qval_calc;
	vector<int> Pindex, Qindex;
	vector<double> V_mag, V_ang; //for constructing stateVector from hot start
	double *PQspec, *PQcalc;

	//In the linear system Ax=b, the vectors are of the same degree as the Jacobian matrix
	//double *powerMismatch;	//b in Ax = b
	//double *stateVector;		//x in Ax = b 

	//----------------------------------------------Prompt user for flat start or hot start-----------------------------------------
	/*int startType;
	cout<<"What type of simulation would you like to perform:\n1)Flat start - V = 1+j0\n2)\"Hot\" start - V magnitude and angle come from previous solution"<<endl;
	cin>>startType;*/

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

	//Reading from busdata.txt (tab delimited)
	if (!busData) {
		cout<<"There was a problem reading the 'Bus Data' file."<<endl;
		return 1;
	}

	while (busData>>bus_i>>bustype>>P>>Q>>Vmag>>Vang>>VMax>>VMin) { //infile>>a>>b extracts two integers separated by whitespace and stores them in a and b
		busNum[busDataIdx]=bus_i;
		busType[busDataIdx]=bustype;
		Pd[busDataIdx]=P/100;
		Qd[busDataIdx]=Q/100;
		Vm[busDataIdx]=Vmag;
		Va[busDataIdx]=Vang;
		Vmax[busDataIdx]=VMax;
		Vmin[busDataIdx]=VMin;
		busDataIdx++;
	}

	//For flat start, not using previous solution. V = 1+j0. Vmag = 1pu for PQ and slack buses. Vmag is known at PV buses.
	/*if (startType == 1) {
		for (int i=0; i<numberOfBuses; i++) {
			if (busType[i]!=2)
				Vm[i] = 1;
			Va[i] = 0;
		}
	}*/

	vector<int> PVbusesVec;
	//Constructing PQindex vector which holds indices of PV and PQ buses 
	for (int i=0; i<numberOfBuses; i++) {
		if (busType[i] == 1) {
			N_p++; //increment PQ bus counter since PQ is represented by 1
			Pindex.push_back(i);
			Qindex.push_back(i);
		}
		if (busType[i] == 2) {
			N_g++; //increment PV bus counter since PV is represented by 2 
			Pindex.push_back(i);
			PVbusesVec.push_back(i);
			//For a PV bus there is no initial value of Q 
		}
		if (busType[i] == 3) {
			slackBus = i;
		}
	}

	jacSize = numberOfBuses+N_p-1; //Degree of the Jacobian matrix and length of vectors in linear system
	int *PQbuses = &Qindex[0];
	int *PVbuses = &PVbusesVec[0];
	Pindex.insert(Pindex.end(), Qindex.begin(), Qindex.end()); //joins Pindex and Qindex to get a vector which holds indices of both PV and PQ buses. Store in Pindex
	PQindex = &Pindex[0]; //store vector Pindex as an array PQindex - compatible with GPU

//-------------------------------------------------------------------BUS ADMITTANCE MATRIX CONSTRUCTION--------------------------------------------------------------

	//Reading from linedata.txt (tab delimited)
	if(!lineData) {
		cout<<"There was a problem reading the 'Line Data' file"<<endl;
		return 1;
	}

	while (lineData>>fromBus>>toBus>>r>>x>>b) {
		fromBusArr[lineDataIdx] = fromBus;
		toBusArr[lineDataIdx] = toBus;
		R[lineDataIdx] = r;
		X[lineDataIdx] = x; //r+jX
		Bact[lineDataIdx] = b;
		lineDataIdx++;
	}

	for (int i=0; i<numLines; i++) {
		B[i] = complex<double> (0, Bact[i]/2);
		Z[i] = complex<double> (R[i], X[i]);
	}

	//This is used to number buses consecutively in cases where numbering is sporadic
	for (int i=0; i<numberOfBuses; i++) {
		tempBusNum[i] = i;
	}

	//Arranging bus numbering 
	for (int i=0; i<numLines; i++) {
		for (int j=0; j<numberOfBuses; j++) {
			if (fromBusArr[i] == busNum[j]) 
				fromBusArr[i] = tempBusNum[j];
			if (toBusArr[i] == busNum[j])
				toBusArr[i] = tempBusNum[j];
		}
		if (fromBusArr[i] == slackBus) {
			numSlackLines+=2;
		}
		if (toBusArr[i] == slackBus) {
			numSlackLines+=2;
		}
	}
	numSlackLines++;

	int ynnz = (2*numLines)+numberOfBuses;
	int *yrow = new int [ynnz], *ycol = new int [ynnz];
	complex<double> *y = new complex<double> [ynnz];

	startCounter();
	createYbusSparse(fromBusArr, toBusArr, yrow, ycol, y, Z, B, numLines, numberOfBuses);
	cout<<"CPU Elapsed Time - Y bus sparse: "<<getCounter()<<" ms"<<endl;

	ofstream output;
	output.open("sparseY.txt");
	for (int i=0; i<ynnz; i++)
	{
		output<<"\t"<<yrow[i]<<"\t"<<ycol[i]<<"\t"<<y[i]<<endl;
	}
	output.close();

//--------------------------------------------------------------------------Power Equations--------------------------------------------------------------------------
	
	double *Psparse = new double [numberOfBuses], *Qsparse = new double [numberOfBuses];

	for (int i=0; i<numberOfBuses; i++) {
		theta[i] = 0;
		Psparse[i] = 0;
		Qsparse[i] = 0;
	}

	startCounter();
	powerEqn(Psparse, Qsparse, Vm, theta, yrow, ycol, y, ynnz);	
	cout<<"Power eqn sparse time: "<<getCounter()<<endl;

	output.open("power equations.txt");
	for (int i=0; i<numberOfBuses; i++)
		output<<Psparse[i]<<endl;
	output<<endl;
	for (int i=0; i<numberOfBuses; i++)
		output<<Qsparse[i]<<endl;
	output.close();
	//---------------------------------------------------Formation of PQspec, PQcalc, Power Mismatch Vector and State Vector-----------------------------------
	
	//To construct the power mismatch vector, b (in Ax = b)
	//SPECIFIED values of P and Q read from text file into Pd and Qd. Place these values in separate vectors for SPECIFIED value of P and Q
	for (int i=0; i<numberOfBuses; i++) {
		if (busType[i]!=3) {
			Pval_spec.push_back(Pd[i]);
			Pval_calc.push_back(Psparse[i]);
			//V_ang.push_back(Va[i]);
			V_ang.push_back(theta[i]);
		}
		if (busType[i]==1) {
			Qval_spec.push_back(Qd[i]);
			Qval_calc.push_back(Qsparse[i]);
			V_mag.push_back(Vm[i]);
		}
	}
	//Append these vectors to get a single vector of SPECIFIED real and rxve power
	Pval_spec.insert(Pval_spec.end(), Qval_spec.begin(), Qval_spec.end()); 
	//Create an array and store the appended vector in it to use on the GPU
	PQspec = &Pval_spec[0];

	//Append these vectors to get a single vector of CALCULATED real and rxve power
	Pval_calc.insert(Pval_calc.end(), Qval_calc.begin(), Qval_calc.end()); 
	//Assign the appended vector to an array for use on the GPU
	PQcalc = &Pval_calc[0];

	//In the linear system Ax=b, the vectors are of the same degree as the Jacobian matrix
	
	double *stateVector = new double[jacSize];			//x in Ax = b
	double *powerMismatch = new double[jacSize];		//b in Ax = b
	
	//Append these vectors to get stateVector (Vang and Vmag)
	V_ang.insert(V_ang.end(), V_mag.begin(), V_mag.end());
	stateVector = &V_ang[0];

	//Power mismatch vector, b in Ax=b, is found by subtracting calculated from specified.
	output.open("StateVector.txt");
	for (int i=0; i<jacSize; i++) 
		output<<stateVector[i]<<endl;
	output.close();

	output.open ("PowerMismatch.txt");
	for (int i=0; i<jacSize; i++) {
		powerMismatch[i] = PQspec[i] - PQcalc[i];
		output<<powerMismatch[i]<<";"<<endl;
	}
	output.close();
//--------------------------------------------------------------Creation of Jacobian Matrix----------------------------------------------------------------------
	cout<<"Y nnz: "<<ynnz<<endl;
	//int nnzJac= (ynnz + numLines - numSlackLines)*4; //concise
	int nnzJac= (ynnz - numSlackLines)*4;
	//int nnzJac = 0;
	cout<<"NNZ Jac before: "<<nnzJac<<endl;	
	
	bool *boolRow = new bool[ynnz], *boolCol = new bool[ynnz];
	int *J22row = new int[ynnz], *J22col = new int[ynnz];

	bool *boolCheck = new bool[numLines];
	
	//bool *boolCol1 = new bool[ynnz];
	//bool *boolRow1 = new bool[ynnz];

	int J12count=0; int J22count = 0;
	
	/*startCounter();
	for (int i=0; i<ynnz; i++) {
		if ((yrow[i] != slackBus) &&(ycol[i] != slackBus)) {
			for (int j=0; j<N_p; j++) {
				//if ((ycol[i] == PQbuses[j]) && (real(y[i]) != 0)) {
				if (ycol[i] == PQbuses[j]) {
					boolCol1[i] = true;
					if (real(y[i]) != 0)
						J12count++;
				}
				//if ((yrow[i] == PQbuses[j]) && (real(y[i]) != 0)) {
				if (yrow[i] == PQbuses[j]) {
					boolRow1[i] = true;
					if (real(y[i]) != 0)
						J12count++;
				}
			}
			if ((boolRow1[i] == true) && (boolCol1[i] == true))
				J22count++;
		}
	}
	cout<<J12count<<endl;
	cout<<J22count<<endl;
	nnzJac = (ynnz - numSlackLines) + J12count + J22count;
	cout<<"Time taken to find nnzJac: "<<getCounter()<<endl;
	cout<<"nnzJac: "<<nnzJac<<endl;*/

	startCounter();
	for (int i=0; i<numLines; i++) {
		for (int j=0; j<N_g; j++) {
		//for (int j=0; j<N_p; j++) {
			if (yrow[i] != slackBus && ycol[i] == PVbuses[j]) {
				J12count++;
				boolCheck[i] = true;
			}
			if (ycol[i] != slackBus && yrow[i] == PVbuses[j]) {
				J12count++;
				boolCheck[i] = true;
			}
		}
		if (boolCheck[i] == true)
			J22count++;
	}
	cout<<"J12count: "<<J12count<<endl;
	cout<<"J22count: "<<J22count<<endl;
	J12count+=N_g;
	J22count*=2;
	J22count+=N_g;
	nnzJac = nnzJac - (J12count*2) - J22count;
	cout<<"Time taken to find nnzJac: "<<getCounter()<<endl;
	cout<<"nnzJac: "<<nnzJac<<endl;
	

	int *jacRow = new int[nnzJac], *jacCol = new int[nnzJac];
	double* jacVal = new double[nnzJac];

	startCounter();
	int count = 0;
	createJ11(yrow, ycol, y, jacRow, jacCol, jacVal, Vm, theta, Psparse, Qsparse, count, ynnz, slackBus);
	createJ12_J21(yrow, ycol, y, jacRow, jacCol, jacVal, Vm, theta, Psparse, Qsparse, PQbuses, J22row, J22col, boolRow, boolCol, count, ynnz, N_p, slackBus, numberOfBuses);
	createJ22(yrow, ycol, y, jacRow, jacCol, jacVal, Vm, theta, Qsparse, J22row, J22col, boolRow, boolCol, count, ynnz);		
	cout<<"Jacobian matrix using arrays: "<<getCounter()<<" ms"<<endl;

	output.open("jacSparseArrayInitialNnz.txt");
	for (int i=0; i<nnzJac; i++)
		output<<jacRow[i]<<"\t"<<jacCol[i]<<"\t"<<jacVal[i]<<endl;
	output.close();
	
	//------------------------trying something------------------------

	/*int *jacRow2 = new int[nnzJac], *jacCol2 = new int[nnzJac];
	double* jacVal2 = new double[nnzJac];

	startCounter();
	createJacobianSparse2(yrow, ycol, y, jacRow2, jacCol2, jacVal2, Vm, theta, Psparse, Qsparse, PQbuses, J22row, J22col, boolRow, boolCol, ynnz, N_p, slackBus, numberOfBuses);
	cout<<"Jacobian matrix using arrays and one loop: "<<getCounter()<<" ms"<<endl;

	output.open("jacSparseArrayTrial2.txt");
	for (int i=0; i<nnzJac; i++)
		output<<jacRow2[i]<<"\t"<<jacCol2[i]<<"\t"<<jacVal2[i]<<endl;
	output.close();
	*/
	//----------------------------------------------------------------

	int jacNnz = 0;
	for (int i=0; i<nnzJac; i++) {
		if (jacVal[i] != 0)
			jacNnz++;
	}
	double *jac2 = new double[jacNnz];
	int *jacColIdx2 = new int[jacNnz];
	int *jacRowIdx2 = new int [jacNnz];
	int j=0;
	cout<<jacNnz<<endl;
	for (int i=0; i<nnzJac; i++) {
		if (jacVal[i] !=0) {
			jacRowIdx2[j] = jacRow[i];
			jacColIdx2[j] = jacCol[i];
			jac2[j++] = jacVal[i];
		}
	}
	cout<<"NNZ After elimination of zeros: "<<jacNnz<<endl;
	output.open("jacSparseArrayAfterZeroElimination.txt");
	for (int i=0; i<jacNnz; i++)
		output<<jacRowIdx2[i]<<"\t"<<jacColIdx2[i]<<"\t"<<jac2[i]<<endl;
	output.close();

	vector<int> jacRowIdx1, jacColIdx1, yForJ22;
	vector<double> jac1;

	startCounter();
	createJacobianSparse(jacRowIdx1, jacColIdx1, yForJ22, jac1, slackBus, ynnz, N_p, N_g, yrow, ycol, y, Vm, theta, PQindex, Psparse, Qsparse, jacSize);
	cout<<"Jacobian matrix Calculation: "<<getCounter()<<endl;
	cout<<"ynnz:"<<ynnz<<endl;
	cout<<"Vector method jacNnz: "<<jac1.size()<<endl;
	cout<<jacRowIdx1.size()<<endl;
	cout<<jacColIdx1.size()<<endl;
	jacNnz = jac1.size();

	output.open("jacSparseVector.txt");
	for (int i=0; i<jacNnz; i++)
		output<<jacRowIdx1[i]<<"\t"<<jacColIdx1[i]<<"\t"<<jac1[i]<<endl;
	output.close();

	//Cast vectors to arrays
	
	double *jac = new double[jacNnz];
	int *jacColIdx = new int[jacNnz];
	int *jacRowIdx = new int [jacNnz];

	jac = &jac1[0];
	jacColIdx = &jacColIdx1[0];
	jacRowIdx = &jacRowIdx1[0];
	
//--------------------------------------------------------------SOLUTION OF LINEAR SYSTEM-----------------------------------------------------------------
	//Variables required to represent coefficient matrix and preconditioner in sparse (CSR) format
	double *csrValJac = new double[jacNnz];
	int* csrColIndJac = new int[jacNnz];
	int* csrRowPtrJac = new int[jacSize+1];

	coo2csr(jac, jacRowIdx, jacColIdx, csrValJac, csrColIndJac, csrRowPtrJac, jacNnz, jacSize);

	/*for (int i=0; i<jac1.size(); i++)
		cout<<csrColIndJac[i]<<"\t"<<jacColIdx[i]<<endl;

	for (int i=0; i<jac1.size(); i++)
		cout<<csrValJac[i]<<"\t"<<jac[i]<<endl;

	for (int i=0; i<jacSize+1; i++)
		cout<<csrRowPtrJac[i]<<endl;*/

	int lnnz = 0, unnz = 0;
	int *csrColIndU, *csrColIndL, *csrRowPtrU, *csrRowPtrL;
	double *csrValU, *csrValL;

	csrRowPtrU = new int[jacSize];
	csrRowPtrL = new int[jacSize];

	csrColIndU = csrColIndL = 0;
	csrValL = csrValU = 0;

	startCounter();
	csrILU(csrValJac, csrColIndJac, csrRowPtrJac, csrValL, csrColIndL, csrRowPtrL, csrValU, csrColIndU, csrRowPtrU, jacSize, lnnz, unnz);
	cout<<"SparseCSR ILU: "<<getCounter()<<endl;

	startCounter();
	pbiCGStab(stateVector, powerMismatch, jacSize, csrValJac, csrRowPtrJac, csrColIndJac, csrValL, csrRowPtrL, csrColIndL, csrValU, csrRowPtrU, csrColIndU);
	cout<<"Linear Solver Time elapsed: "<<getCounter()<<" ms\n"<<endl;

	
	output.open("output.txt");
	for(int i=0; i<jacSize; i++)
		output<<stateVector[i]<<endl;
	output.close();

	//check convergence
	//call Jacobian
	//loop
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
	cout<<"\nThere will be "<< numberOfBuses-1<<" P equations and "<<N_p<<" Q equations to solve."<<endl;
	cout<<"There will be a total of "<<numberOfBuses+N_p-1<<" equations for the system and "<< numberOfBuses+N_p-1<< " unknowns (V and delta) to be solved."<<endl;
	cout<<"The Jacobian matrix will be of size "<<2*(numberOfBuses-1)-N_g<<" x "<<2*(numberOfBuses-1)-N_g<<endl;

	//delete all dynamic arrays
	delete[] busNum;
	delete[] busType;
	delete[] tempBusNum;
	delete[] Pd;
	delete[] Qd;
	delete[] Vm;
	delete[] Va;
	delete[] Vmax;
	delete[] Vmin;
	delete[] fromBusArr;
	delete[] toBusArr;
	delete[] R;
	delete[] X;
	delete[] Z;
	delete[] B;
	delete[] theta;
	//delete[] PQindex;
	delete[] Bact;
	delete[] yrow;
	delete[] ycol;
	delete[] y;
	delete[] Psparse;
	delete[] Qsparse;
	//delete[] stateVector;
	delete[] powerMismatch;
	//delete[] jac;
	//delete[] jacColIdx;
	//delete[] jacRowIdx;
	/*delete[] csrValJac;
	delete[] csrColIndJac;
	delete[] csrRowPtrJac;
	delete[] csrRowPtrU;
	delete[] csrRowPtrL;*/

	return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void createYbusSparse(int *fromBusArr, int *toBusArr, int *yrow, int *ycol, complex<double> *y, complex<double> *Z, complex<double> *B, int numLines, int numberOfBuses)
{
	complex <double> temp,tempConj,tempReal,tempImag;
	for (int i=0; i<numLines; i++)
	{
		int k = fromBusArr[i];
		int j = toBusArr[i];
		yrow[i] = k;
		yrow[i+numLines] = j;
		ycol[i] = j;		
		ycol[i+numLines] = k;
	
		tempConj = conj(Z[i]);
		tempReal = real(Z[i])*real(Z[i]);
		tempImag = imag(Z[i])*imag(Z[i]);
		
		y[i] = -(tempConj)/(tempConj*Z[i]);
		y[i+numLines] = y[i];
		
		temp = ((tempConj)/(tempReal+tempImag)) + B[i];
		
		y[k+(2*numLines)] += temp;
		y[j+(2*numLines)] += temp;
	}

	for (int i=0; i<numberOfBuses; i++)
	{
		yrow[2*numLines+i] = i;
		ycol[2*numLines+i] = i;
	}
}

void powerEqn(double *Psparse, double *Qsparse, double *Vm, double *theta, int *yrow, int *ycol, complex<double> *y, int ynnz)
{
	for (int i=0; i<ynnz; i++)
	{
		Psparse[yrow[i]] += Vm[yrow[i]]*(Vm[ycol[i]]*((real(y[i])*cos(theta[yrow[i]] - theta[ycol[i]])) + imag(y[i])*sin(theta[yrow[i]] - theta[ycol[i]])));
		Qsparse[yrow[i]] += Vm[yrow[i]]*(Vm[ycol[i]]*((real(y[i])*sin(theta[yrow[i]] - theta[ycol[i]])) - imag(y[i])*cos(theta[yrow[i]] - theta[ycol[i]])));
	}
}

void createJacobianSparse(vector<int>& jacRowIdx1, vector<int>& jacColIdx1, vector<int>& yForJ22, vector<double>& jac1, int slackBus, int ynnz, int N_p, int N_g, int *yrow, int *ycol, complex<double> *y, double *Vm, double *theta, int *PQindex, double *Psparse, double *Qsparse, int jacSize)
{
	bool *boolRow = new bool[ynnz];
	bool *boolCol = new bool[ynnz];
	double tempJac = 0;
	for (int i=0; i<ynnz; i++)
	{
		//-----------------------------J11---------------------------
		if (yrow[i] != slackBus && ycol[i] != slackBus)  {

			if (yrow[i] > slackBus)  //accounting for slack bus != 1
				jacRowIdx1.push_back(yrow[i] - 1);
			else 
				jacRowIdx1.push_back(yrow[i]);
			if (ycol[i] >slackBus)
				jacColIdx1.push_back(ycol[i] - 1);
			else 												//accounting for slack bus != 1
				jacColIdx1.push_back(ycol[i]);
			
			if (yrow[i] == ycol[i]) {	//diagonal elements
				tempJac = -Qsparse[yrow[i]] - (Vm[yrow[i]]*Vm[yrow[i]]*imag(y[i]));
				jac1.push_back(tempJac);
			}
			else {	//off-diagonal
				tempJac = Vm[yrow[i]]*Vm[ycol[i]] *((real(y[i])*sin(theta[yrow[i]] - theta[ycol[i]])) - (imag(y[i]) * cos(theta[yrow[i]] - theta[ycol[i]]))); //non diag calcs
				jac1.push_back(tempJac);
			}
		
			for (int j = (N_p+N_g); j<jacSize; j++) {
				//-------------------------J12-----------------------
				if ((ycol[i] == PQindex[j]) && (real(y[i]) != 0)) {
				//if (ycol[i] == PQindex[j]) {
					jacColIdx1.push_back(j);
					if (yrow[i] > slackBus) 
						jacRowIdx1.push_back(yrow[i]-1);
					else
						jacRowIdx1.push_back(yrow[i]);	
					if (yrow[i] == ycol[i]) {		//diagonal elements
						tempJac = Vm[yrow[i]]*Vm[yrow[i]]*real(y[i]) + Psparse[yrow[i]];
						jac1.push_back(tempJac);
					} else {						//off-diagonal elements
						tempJac = Vm[yrow[i]]*Vm[ycol[i]] *((real(y[i])*cos(theta[yrow[i]] - theta[ycol[i]])) + (imag(y[i]) * sin(theta[yrow[i]] - theta[ycol[i]]))); //non diag calcs
						jac1.push_back(tempJac);
					}
				}
					//----------------------J21----------------------------
				if ((yrow[i] == PQindex[j]) && (real(y[i]) != 0)) {
				//if (yrow[i] == PQindex[j]) {
					jacRowIdx1.push_back(j);
					if (ycol[i] > slackBus) 
						jacColIdx1.push_back(ycol[i]-1);
					else
						jacColIdx1.push_back(ycol[i]);	
					if (yrow[i] == ycol[i]) {		//diagonal elements
						tempJac = Psparse[yrow[i]] - Vm[yrow[i]]*Vm[yrow[i]]*real(y[i]);
						jac1.push_back(tempJac);
					} else {						//off-diagonal elements
						tempJac = -Vm[yrow[i]]*Vm[ycol[i]] *((real(y[i])*cos(theta[yrow[i]] - theta[ycol[i]])) + (imag(y[i]) * sin(theta[yrow[i]] - theta[ycol[i]]))); //non diag calcs
						jac1.push_back(tempJac);
					}
				}

				if (yrow[i] == PQindex[j])
					boolRow[i] = true;
				if (ycol[i] == PQindex[j])
					boolCol[i] = true;
			}
			if (boolRow[i] == true && boolCol[i] == true) {
				yForJ22.push_back(i);
			}
		}
	}

	//---------------------------J22-----------------------------
	for (unsigned int i = 0; i<yForJ22.size(); i++)
	{
		int idx = yForJ22[i];
		if (yrow[idx] == ycol[idx]) {
			for (int j = (N_p+N_g); j<jacSize; j++) {
				if (yrow[idx] == PQindex[j])
					jacRowIdx1.push_back(j);
				if (ycol[idx] == PQindex[j])
					jacColIdx1.push_back(j);
			}
			tempJac = Qsparse[yrow[idx]] - Vm[yrow[idx]]*Vm[yrow[idx]]*imag(y[idx]);
			jac1.push_back(tempJac);
		}
		else {
			for (int j = (N_p+N_g); j<jacSize; j++) {
				if (yrow[idx] == PQindex[j])
					jacRowIdx1.push_back(j);
				if (ycol[idx] == PQindex[j])
					jacColIdx1.push_back(j);
			}
			tempJac = Vm[yrow[idx]]*Vm[ycol[idx]]*((real(y[idx])*sin(theta[yrow[idx]]-theta[ycol[idx]]))-(imag(y[idx])*cos(theta[yrow[idx]]-theta[ycol[idx]])));
			jac1.push_back(tempJac);
		}
	}
}

void createJacobianSparse2(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, 
	int *PQbuses, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int ynnz, int N_p, int slackBus, int numberOfBuses)
{
	int count=0;
	for (int yIdx=0; yIdx<ynnz; yIdx++) 
	{
		if (yrow[yIdx] != slackBus && ycol[yIdx] != slackBus) {
			if (yrow[yIdx] < slackBus)
				jacRow[count] = yrow[yIdx];
			else
				jacRow[count] = yrow[yIdx] - 1;

			if (ycol[yIdx] < slackBus)
				jacCol[count] = ycol[yIdx];
			else
				jacCol[count] = ycol[yIdx] - 1;

			if (yrow[yIdx] == ycol[yIdx]) //J11 diagonal calculations
				jacVal[count] = -Qsparse[yrow[yIdx]] - (Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*imag(y[yIdx]));
			else //J11 off-diagonal calculations
				jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*sin(theta[yrow[yIdx]] - theta[ycol[yIdx]])) - (imag(y[yIdx]) * cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])));
			count++;
		
			for (int PQidx = 0; PQidx<N_p; PQidx++) {
				//--------------J12------------------
				if (ycol[yIdx] == PQbuses[PQidx]) {
					boolCol[yIdx] = true;
					jacCol[count] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] < slackBus)
						jacRow[count] = yrow[yIdx];
					else
						jacRow[count] = yrow[yIdx] - 1;
					J22col[yIdx] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] == ycol[yIdx])
						jacVal[count] = Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*real(y[yIdx]) + Psparse[yrow[yIdx]];
					else 
						jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])) + (imag(y[yIdx]) * sin(theta[yrow[yIdx]] - theta[ycol[yIdx]]))); //non diag calcs
					count++;
				}

				//--------------J21------------------
				if (yrow[yIdx] == PQbuses[PQidx]) {
					boolRow[yIdx] = true;
					jacRow[count] = numberOfBuses + PQidx - 1;
					if (ycol[yIdx] < slackBus)
						jacCol[count] = ycol[yIdx];
					else
						jacCol[count] = ycol[yIdx] - 1;
					J22row[yIdx] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] == ycol[yIdx])
						jacVal[count] = Psparse[yrow[yIdx]] - Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*real(y[yIdx]);
					else 
						jacVal[count] = -Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])) + (imag(y[yIdx]) * sin(theta[yrow[yIdx]] - theta[ycol[yIdx]]))); //non diag calcs
					count++;
				}
			}

			if (boolRow[yIdx] == true && boolCol[yIdx] == true) {
				jacRow[count] = J22row[yIdx];
				jacCol[count] = J22col[yIdx];
				if (yrow[yIdx] == ycol[yIdx])
					jacVal[count] = Qsparse[yrow[yIdx]] - Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*imag(y[yIdx]);
				else
					jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]]*((real(y[yIdx])*sin(theta[yrow[yIdx]]-theta[ycol[yIdx]]))-(imag(y[yIdx])*cos(theta[yrow[yIdx]]-theta[ycol[yIdx]])));
				count++;
			}
		}
	}
}

void createJ11(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, int &count, int ynnz, int slackBus)
{
	for (int yIdx=0; yIdx<ynnz; yIdx++) 
	{
		if (yrow[yIdx] != slackBus && ycol[yIdx] != slackBus) {
			if (yrow[yIdx] < slackBus)
				jacRow[count] = yrow[yIdx];
			else
				jacRow[count] = yrow[yIdx] - 1;

			if (ycol[yIdx] < slackBus)
				jacCol[count] = ycol[yIdx];
			else
				jacCol[count] = ycol[yIdx] - 1;

			if (yrow[yIdx] == ycol[yIdx]) //J11 diagonal calculations
				jacVal[count] = -Qsparse[yrow[yIdx]] - (Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*imag(y[yIdx]));
			else //J11 off-diagonal calculations
				jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*sin(theta[yrow[yIdx]] - theta[ycol[yIdx]])) - (imag(y[yIdx]) * cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])));
			count++;
		}
	}
}

void createJ12_J21(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Psparse, double *Qsparse, int *PQbuses, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int &count, int ynnz, int N_p, int slackBus, int numberOfBuses)
{
	for (int yIdx=0; yIdx<ynnz; yIdx++) {	
		if (yrow[yIdx] != slackBus && ycol[yIdx] != slackBus) {
			for (int PQidx = 0; PQidx<N_p; PQidx++) {
				//--------------J12------------------
				if (ycol[yIdx] == PQbuses[PQidx]) {
					boolCol[yIdx] = true;
					jacCol[count] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] < slackBus)
						jacRow[count] = yrow[yIdx];
					else
						jacRow[count] = yrow[yIdx] - 1;
					J22col[yIdx] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] == ycol[yIdx])
						jacVal[count] = Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*real(y[yIdx]) + Psparse[yrow[yIdx]];
					else 
						jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])) + (imag(y[yIdx]) * sin(theta[yrow[yIdx]] - theta[ycol[yIdx]]))); //non diag calcs
					count++;
				}

				//--------------J21------------------
				if (yrow[yIdx] == PQbuses[PQidx]) {
					boolRow[yIdx] = true;
					jacRow[count] = numberOfBuses + PQidx - 1;
					if (ycol[yIdx] < slackBus)
						jacCol[count] = ycol[yIdx];
					else
						jacCol[count] = ycol[yIdx] - 1;
					J22row[yIdx] = numberOfBuses + PQidx - 1;
					if (yrow[yIdx] == ycol[yIdx])
						jacVal[count] = Psparse[yrow[yIdx]] - Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*real(y[yIdx]);
					else 
						jacVal[count] = -Vm[yrow[yIdx]]*Vm[ycol[yIdx]] *((real(y[yIdx])*cos(theta[yrow[yIdx]] - theta[ycol[yIdx]])) + (imag(y[yIdx]) * sin(theta[yrow[yIdx]] - theta[ycol[yIdx]]))); //non diag calcs
					count++;
				}
			}
		}
	}
}

void createJ22(int *yrow, int *ycol, complex<double> *y, int *jacRow, int *jacCol, double *jacVal, double *Vm, double *theta, double *Qsparse, int *J22row, int *J22col, bool *boolRow, bool* boolCol, int &count, int ynnz)
{
	for (int yIdx=0; yIdx<ynnz; yIdx++) {
		if (boolRow[yIdx] == true && boolCol[yIdx] == true) {
			jacRow[count] = J22row[yIdx];
			jacCol[count] = J22col[yIdx];
			if (yrow[yIdx] == ycol[yIdx])
				jacVal[count] = Qsparse[yrow[yIdx]] - Vm[yrow[yIdx]]*Vm[yrow[yIdx]]*imag(y[yIdx]);
			else
				jacVal[count] = Vm[yrow[yIdx]]*Vm[ycol[yIdx]]*((real(y[yIdx])*sin(theta[yrow[yIdx]]-theta[ycol[yIdx]]))-(imag(y[yIdx])*cos(theta[yrow[yIdx]]-theta[ycol[yIdx]])));
			count++;
		}
	}	
}

double dot(double* vec1, double* vec2, int n)
{
	double dotProd = 0;
	for (int i=0; i<n; i++)
	{
		dotProd += vec1[i]*vec2[i];
	}
	return dotProd;
}

double norm(double* vec1, int n)
{
	double val = dot(vec1, vec1, n);
	double nrm = sqrt(val);
	return nrm;
}

void axpy(double a, double* x, double* y, int n) //y = ax+y
{
	for (int i=0; i<n; i++) {
		y[i] = a*x[i] + y[i];
	}
}

void biCG(double* x, double* b, int n, double* csrValA, int* csrRowPtrA, int* csrColIndA)
{
	//vector variables 
	double *r, *r_tld, *p, *p_hat, *s, *s_hat, *t, *v;
	//include preconditioner

	//allocating memory for vectors
	r = new double[n];
	r_tld = new double[n];
	p = new double[n];
	p_hat = new double[n];
	s = new double[n];
	s_hat = new double[n];
	t = new double[n];
	v = new double[n];

	//scalar variables
	double rho=0, beta=0, rho_1=1, alpha=1, omega=1, snorm, resid;
	int iter; 

	//Initialize variables
	for (int i=0; i<n; i++) {
		r[i] = 0;
		v[i] = 1;
		s[i] = 0;
		t[i] = 0;
		p_hat[i] = 0;
		s_hat[i] = 0;
		t[i] = 0;
	}

	spMVcsr(csrValA, csrRowPtrA, csrColIndA, x, r, n);
	for (int i=0; i<n; i++) {
		r[i] = b[i] - r[i]; //r = b - Ax
		r_tld[i] = r[i];
	}

	//Find norm of RHS vector
	double bnrm = norm(b, n);
	if (bnrm == 0)
		bnrm = 1.0;

	double error = norm(r, n)/bnrm;
	if (error <= Tol) //x is close enough to its solution already
		return;

	for (iter=0; iter<max_it; iter++)
	{
		rho = dot(r_tld, r, n);
		if (rho == 0) //if rho is 0 this implies breakdown => break out of the loop
			break;

		if (iter>0) 
		{
			beta = (rho/rho_1)*(alpha/omega);
			axpy((-omega), v, p, n); //p = p - omega*v
			for (int j=0; j<n; j++)
				p[j] = r[j] + beta*p[j]; //p = r + beta*p
		}
		else //only on the first iteration p=r
		{
			for (int j=0; j<n; j++) {
				p[j] = r[j]; 
			}
		}

		spMVcsr(csrValA, csrRowPtrA, csrColIndA, p, v, n);
		alpha = rho/dot(r_tld, v, n);
		 
		for (int j=0; j<n; j++)
			s[j]=r[j] - alpha*v[j]; //s = r-alpha*v

		snorm = norm(s, n);
		if (snorm < Tol) { //initial snorm check
			axpy(alpha, p, x, n);
			resid = snorm/bnrm;
			break;
		}

		spMVcsr(csrValA, csrRowPtrA, csrColIndA, s, t, n);
		omega = dot(t, s, n)/dot(t, t, n);

		for (int j=0; j<n; j++) {
			x[j] = x[j] + alpha*p[j] + omega*s[j];
			r[j] = s[j] - omega*t[j];
		}
		 
		//final convergence checks 
		error = norm(r, n)/bnrm; 
		if (snorm <= Tol)
			break;
		if (error<=Tol)
			break;
		if (omega == 0)
			break;

		rho_1 = rho;
		cout<<"Rho_1 = "<<rho_1<<endl;
	}

	int flag;
	if (error<=Tol || snorm<=Tol) {
		flag = 0;
	}
	else if (omega==0) {
		flag = -2;
	}
	else if (rho == 0) {
		flag = -1;
	}
	else {
		flag = 1;
	}

	if (!flag)
		cout<<"BiCGStab produced answer with residual "<<resid<<" in "<<iter<<" iterations"<<endl;
	else
		cout<<"BiCGStab produced error "<<flag<<" after "<<iter<<" iterations."<<endl;
}

void spMVcsr(double csrValA[], int csrRowPtrA[], int csrColIndA[], double x[], double y[], int rows)
{
	//Finds y = Ax
	for (int i=0; i<rows; i++) //c1N
		y[i]=0;
	for (int i=0; i<rows; i++) //c2N
	{
		for (int k= csrRowPtrA[i]; k<csrRowPtrA[i+1]; k++) //c3nnzPerRow
		{
			y[i]+= csrValA[k]*x[csrColIndA[k]]; //c4
		}
	}
}

void dense2csr(double *a, int nnz, int rows, int cols, double csrValA[], int csrRowPtrA[], int csrColIndA[])
{
	int k=0,m=-1,n=0;
	//Loop through matrix, find non zero elements, store values and colIdx
	for (int i=0; i<rows; i++) {
		for (int j=0; j<cols; j++) {
			if (a[i*rows+j] != 0) {
				csrValA[k] = a[i*rows+j];
				csrColIndA[k] = j;
				if(m<0)
					m=k;				
				k++;
			}
		}//end for j
		if(m>=0)csrRowPtrA[n] = m;
		else csrRowPtrA[n] = 0;
		n++;
		m=-1;
	}

	csrRowPtrA[rows] = nnz;
}

void coo2csr(double *cooVal, int *cooRowIdx, int *cooColIdx,double *csrVal, int *csrColIdx, int *csrRowPtr, int nnz, int numRows)
{
	int csrRowPtrCounter = 0;
	int count = 0;
	int idx=0;
	for (int i=0; i<numRows; i++)
	{
		csrRowPtr[count++] = csrRowPtrCounter;
		for (int j=0; j<nnz; j++)
		{
			if (cooRowIdx[j] == i) {
				csrVal[idx] = cooVal[j];
				csrColIdx[idx++] = cooColIdx[j];
				csrRowPtrCounter++;
			}
		}
	}
	csrRowPtr[count] = nnz;
}

int findNNZ(double* A, int n)
{
	int nnz=0;
	for(int i=0; i<n; i++){
		for (int j=0; j<n; j++) {
			if (A[i*n+j] !=0)
				nnz++;
		}
	}
	return nnz;
}

void triangularSolve(double *csrValL, int* csrRowPtrL, int* csrColIndL, double *csrValU, int* csrRowPtrU, int* csrColIndU, double* Udiag, double* x, double *b, int n)
{
	double sum;
	//double diag;
	double *z = new double[n];
	for (int i=0; i<n; i++) {
		z[i]=0; //intermediate vector
		x[i]=0;
	}
	//Solve Lz = b
	for (int i=0; i<n; i++) {
		sum=0;
		for (int k=csrRowPtrL[i]; k<csrRowPtrL[i+1]; k++)
			sum+=csrValL[k] * z[csrColIndL[k]];
		z[i] = (b[i] - sum);
	}

	//Solve Ux = z
	for (int i=n-1; i>=0; i--){
		sum=0;
		for (int k=csrRowPtrU[i]; k<csrRowPtrU[i+1]; k++) {
			sum+=csrValU[k] * x[csrColIndU[k]];
		}
		x[i] = (z[i]-sum)/Udiag[i]; //x[] is final result
	}
}

void csrILU(double* val, int* col, int* rowPtr, double* &lval, int* &lcol, int* lptr, double* &uval, int* &ucol, int* uptr, int numRows, int &lnnz, int &unnz)
{
	int i,j,k,pn,qn,rn;
	double multiplier;


	//Get size of L and U
	for (i=0; i<numRows; i++) {
		for (j=rowPtr[i]; j<rowPtr[i+1]; j++) {
			if (col[j] < i)
				lnnz++;
			else
				unnz++;
		}
	}

	lval = new double[lnnz];
	uval = new double[unnz];
	lcol = new int[lnnz];
	ucol = new int[unnz];

	lptr[0] = uptr[0] = 0;

	for (i=0; i<numRows; i++) {
		lptr[i+1] = lptr[i];
		uptr[i+1] = uptr[i];

		for (j=rowPtr[i]; j<rowPtr[i+1]; j++)
			if (col[j]<i) {
				k = lptr[i+1]++;
				lval[k] = val[j];
				lcol[k] = col[j];
			} else if (col[j] >=i) {
				k = uptr[i+1]++;
				uval[k] = val[j];
				ucol[k] = col[j];
			}
	}

	//Sort row major by columns
	for (i=0; i<numRows; i++) {
		QSort(lcol, lval, lptr[i], lptr[i+1] - lptr[i]);
		QSort(ucol, uval, uptr[i], uptr[i+1] - uptr[i]);
	}

	//Factor matrix
	for (i=1; i<numRows; i++) {
		for (int j=lptr[i]; j<lptr[i+1]; j++) {
			pn = uptr[lcol[j]];
			multiplier = (lval[j]/=uval[pn]);

			qn = j+1;
			rn = uptr[i];

			for (pn++; ucol[pn]<i && pn<uptr[lcol[j]+1]; pn++) {
				while (lcol[qn]<ucol[pn] && qn<lptr[i+1])
					qn++;
				if (ucol[pn] == lcol[qn] && qn<lptr[i+1])
					lval[qn] -= multiplier * uval[pn];
			}
			for (; pn<uptr[lcol[j]+1]; pn++) {
				while (ucol[rn] < ucol[pn] && rn<uptr[i+1])
					rn++;
				if (ucol[pn] == ucol[rn] && rn<uptr[i+1])
					uval[rn] -= multiplier * uval[pn];
			}
		}
	}
	ofstream lower("iluLow.txt");
	ofstream upper("iluUp.txt");
	for (int i=0; i<lnnz; i++)
		lower<<lcol[i]<<"\t"<<lval[i]<<endl;
	for (int i=0; i<numRows; i++)
		lower<<lptr[i]<<endl;
	lower.close();

	for (int i=0; i<unnz; i++)
		upper<<ucol[i]<<"\t"<<uval[i]<<endl;
	for (int i=0; i<numRows; i++)
		upper<<uptr[i]<<endl;
	upper.close();
}

int QSort(int *v, double* x, int base_ptr, int total_elems)
{
	int pivot_buffer;
	double pixot_buffer;

	if (total_elems > MAX_THRESH) {

		int lo = base_ptr;
		int hi = lo + total_elems - 1;

		stack_node stack[STACK_SIZE];
		stack_node *top = stack + 1;

		while (STACK_NOT_EMPTY) {
			int left_ptr;
			int right_ptr;
			{
				{
					int mid = lo + (hi - lo) / 2;

					if (CMP(v[mid], v[lo])) {
						swap_int (v[mid], v[lo]);
						SWAP_double (x[mid], x[lo]);
					}
					if (CMP(v[hi], v[mid])) {
						swap_int(v[hi], v[mid]);
						SWAP_double (x[hi], x[mid]);
					}
					else
						goto jump_over;

					if (CMP(v[mid], v[lo])) {
						swap_int (v[mid], v[lo]);
						SWAP_double (x[mid], x[lo]);
					}

jump_over:
					pivot_buffer = v[mid];
					pixot_buffer = x[mid];
				}

				left_ptr = lo + 1;
				right_ptr = hi - 1;

				do {
					while (CMP(v[left_ptr], pivot_buffer))
						left_ptr++;

					while (CMP(pivot_buffer, v[right_ptr]))
						right_ptr--;

					if (left_ptr < right_ptr) {
						swap_int (v[left_ptr], v[right_ptr]);
						SWAP_double (x[left_ptr], x[right_ptr]);
						left_ptr++;
						right_ptr--;
					}
					else if (left_ptr == right_ptr) {
						left_ptr++;
						right_ptr--;
						break;
					}
				} while (left_ptr <= right_ptr);
			}

			if ((right_ptr - lo) <= MAX_THRESH) {
				if ((hi - left_ptr) <= MAX_THRESH)
					POP (lo,hi);
				else
					lo = left_ptr;
			}
			else if ((hi - left_ptr) <= MAX_THRESH)
				hi = right_ptr;
			else if ((right_ptr - lo) > (hi - left_ptr)) {
				PUSH(lo, right_ptr);
				lo = left_ptr;
			}
			else {
				PUSH (left_ptr, hi);
				hi = right_ptr;
			}
		}
	}
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

	{
		int end_ptr = base_ptr + total_elems - 1;
		int run_ptr;
		int tmp_ptr = base_ptr;
		int thresh = MIN (end_ptr, base_ptr + MAX_THRESH);

		for (run_ptr = tmp_ptr + 1; run_ptr <= thresh; run_ptr++)
			if (CMP(v[run_ptr], v[tmp_ptr]))
				tmp_ptr = run_ptr;

		if (tmp_ptr != base_ptr) {
			swap_int(v[tmp_ptr], v[base_ptr]);
			SWAP_double(x[tmp_ptr], x[base_ptr]);
		}

		for (run_ptr = base_ptr + 1; (tmp_ptr = run_ptr += 1) <= end_ptr; ) {
			while (CMP(v[run_ptr], v[tmp_ptr-=1]))
				;
			if ((tmp_ptr += 1) != run_ptr) {
				int trav;

				for (trav = run_ptr + 1; --trav >= run_ptr;) {
					int c;
					double d;
					c = v[trav];
					d = x[trav];
					int hi, lo;

					for (hi = lo = trav; (lo -= 1) >= tmp_ptr; hi = lo) {
						v[hi] = v[lo];
						x[hi] = x[lo];
					}
					v[hi] = c;
					x[hi] = d;
				}
			}
		}
	}
	return 1;
}

void lowerTriangularSolve(int m, double *val, int* indx, int* pntr, double* b, double* c)
{
	double z;

	c-=1;
	val-= pntr[0];
	indx-=pntr[0];

	for(int i=0; i<m; i++) {
		z=0;
		for (int j=pntr[i]; j<pntr[i+1]; j++)
			z+= c[indx[j]] * val[j];
		c[i+pntr[0]] = b[i] - z;
	}
}

void upperTriangularSolve(int m, double *val, int* indx, int*pntr, double* b, double* c)
{
	double z;

	c-=1;
	val-= pntr[0];
	indx -= pntr[0];

	for (int i= m-1; i>= 0; i--) {
		z = 0;
		for (int j = pntr[i] + 1; j< pntr[i+1]; j++)
			z += c[indx[j]] * val[j];
		c[i+pntr[0]] = (b[i] - z) / val[pntr[i]];
	}
}

void printMatrix(double* mat, int n)
{
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			cout<<mat[i*n+j]<<" ";
		}
		cout<<endl;
	}
	cout<<endl;
}

void printVector(double* vec, int n)
{
	cout.precision(3);
	for (int i=0; i<n; i++)
		cout<<vec[i]<<fixed<<endl;
	cout<<endl;
}

void findDiag(double *A, int n, double* diag)
{
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			if (i==j)
				diag[i] = A[i*n+j];
		}
	}
}

void pbiCGStab(double* x, double* b, int n, double* csrValA, int* csrRowPtrA, int* csrColIndA, double* csrValL, int* csrRowPtrL, int* csrColIndL, double* csrValU, int* csrRowPtrU, int* csrColIndU)
{
	//vector variables 
	double *r, *r_tld, *p, *p_hat, *s, *s_hat, *t, *v, *xhalf;
	//include preconditioner

	//allocating memory for vectors
	r = new double[n];
	r_tld = new double[n];
	p = new double[n];
	p_hat = new double[n];
	s = new double[n];
	s_hat = new double[n];
	t = new double[n];
	v = new double[n];
	xhalf = new double[n];
	//M - preconditioner

	//scalar variables
	double rho=1, beta=0, rho_1, alpha=1, omega=1, snorm, resid=0;
	int iter;

	//Initialize variables
	for (int i=0; i<n; i++) {
		r[i] = 0;
		v[i] = 1;
		s[i] = 0;
		t[i] = 0;
		p_hat[i] = 0;
		s_hat[i] = 0;
	}

	spMVcsr(csrValA, csrRowPtrA, csrColIndA, x, r, n);
	for (int i=0; i<n; i++) {
		r[i] = b[i] - r[i]; //r = b - Ax
		r_tld[i] = r[i];
	}

	//Find norm of RHS vector
	double bnrm = norm(b, n);
	if (bnrm == 0)
		bnrm = 1.0;

	double error = norm(r, n)/bnrm;
	if (error <= Tol) //x is close enough to its solution already
		return;

	for (iter=0; iter<max_it; iter++)
	{
		rho_1 = rho;
		rho = dot(r_tld, r, n);
		if (rho == 0) //if rho is 0 this implies breakdown => break out of the loop
			break;

		if (iter>0) 
		{
			beta = (rho/rho_1)*(alpha/omega);
			axpy((-omega), v, p, n); //p = p - omega*v
			for (int j=0; j<n; j++)
				p[j] = r[j] + beta*p[j]; //p = r + beta*p
		}
		else //only on the first iteration p=r
		{
			for (int j=0; j<n; j++) {
				p[j] = r[j]; 
			}
		}

		//preconditioner with p_hat
		//triangularSolve(csrValL, csrRowPtrL, csrColIndL, csrValU, csrRowPtrU, csrColIndU, Udiag, p_hat, p, n);
		lowerTriangularSolve(n, csrValL, csrColIndL, csrRowPtrL, &p[0], &p_hat[1]);
		upperTriangularSolve(n, csrValU, csrColIndU, csrRowPtrU, &p_hat[0], &p_hat[1]);

		spMVcsr(csrValA, csrRowPtrA, csrColIndA, p_hat, v, n);

		/*ofstream phat("v.txt");
		for (int i=0; i<n; i++)
			phat<<v[i]<<endl;
		phat.close();*/
		
		alpha = rho/dot(r_tld, v, n);

		for(int i=0; i<n; i++)
			xhalf[i] = x[i] + alpha*p_hat[i];
		 
		for (int j=0; j<n; j++)
			s[j]=r[j] - alpha*v[j]; //s = r-alpha*v

		snorm = norm(s, n);
		if (snorm < Tol) { //initial snorm check
			axpy(alpha, p_hat, x, n);
			resid = snorm/bnrm;
			break;
		}

		//preconditioner with s_hat
		//triangularSolve(csrValL, csrRowPtrL, csrColIndL, csrValU, csrRowPtrU, csrColIndU, Udiag, s_hat, s, n);
		lowerTriangularSolve(n, csrValL, csrColIndL, csrRowPtrL, &s[0], &s_hat[1]);
		upperTriangularSolve(n, csrValU, csrColIndU, csrRowPtrU, &s_hat[0], &s_hat[1]);
		spMVcsr(csrValA, csrRowPtrA, csrColIndA, s_hat, t, n);

		omega = dot(t, s, n)/dot(t, t, n);

		for (int j=0; j<n; j++) {
			x[j] = xhalf[j] + omega*s_hat[j];
			r[j] = s[j] - omega*t[j];
		}
		 
		//final convergence checks 
		error = norm(r, n)/bnrm; 
		if (snorm <= Tol)
			break;
		if (error<=Tol)
			break;
		if (omega == 0)
			break;
	}

	int flag;
	if (error<=Tol || snorm<=Tol) {
		flag = 0;
	}
	else if (omega==0) {
		flag = -2;
	}
	else if (rho == 0) {
		flag = -1;
	}
	else {
		flag = 1;
	}

	if (!flag)
		cout<<"BiCGStab produced answer with residual "<<error<<" in "<<iter<<" iterations"<<endl;
	else
		cout<<"BiCGStab produced error "<<flag<<" after "<<iter<<" iterations."<<endl;
}