#include <ilcplex/cplex.h>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <iomanip>
#include <windows.h>
#include <tchar.h>
using namespace std;
static int populatebyrow(CPXENVptr env, CPXLPptr lp);

static void usage(char *progname);



long I, J, J1, J2, K, K1, K2;
long SC_cost, BS_cost, SC_cap, BS_cap, BS_rad;
std::vector<std::vector<long> > N;
std::vector<std::vector<long> > D;
std::vector<std::vector<int>> Yjk;
long X_beg;
long Y_beg;
long B_beg;
long M_beg;
long Z_beg;
long NUMCOLS;
vector<string> ret;

void dirList(string dir, string instName)
{
	
	WIN32_FIND_DATA ffd;
	wchar_t path[100];
	int i = 0;
	do
	{
		path[i] = dir.c_str()[i];
	} while (dir.c_str()[i++] != 0);

	if (path[_tcslen(path) - 1] != '\\')
		_tcscat(path, _T("\\"));
	_tcscat(path, _T("*.*"));

	HANDLE hFind = FindFirstFile(path, &ffd);
	if (hFind == INVALID_HANDLE_VALUE)
	{
		//cerr << _T("Invalid handle value.") << GetLastError() << endl;
		return ;
	}

	bool finished = false;
	bool diffe;
	while (!finished)
	{
		//cout << ffd.cFileName << endl;
		char name[50];
		int i = 0;
		do
		{
			name[i] = ffd.cFileName[i];
		} while (ffd.cFileName[i++] != 0);
		if (instName.size() > 0)
		{
			if (name[0] == '.')
			{
				if (!FindNextFile(hFind, &ffd))
					finished = true;
				continue;
			}
			diffe = false;
			for (int i = 0; i < instName.size() && !diffe; i++)
			{
				if (instName[i] != name[i])
				{
					if (!FindNextFile(hFind, &ffd))
						finished = true;
					diffe = true;
				}
			}
			if (diffe)
				continue;
		}
		ret.push_back(name);
		if (ret[ret.size() - 1][0] == '.')
			ret.erase(ret.begin() + ret.size() - 1);

		if (!diffe && !FindNextFile(hFind, &ffd))
			finished = true;
	}
}


int main(int argc, char *argv[])
{
	string dir = "instance";
	string name = "extraExtraLarge";
	dirList(dir,name);
	ofstream out, outExt;
	ostringstream s1,s2;
	s1 << "cplex" << name<<".txt";
	s2 << s1.str() << "Ext" << ".txt";
	out.open(s1.str());
	outExt.open(s2.str());
	for (int i = 0; i < ret.size(); i++)
	{
		time_t startTime = clock();
		std::ifstream in;
		
		CPXENVptr     env = NULL;
		CPXLPptr      lp = NULL;
		int           status = 0;
		ostringstream s;
		s << dir <<"/" << ret[i] ;
		string instName = s.str();

		/* Initialize the variables */
		
		in.open(instName, std::fstream::in);
		
		//out <<  inst<<" ";
		in >> K1 >> K2 >> J1 >> J2 >> I >> SC_cost >> BS_cost >> SC_cap >> BS_cap >> BS_rad;
		K = K1 + K2;
		J = J1 + J2;

		int t;
		Yjk.resize(J1);
		for (int j = 0; j < J1; j++)
		{
			Yjk[j].resize(K1);
			for (int k = 0; k < K1; k++)
			{
				in >> t;
				Yjk[j][k] = t;
			}
		}

		N.resize(J);
		for (int j = 0; j < J2; j++)
		{
			N[j].resize(K);
			for (int k = 0; k < K; k++)
				in >> N[j][k];
		}
		D.resize(I);
		for (int i = 0; i < I; i++)
		{
			D[i].resize(J);
			for (int j = 0; j < J; j++)
				in >> D[i][j];
		}

		in.close();
		/* Initialize the CPLEX environment */

		env = CPXopenCPLEX(&status);
		if (env == NULL)
		{
			cout << "greska!!!!" << endl;
			std::cerr << "Failure to create environment, error %d." << std::endl;
			goto TERMINATE;
		}



		status = CPXsetdblparam(env, CPX_PARAM_TILIM, 3600);
		if (status) {
			cout << "greska!!!!" << endl;
			cerr << "Failure to set time limit, error %d." << endl;
			goto TERMINATE;
		}

		/* Create the problem. */

		lp = CPXcreateprob(env, &status, "optimal.wireless.network.design");

		if (lp == NULL) {
			cout << "greska!!!!" << endl;
			cerr << "Failed to create LP." << endl;
			goto TERMINATE;
		}
		/* populate by row */
		status = populatebyrow(env, lp);

		if (status) {
			cout << "greska!!!!" << endl;
			cerr << "Failed to populate problem." << endl;
			goto TERMINATE;
		}

		//CPXwriteprob(env, lp, "problem.lp", "LP");



		CPXchgprobtype(env, lp, CPXPROB_MILP);
		CPXmipopt(env, lp);
		
		double objval;
		double *x = (double *)malloc(NUMCOLS * sizeof(double));
		outExt << ret[i] << endl;
		//cout << instances[i] << endl;
		if (CPXsolution(env, lp, NULL, &objval, x, NULL, NULL, NULL))
		{
			out << "No solution" << endl;
			
			goto TERMINATE;
		}
		

		//if (CPXgetobjval(env, lp, &objval))
		
		
		/*ostringstream s1;
		s1 << "sol" << inst;
		CPXsolwrite(env, lp, s1.str().c_str());
		*/
		int cvorova = CPXgetnodecnt(env, lp);

		int iteracija = CPXgetmipitcnt(env, lp);
		
		out << (int)objval << " " << std::setprecision(8) << (clock() - startTime) / 1000.0 << endl;
		outExt << (int)objval << " " << std::setprecision(8) << (clock() - startTime) / 1000.0 << endl;

		outExt << "bs: ";
		for (int i = 0; i < J2; i++)
		{
			if (x[B_beg + i] == 1)
			{
				outExt << i+J1 << "->";
				for (int j = 0; j < K; j++)
				{
					if (x[Y_beg + i*K + j] == 1)
						outExt << j << " ";
				}
			}
		}
		outExt << endl << "sc: ";
		for (int i = 0; i < K2; i++)
		{
			//out << x[M_beg + i] << " ";
			if (x[M_beg + i] == 1)
				outExt << i + K1<< " ";
		}
		outExt << endl;
		
		

	TERMINATE:

		/* Free up the problem as allocated by CPXcreateprob, if necessary */

		if (lp != NULL) {
			status = CPXfreeprob(env, &lp);
			if (status) {
				fprintf(stderr, "CPXfreeprob failed, error code %d.\n", status);
			}
		}

		/* Free up the CPLEX environment, if necessary */

		if (env != NULL) {
			status = CPXcloseCPLEX(&env);

			/* Note that CPXcloseCPLEX produces no output,
			so the only way to see the cause of the error is to use
			CPXgeterrorstring.  For other CPLEX routines, the errors will
			be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

			if (status) {
				char  errmsg[CPXMESSAGEBUFSIZE];
				fprintf(stderr, "Could not close CPLEX environment.\n");
				CPXgeterrorstring(env, status, errmsg);
				fprintf(stderr, "%s", errmsg);
			}
		}

	}
	out.close();
	outExt.close();
	return 0;

}  /* END main */

static void free_and_null(int *rmatind, double *rmatval)
{
	delete[] rmatval; rmatval = NULL;
	delete[] rmatind; rmatind = NULL;
} /* END free_and_null */

static void usage(char *progname)
{
	fprintf(stderr, "Usage: %s X\n", progname);
	fprintf(stderr, "   where X is one of the instance names \n");
	fprintf(stderr, " Exiting...\n");
} /* END usage */

static int populatebyrow(CPXENVptr env, CPXLPptr lp)
{
	X_beg = 0;
	Y_beg = I*J;
	B_beg = Y_beg + J2*K;
	M_beg = B_beg + J2;
	Z_beg = M_beg + K2;
	NUMCOLS = Z_beg + I*J2*K;
	int      status = 0;
	double   *obj = new double[NUMCOLS];
	double   *lb = new double[NUMCOLS];
	double   *ub = new double[NUMCOLS];
	char	 *type = new char[NUMCOLS];
	char     **colname = new char*[NUMCOLS];
	int      *rmatbeg;
	int      *rmatind;
	double   *rmatval;
	double   *rhs;
	char     *sense;

	status = CPXchgobjsen(env, lp, CPX_MIN);  /* Problem is minimization */
	if (status)  goto TERMINATE;


	/* filling obj */
	/* obj for X */
	for (int i = 0; i < I; i++)
	for (int j = 0; j < J; j++)
	{
		obj[i*J + j] = 0;
		std::ostringstream s;
		s << "X." << i << "." << j;
		colname[i*J + j] = new char[s.str().length()];
		strcpy(colname[i*J + j], s.str().c_str());
	}

	//obj for Y
	for (int j = 0; j < J2; j++)
	{
		for (int k = 0; k < K; k++)
		{
			obj[Y_beg + j*K + k] = N[j][k];
			std::ostringstream s;
			s << "Y." << j + J1 << "." << k;
			colname[Y_beg + j*K + k] = new char[s.str().length()];
			strcpy(colname[Y_beg + j*K + k], s.str().c_str());
		}
	}


	//obj for B
	for (int j = 0; j < J2; j++)
	{
		obj[B_beg + j] = BS_cost;
		std::ostringstream s;
		s << "B." << j + J1;
		colname[B_beg + j] = new char[s.str().length()];
		strcpy(colname[B_beg + j], s.str().c_str());
	}


	//obj for M
	for (int k = 0; k < K2; k++)
	{
		obj[M_beg + k] = SC_cost;
		std::ostringstream s;
		s << "M." << k + K1;
		colname[M_beg + k] = new char[s.str().length()];
		strcpy(colname[M_beg + k], s.str().c_str());
	}


	//obj for Z
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J2; j++)
		{
			for (int k = 0; k < K; k++)
			{
				obj[Z_beg + i*J2*K + j*K + k] = 0;
				std::ostringstream s;
				s << "Z." << i << "." << j + J1 << "." << k;
				colname[Z_beg + i*J2*K + j*K + k] = new char[s.str().length()];
				strcpy(colname[Z_beg + i*J2*K + j*K + k], s.str().c_str());
			}
		}
	}
	memset(lb, 0, NUMCOLS*sizeof(double));
	for (int i = 0; i < NUMCOLS; i++)
		ub[i] = 1;
	for (int i = 0; i < NUMCOLS; i++)
		type[i] = CPX_BINARY;

	status = CPXnewcols(env, lp, NUMCOLS, obj, lb, ub, type, colname);
	if (status)  goto TERMINATE;

	/* Now add the constraints.  */
	char **rownames;
	// (1)   Dij * Xij <= BS_rad
	rmatind = new int[1];
	rmatbeg = new int[1];
	rmatval = new double[1];
	rhs = new double[1];
	sense = new char[1];
	rownames = new char*[1];
	rmatbeg[0] = 0;
	rhs[0] = BS_rad;
	sense[0] = 'L';
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
		{
			rmatval[0] = D[i][j];
			rmatind[0] = i*J + j;

			rownames[0] = new char[2];
			strcpy(rownames[0], "C1");

			CPXaddrows(env, lp, 0, 1, 1, rhs, sense, rmatbeg, rmatind, rmatval, NULL, rownames);
		}
	}
	free_and_null(rmatind, rmatval);

	// (2)   Xij - Bj <= 0
	rmatind = new int[2];
	rmatval = new double[2];

	rhs[0] = 0;
	sense[0] = 'L';
	rmatval[0] = 1; rmatval[1] = -1;
	for (int i = 0; i < I; i++)
	{
		for (int j = J1; j < J; j++)
		{
			rmatind[0] = i*J + j; rmatind[1] = B_beg + j - J1;
			CPXaddrows(env, lp, 0, 1, 2, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
		}
	}

	// (3)   Yjk - Mk <= 0
	for (int j = 0; j < J2; j++)
	{
		for (int k = 0; k < K2; k++)
		{
			rmatval[0] = 1;		  rmatval[1] = -1;
			rmatind[0] = Y_beg + j*K + K1 + k; rmatind[1] = M_beg + k;

			CPXaddrows(env, lp, 0, 1, 2, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
		}
	}

	// (4)   S(Xij) = 1
	rhs[0] = 1;
	sense[0] = 'E';
	rmatind = new int[J];
	rmatval = new double[J];

	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
		{
			rmatval[j] = 1;
			rmatind[j] = i*J + j;
		}

		CPXaddrows(env, lp, 0, 1, J, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
	}

	free_and_null(rmatind, rmatval);

	// (5)   S(Yjk) - Bj = 0;
	rhs[0] = 0;
	rmatind = new int[K + 1];
	rmatval = new double[K + 1];

	for (int j = 0; j < J2; j++)
	{
		for (int k = 0; k < K; k++)
		{
			rmatval[k] = 1;
			rmatind[k] = Y_beg + j*K + k;
		}
		rmatval[K] = -1;
		rmatind[K] = B_beg + j;

		CPXaddrows(env, lp, 0, 1, K + 1, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
	}

	free_and_null(rmatind, rmatval);

	// (6)   S(Xij) <= BS_cap
	rhs[0] = BS_cap;
	sense[0] = 'L';
	rmatval = new double[I];
	rmatind = new int[I];
	for (int j = 0; j < J; j++)
	{
		for (int i = 0; i < I; i++)
		{

			rmatval[i] = 1;
			rmatind[i] = i*J + j;
		}

		CPXaddrows(env, lp, 0, 1, I, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
	}
	free_and_null(rmatind, rmatval);

	// (7)   SS(Zijk) + SS(XijYjk) <= Jk
	rhs[0] = SC_cap;
	rmatval = new double[J2*I + J1*I];
	rmatind = new int[J2*I + J1*I];
	int t;
	for (int k = 0; k < K; k++)
	{
		t = 0;
		for (int j = 0; j < J2; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[t] = Z_beg + i*J2*K + j*K + k;
				rmatval[t] = 1;
				t++;
			}
		}

		for (int j = 0; j < J1; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[t] = i*J + j;
				rmatval[t] = (k < K1 ? Yjk[j][k] : 0);
				t++;
			}
		}

		CPXaddrows(env, lp, 0, 1, J2*I + J1*I, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
	}

	free_and_null(rmatind, rmatval);



	// (8)   -Zijk <= 0
	rhs[0] = 0;
	rmatval = new double[1];
	rmatind = new int[1];
	rmatval[0] = -1;
	for (int k = 0; k < K; k++)
	{
		for (int j = 0; j < J2; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[0] = Z_beg + i*J2*K + j*K + k;

				CPXaddrows(env, lp, 0, 1, 1, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
			}
		}

	}
	free_and_null(rmatind, rmatval);

	// (9)   Zijk - Xij <= 0
	rmatval = new double[2];
	rmatind = new int[2];
	rmatval[0] = 1; rmatval[1] = -1;
	for (int k = 0; k < K; k++)
	{
		for (int j = 0; j < J2; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[0] = Z_beg + i*J2*K + j*K + k;
				rmatind[1] = i*J + j + J1;
				CPXaddrows(env, lp, 0, 1, 2, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
			}
		}
	}

	// (10)  Zijk - Yjk <= 0
	for (int k = 0; k < K; k++)
	{
		for (int j = 0; j < J2; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[0] = Z_beg + i*J2*K + j*K + k;
				rmatind[1] = Y_beg + j*K + k;
				CPXaddrows(env, lp, 0, 1, 2, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
			}
		}
	}
	free_and_null(rmatind, rmatval);

	// (11)   - Zijk + Yjk + Xij <= 1
	rhs[0] = 1;
	rmatval = new double[3];
	rmatind = new int[3];
	rmatval[0] = -1; rmatval[1] = 1; rmatval[2] = 1;
	for (int k = 0; k < K; k++)
	{
		for (int j = 0; j < J2; j++)
		{
			for (int i = 0; i < I; i++)
			{
				rmatind[0] = Z_beg + i*J2*K + j*K + k;
				rmatind[1] = Y_beg + j*K + k;
				rmatind[2] = i*J + j + J1;
				CPXaddrows(env, lp, 0, 1, 3, rhs, sense, rmatbeg, rmatind, rmatval, NULL, NULL);
			}
		}
	}
	free_and_null(rmatind, rmatval);


	////////////////////////////////////////////////////////////////////////////////////////




TERMINATE:

	return (status);

}  /* END populatebyrow */

