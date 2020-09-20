#include<iostream>
#include<Eigen/Dense>
#include"kalman.h"
using namespace Eigen;
using namespace std;
int main1() {

	/*

		VectorXd : X-> dynamic shape 
				   d-> double

		MatrixXd : X -> unknown shape (dynamic matric on the heap)
				   d -> The type of variable is double 

		Matrix3d : This is a fixed size matrix on the stack 
				   - It's much faster then the heap.
			

		
		
	*/

	VectorXd v1(3);
	v1 << 1, 2, 3;
	cout << "vetor 1 size " << v1.size() <<  endl;

	VectorXd v2 = VectorXd::Constant(5,10);
	cout << "v2 = " << endl << v2 << endl;




	MatrixXd m1;
	Matrix3d m2;

	cout << "size of m2 is " << m2.size() << endl;

	m2 << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;

	cout << "m2 = " << m2 << endl;

	MatrixXd result,transpose;

	result = m2 * v1;
	cout << "Result = " << endl << result << endl;

	transpose = m2.transpose();
	cout << "Transpose = " << endl << transpose << endl;



	return 0;
}

int main() {

	
}