#ifndef DEFORMER_H
#define DEFORMER_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "mesh.h"

// Deform mesh using Laplacian coordinates
class Deformer {
public:
	Deformer();
	~Deformer();

	void setMesh(Mesh* mesh);

	/*====== Programming Assignment 2 ======*/
	// This is the place where the editing techniques take place
	void deform();
	/*====== Programming Assignment 2 ======*/

private:
	/*====== Programming Assignment 2 ======*/
	// Build left hand side matrix and pre-factorize it
	void buildSystemMat();
	/*====== Programming Assignment 2 ======*/

	void clear();

	Mesh* mMesh;
	std::vector< Vertex* > mRoiList;
	// Solver for pre-factorizing the system matrix of the deformation
	Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >* mCholeskySolver;

	// A and Transpose of A
	Eigen::SparseMatrix< double > AT;
	Eigen::SparseMatrix< double > A;

	// Number of constraints
	int numConstraints;

	Eigen::VectorXd* X;
	Eigen::VectorXd* Y;
	Eigen::VectorXd* Z;
	Eigen::VectorXd* Xnew;
	Eigen::VectorXd* Ynew;
	Eigen::VectorXd* Znew;
	std::vector<Eigen::VectorXd*> b; // This may be wrong
	std::vector<Eigen::VectorXd*> positions; // This may be wrong
};

#endif // DEFORMER_H
