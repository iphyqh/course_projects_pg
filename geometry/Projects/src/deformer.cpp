#include "deformer.h"
#include <iostream>

Deformer::Deformer() : mMesh(nullptr),
                       mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}


void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for 
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/
	typedef Eigen::Triplet<double> T;
	/*====== Programming Assignment 2 ======*/

	std::vector<T> tripletList;
	int size = mMesh->vertices().size();
	Eigen::SparseMatrix< double > systemMat(size+mRoiList.size(), size);
	for (int i = 0; i < size; i++) {
		tripletList.push_back(T(i, i, 1)); // The weightings that need no updates for each iteration
	}

	// Computation of Laplacian
	for (auto it = begin(mMesh->vertices()); it != end(mMesh->vertices()); ++it) {
		int currIndex = (*it)->index();
		if ((*it)->isBoundary())
		{
			// newPositions.push_back((*it)->position());
			continue;
		}
		else {
			double sumOfWeights = 0.0;
			std::vector<double> weights;
			Eigen::Vector3f deltaPos(0, 0, 0);
			Eigen::Vector3f oldPos = (*it)->position();

			HEdge* currHEdge = (*it)->halfEdge();
			HEdge* HEPrev = (*it)->halfEdge();
			HEdge* HEPost = (*it)->halfEdge();
			Vertex* InitialAdjacent = currHEdge->end();

			do {
				HEPrev = currHEdge->twin()->next();
				HEPost = currHEdge->next();

				Eigen::Vector3f adjPos = currHEdge->end()->position();
				Eigen::Vector3f prevPos = HEPrev->end()->position();
				Eigen::Vector3f postPos = HEPost->end()->position();

				Eigen::Vector3f a0 = oldPos - prevPos;
				Eigen::Vector3f a1 = adjPos - prevPos;
				Eigen::Vector3f b0 = oldPos - postPos;
				Eigen::Vector3f b1 = adjPos - postPos;

				double cota = ((a0.dot(a1)) / ((a0.cross(a1)).norm()));
				double cotb = ((b0.dot(b1)) / ((b0.cross(b1)).norm()));
				double weight = (cota + cotb) / 2;
				weights.push_back(weight);
				sumOfWeights += weight;

				currHEdge = currHEdge->twin()->next();
			} while (currHEdge->end() != InitialAdjacent);

			currHEdge = (*it)->halfEdge();
			int i = 0;
			do {
				int index = currHEdge->end()->index();
				tripletList.push_back(T(currIndex, index, -weights[i] / sumOfWeights)); // minus added specially
				currHEdge = currHEdge->twin()->next();
				i++;
			} while (currHEdge->end() != InitialAdjacent);
		}

	}
	// The rows for handles in vector b
	int index = 0;
	for (auto it = begin(mRoiList); it != end(mRoiList); it++) {
		tripletList.push_back(T(mMesh->vertices().size()+index, (*it)->index(), 1));
		index++;
	}

	systemMat.setFromTriplets(tripletList.begin(), tripletList.end());
	
	A = systemMat;
	AT = systemMat.transpose();
	systemMat = AT * A;

	// Do factorization
	if (systemMat.nonZeros() > 0) {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		} else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
	numConstraints = mRoiList.size();
	size = mMesh->vertices().size() + numConstraints;
	X = new Eigen::VectorXd(size); //Vertex original position X
	Y = new Eigen::VectorXd(size); //Vertex original position Y
	Z = new Eigen::VectorXd(size); //Vertex original position Z
	//std::vector<Eigen::VectorXd> b;
	b.push_back(X);
	b.push_back(Y);
	b.push_back(Z);

	size = mMesh->vertices().size();
	Xnew = new Eigen::VectorXd(size); //Vertex new position X
	Ynew = new Eigen::VectorXd(size); //Vertex new position Y
	Znew = new Eigen::VectorXd(size); //Vertex new position Z
	//std::vector<Eigen::VectorXd> positions;
	positions.push_back(Xnew);
	positions.push_back(Ynew);
	positions.push_back(Znew);
	for (auto it = begin(mMesh->vertices()); it != end(mMesh->vertices()); ++it) {
		int currIndex = (*it)->index();
		for (int i = 0; i < 3; i++) {
			(*(b[i]))[currIndex] = (*it)->position()[i];
			(*(positions[i]))[currIndex] = (*it)->position()[i];
		}
	}


	// --------------------- Attempt to fix the bug here ----------------------
	// Computing delta
	for (int i = 0; i < 3; i++)
		*b[i] = A * (*(positions[i]));
	// --------------------- Attempt to fix the bug here ----------------------

}

void Deformer::deform() {
	if (mCholeskySolver == nullptr) {
		return;
	}

	std::cout << "deform!" << std::endl;

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques 
	/* take place.
	/* Solve for the new vertex positions after the 
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/
	numConstraints = mRoiList.size();
	int size = mMesh->vertices().size();
	//Eigen::VectorXd X(size); //Vertex original position X
	//Eigen::VectorXd Y(size); //Vertex original position Y
	//Eigen::VectorXd Z(size); //Vertex original position Z

	//b.push_back(X);
	//b.push_back(Y);
	//b.push_back(Z);

	//size = mMesh->vertices().size();
	//Eigen::VectorXd Xnew(size); //Vertex new position X
	//Eigen::VectorXd Ynew(size); //Vertex new position Y
	//Eigen::VectorXd Znew(size); //Vertex new position Z

	//positions.push_back(Xnew);
	//positions.push_back(Ynew);
	//positions.push_back(Znew);
	//for (auto it = begin(mMesh->vertices()); it != end(mMesh->vertices()); ++it) {
	//	int currIndex = (*it)->index();
	//	for (int i = 0; i < 3; i++) {
	//		(b[i])[currIndex] = (*it)->position()[i];
	//		(positions[i])[currIndex] = (*it)->position()[i];
	//	}
	//}


	//// --------------------- Potential bug here! Delta computation may need to be done earlier then deform()!! ----------------------
	//// Computing delta
	//for (int i = 0; i < 3; i++)
	//	b[i] = A * positions[i];
	//// --------------------- Potential bug here! Delta computation may need to be done earlier then deform()!! ----------------------



	int index = 0;
	// The rows for handles in vector b
	for (auto it = begin(mRoiList); it != end(mRoiList); it++) {
		for (int i = 0; i < 3; i++)
			(*b[i])[size+index] = mRoiList[index]->position()[i];
		index++;
	}

	std::cout << "Out~! " << std::endl;
	
	for (int i = 0; i < 3; i++) {
		*positions[i] = mCholeskySolver->solve(AT * (*b[i]));
	}

	// Just the same old way of traversing the vertices like in mesh.cpp
	for (auto it = begin(mMesh->vertices()); it != end(mMesh->vertices()); ++it) {
		int currIndex = (*it)->index();
		(*it)->setPosition(Eigen::Vector3f( (*positions[0])[currIndex], (*positions[1])[currIndex], (*positions[2])[currIndex]));
	}

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html


	/*====== Programming Assignment 2 ======*/
}
