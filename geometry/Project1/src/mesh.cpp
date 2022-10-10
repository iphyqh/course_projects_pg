#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Sparse>
#include <set>
#include <chrono>

#define EPSILON 0.0001

HEdge::HEdge(bool b) {
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mFlag = false;
	mValid = true;
}

HEdge* HEdge::twin() const {
	return mTwin;
}

HEdge* HEdge::setTwin(HEdge* e) {
	mTwin = e;
	return mTwin;
}

HEdge* HEdge::prev() const {
	return mPrev;
}

HEdge* HEdge::setPrev(HEdge* e) {
	mPrev = e;
	return mPrev;
}

HEdge* HEdge::next() const {
	return mNext;
}

HEdge* HEdge::setNext(HEdge* e) {
	mNext = e;
	return mNext;
}

Vertex* HEdge::start() const {
	return mStart;
}

Vertex* HEdge::setStart(Vertex* v) {
	mStart = v;
	return mStart;
}

Vertex* HEdge::end() const {
	return mNext->start();
}

Face* HEdge::leftFace() const {
	return mFace;
}

Face* HEdge::setFace(Face* f) {
	mFace = f;
	return mFace;
}

bool HEdge::flag() const {
	return mFlag;
}

bool HEdge::setFlag(bool b) {
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const {
	return mBoundary;
}

bool HEdge::isValid() const {
	return mValid;
}

bool HEdge::setValid(bool b) {
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == nullptr) {
		mStart = nullptr;
		mNext = nullptr;
	} else {
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}

HEdge* OneRingHEdge::nextHEdge() {
	HEdge* ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart) {
		mNext = mNext->prev()->twin();
	} else {
		mNext = nullptr;
	}
	return ret;
}

OneRingVertex::OneRingVertex(const Vertex* v): ring(v) {
}

Vertex* OneRingVertex::nextVertex() {
	HEdge* he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f& v): mPosition(v), mHEdge(nullptr), mFlag(0) {
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z): mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}


const Eigen::Vector3f& Vertex::position() const {
	return mPosition;
}

const Eigen::Vector3f& Vertex::setPosition(const Eigen::Vector3f& p) {
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f& Vertex::normal() const {
	return mNormal;
}

const Eigen::Vector3f& Vertex::setNormal(const Eigen::Vector3f& n) {
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f& Vertex::color() const {
	return mColor;
}

const Eigen::Vector3f& Vertex::setColor(const Eigen::Vector3f& c) {
	mColor = c;
	return mColor;
}

HEdge* Vertex::halfEdge() const {
	return mHEdge;
}

HEdge* Vertex::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

int Vertex::index() const {
	return mIndex;
}

int Vertex::setIndex(int i) {
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const {
	return mFlag;
}

int Vertex::setFlag(int f) {
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const {
	return mValid;
}

bool Vertex::setValid(bool b) {
	mValid = b;
	return mValid;
}

bool Vertex::isBoundary() const {
	OneRingHEdge ring(this);
	HEdge* curr = nullptr;
	while (curr = ring.nextHEdge()) {
		if (curr->isBoundary()) {
			return true;
		}
	}
	return false;
}

int Vertex::valence() const {
	int count = 0;
	OneRingVertex ring(this);
	Vertex* curr = nullptr;
	while (curr = ring.nextVertex()) {
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mValid(true) {
}

HEdge* Face::halfEdge() const {
	return mHEdge;
}

HEdge* Face::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

bool Face::isBoundary() const {
	HEdge* curr = mHEdge;
	do {
		if (curr->twin()->isBoundary()) {
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const {
	return mValid;
}

bool Face::setValid(bool b) {
	mValid = b;
	return mValid;
}

Mesh::Mesh() {
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
}

Mesh::~Mesh() {
	clear();
}

const std::vector< HEdge* >& Mesh::edges() const {
	return mHEdgeList;
}

const std::vector< HEdge* >& Mesh::boundaryEdges() const {
	return mBHEdgeList;
}

const std::vector< Vertex* >& Mesh::vertices() const {
	return mVertexList;
}

const std::vector< Face* >& Mesh::faces() const {
	return mFaceList;
}


bool Mesh::isVertexPosDirty() const {
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b) {
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const {
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b) {
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const {
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b) {
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename) {
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	if (iglFlag) {
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list
		for (int vidx = 0; vidx < numVertices; ++vidx) {
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
			                                 mVertexMat(vidx, 1),
			                                 mVertexMat(vidx, 2)));
		}
		// Fill in the face list
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
		}

		std::vector< HEdge* > hedgeList;
		for (int i = 0; i < mBHEdgeList.size(); ++i) {
			if (mBHEdgeList[i]->start()) {
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i) {
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	} else {
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge* e1, HEdge* e2) {
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge* e1, HEdge* e2) {
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face* f, HEdge* e) {
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3) {
	Face* face = new Face();

	HEdge* hedge[3];
	HEdge* bhedge[3]; // Boundary half-edges
	Vertex* vert[3];

	for (int i = 0; i < 3; ++i) {
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i) {
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i) {
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i) {
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) {
		Vertex* start = bhedge[i]->start();
		Vertex* end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start) {
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr); // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i) {
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

Eigen::Vector3f Mesh::initBboxMin() const {
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const {
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags() {
	// Init to 255
	for (Vertex* vert : mVertexList) {
		if (vert->flag() != 0) {
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector< Vertex* > tmpList;
	for (Vertex* vert : mVertexList) {
		if (vert->flag() == 255) {
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty()) {
				Vertex* v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex* v2 = orv.nextVertex()) {
					if (v2->flag() == 255) {
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

void Mesh::clear() {
	for (int i = 0; i < mHEdgeList.size(); ++i) {
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i) {
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i) {
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector< int > Mesh::collectMeshStats() {
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Collect mesh information as listed above.
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	std::vector< int > stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

int Mesh::countBoundaryLoops() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	return count;
}

int Mesh::countConnectedComponents() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/


	/*====== Programming Assignment 0 ======*/

	return count;
}




void Mesh::computeVertexNormals() {
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a 
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/

	/*====== Programming Assignment 0 ======*/
	for (auto it = begin(vertices()); it != end(vertices()); ++it) {
		Eigen::Vector3f startPos = (*it)->position();
		OneRingVertex ring(*it);
		Vertex * curr = nullptr;
		Eigen::Vector3f distance = (ring.nextVertex()->position() - startPos);
		Eigen::Vector3f currNormal = Eigen::Vector3f::Zero();
		// Tried to eliminate identical normals for one vertex but it does not seem to make sense.
		// auto cmp = [](Eigen::Vector3f lhs, Eigen::Vector3f rhs) { return (lhs.normalized() - rhs.normalized() ).norm() < 0.001; };
		std::vector<Eigen::Vector3f> s;

		while (curr = ring.nextVertex()) {
			s.push_back(distance.cross(curr->position() - startPos));
			distance = (curr->position() - startPos);
		}
		for (Eigen::Vector3f normal : s)
			(*it)->setNormal((*it)->normal() + normal);
		(*it)->setNormal((*it)->normal()).normalized();
 	}
	
	// Notify mesh shaders
	setVertexNormalDirty(true);
}

// Explicit Smoothing
void Mesh::umbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/
	// If Taubin Smoothing is used, combination lambda = 0.33 and mu = 0.34 is advised. 
	// It is usually necessary to make sure they do not differ significantly
	float lambda = 0.5;
	float mu = 0; 
	int indicator = 1; // indicates whether Taubin Smoothing is enabled, 1 for disabled; 2 for enabled.
	int iteration = 5;
	auto start = std::chrono::high_resolution_clock::now();
	if (cotangentWeights) {


		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 1: Implement the cotangent weighting
		/* scheme for explicit mesh smoothing.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/
		
		// Ready to go for the implicit part now that I can play with matrices and vectors~ (even if we do not have to at this stage)
		const int size = vertices().size();
		typedef Eigen::Triplet<double> T;
		typedef Eigen::SparseMatrix<double> SpMat;


		SpMat Laplacian(size, size);
		
		Eigen::VectorXd X(size); //Vertex Position X
		Eigen::VectorXd Y(size); //Vertex Position Y
		Eigen::VectorXd Z(size); //Vertex Position Z
		std::vector<Eigen::VectorXd> positions;
		
		positions.push_back(X);
		positions.push_back(Y);
		positions.push_back(Z);
		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
			int currIndex = (*it)->index();
			for (int i = 0; i < 3; i++)
				(positions[i])[currIndex] = (*it)->position()[i];
		}
		for (int t = 0; t < iteration; t++) {		
			std::vector<T> tripletList;
		
			for (int i = 0; i < size; i++) {
				tripletList.push_back(T(i, i, -1)); // The weightings that need no updates for each iteration
			}

			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
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
						tripletList.push_back(T(currIndex, index, weights[i]/sumOfWeights));
						currHEdge = currHEdge->twin()->next();
						i++;
					} while (currHEdge->end() != InitialAdjacent);
				}
				
			}
			
			Laplacian.setFromTriplets(tripletList.begin(), tripletList.end());
			for (int i = 0; i < 3; i++)
				positions[i] = positions[i] + lambda * Laplacian * positions[i];
			int i = 0;
			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				(*it)->setPosition(Eigen::Vector3f(positions[0][currIndex], positions[1][currIndex], positions[2][currIndex]));
				i++;
			}

		}


	


	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the uniform weighting 
		/* scheme for explicit mesh smoothing.
		/**********************************************/
		// Ready to go for the implicit part now that I can play with matrices and vectors~ (even if we do not have to at this stage)
		const int size = vertices().size();
		typedef Eigen::Triplet<double> T;
		typedef Eigen::SparseMatrix<double> SpMat;


		SpMat Laplacian(size, size);

		Eigen::VectorXd X(size); //Vertex Position X
		Eigen::VectorXd Y(size); //Vertex Position Y
		Eigen::VectorXd Z(size); //Vertex Position Z
		std::vector<Eigen::VectorXd> positions;

		positions.push_back(X);
		positions.push_back(Y);
		positions.push_back(Z);
		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
			int currIndex = (*it)->index();
			for (int i = 0; i < 3; i++)
				(positions[i])[currIndex] = (*it)->position()[i];
		}
		for (int t = 0; t < iteration; t++) {
			std::vector<T> tripletList;

			for (int i = 0; i < size; i++) {
				tripletList.push_back(T(i, i, -1)); // The weightings that need no updates for each iteration
			}

			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
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
						//Eigen::Vector3f prevPos = HEPrev->end()->position();
						//Eigen::Vector3f postPos = HEPost->end()->position();

						//Eigen::Vector3f a0 = oldPos - prevPos;
						//Eigen::Vector3f a1 = adjPos - prevPos;
						//Eigen::Vector3f b0 = oldPos - postPos;
						//Eigen::Vector3f b1 = adjPos - postPos;

						//double cota = ((a0.dot(a1)) / ((a0.cross(a1)).norm()));
						//double cotb = ((b0.dot(b1)) / ((b0.cross(b1)).norm()));
						//double weight = (cota + cotb) / 2;
						weights.push_back(1);
						sumOfWeights += 1;

						currHEdge = currHEdge->twin()->next();
					} while (currHEdge->end() != InitialAdjacent);

					currHEdge = (*it)->halfEdge();
					int i = 0;
					do {
						int index = currHEdge->end()->index();
						tripletList.push_back(T(currIndex, index, weights[i] / sumOfWeights));
						currHEdge = currHEdge->twin()->next();
						i++;
					} while (currHEdge->end() != InitialAdjacent);
				}

			}

			Laplacian.setFromTriplets(tripletList.begin(), tripletList.end());
			for (int i = 0; i < 3; i++)
				positions[i] = positions[i] + lambda * Laplacian * positions[i];
			int i = 0;
			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				(*it)->setPosition(Eigen::Vector3f(positions[0][currIndex], positions[1][currIndex], positions[2][currIndex]));
				i++;
			}

		}
		//// My first attempt with smoothing, and to keep everything simple, and it is actually much more efficient
		//// I did not use any matrices and vectors on the update step
		//for (int t = 0; t < iteration; t++) {
		//	
		//	std::vector<Eigen::Vector3f> newPositions;
		//	if ((t % indicator) == 0) {
		//		
		//	}
		//	else {
		//		std::cout << "taubin!" << std::endl;
		//		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
		//			if ((*it)->isBoundary())
		//			{
		//				newPositions.push_back((*it)->position());
		//				continue;
		//			}
		//			else {
		//				Eigen::Vector3f oldPos = (*it)->position();
		//				OneRingVertex ring(*it);
		//				Vertex* curr = nullptr;
		//				Eigen::Vector3f accumulator = (ring.nextVertex()->position());
		//				int k = 1;
		//				while (curr = ring.nextVertex()) {
		//					accumulator += (curr->position());
		//					k++;
		//				}
		//				accumulator = accumulator / k;
		//				Eigen::Vector3f deltaPos = accumulator - oldPos;
		//				newPositions.push_back(oldPos - mu * deltaPos);
		//				// (*it)->setPosition(oldPos + lambda * deltaPos); // One Very Possible Big Flaw Here: Update one vertex for 15 iterations without updating the rest? Sounds weird!!
		//			}
		//		}
		//		int i = 0;
		//		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
		//			(*it)->setPosition(newPositions[i]);
		//			i++;
		//		}
		//	}
		//}
	}
	// Get ending timepoint
	auto stop = std::chrono::high_resolution_clock::now();

	// Get duration. Substart timepoints to
	// get duration. To cast it to proper unit
	// use duration cast method
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	std::cout << "Time taken by function: "
		<< duration.count() << " microseconds" << std::endl;
	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}

// Implicit Smoothing
void Mesh::implicitUmbrellaSmooth(bool cotangentWeights) {
	auto start = std::chrono::high_resolution_clock::now();
	/*====== Programming Assignment 1 ======*/
	float lambda = 1;
	float mu = 0;
	int indicator = 1; // indicates whether Taubin Smoothing is enabled, 1 for disabled; 2 for enabled.
	int iteration = 1;
	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	// I will go with the BiCGSTAB offered by Eigen
	//auto fnConjugateGradient = [](const Eigen::SparseMatrix< float >& A,
	//                              const Eigen::VectorXf& b,
	//                              int maxIterations,
	//                              float errorTolerance,
	//                              Eigen::VectorXf& x)
	//{
	//	/**********************************************/
	//	/*          Insert your code here.            */
	//	/**********************************************/
	//	/*
	//	/* Params:
	//	/*  A: 
	//	/*  b: 
	//	/*  maxIterations:	Max number of iterations
	//	/*  errorTolerance: Error tolerance for the early stopping condition
	//	/*  x:				Stores the final solution, but should be initialized. 
	//	/**********************************************/
	//	/*
	//	/* Step 1: Implement the biconjugate gradient
	//	/* method.
	//	/* Hint: https://en.wikipedia.org/wiki/Biconjugate_gradient_method
	//	/**********************************************/

	const int size = vertices().size();
	typedef Eigen::Triplet<double> T;
	typedef Eigen::SparseMatrix<double> SpMat;


	SpMat Laplacian(size, size);
	Eigen::VectorXd X(size); //Vertex Position X
	Eigen::VectorXd Y(size); //Vertex Position Y
	Eigen::VectorXd Z(size); //Vertex Position Z
	std::vector<Eigen::VectorXd> positions;

	positions.push_back(X);
	positions.push_back(Y);
	positions.push_back(Z);
	//};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the cotangent weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/
		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
			int currIndex = (*it)->index();
			for (int i = 0; i < 3; i++)
				(positions[i])[currIndex] = (*it)->position()[i];
		}
		for (int t = 0; t < iteration; t++) {
			std::vector<T> tripletList;

			for (int i = 0; i < size; i++) {
				tripletList.push_back(T(i, i, 1+lambda)); // The weightings that need no updates for each iteration
			}

			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				if ((*it)->isBoundary())
				{
					tripletList.push_back(T((*it)->index(), (*it)->index(), 1));
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
						tripletList.push_back(T(currIndex, index, -weights[i] / sumOfWeights));
						currHEdge = currHEdge->twin()->next();
						i++;
					} while (currHEdge->end() != InitialAdjacent);
				}

			}

			Laplacian.setFromTriplets(tripletList.begin(), tripletList.end());
			Eigen::BiCGSTAB<SpMat> solver;
			solver.compute(Laplacian); // Actually, the Laplacian here stands for I-lambda*Laplacian
			for (int i = 0; i < 3; i++)
				positions[i] = solver.solve(positions[i]);
			std::cout << "#iterations:     " << solver.iterations() << std::endl;
			std::cout << "estimated error: " << solver.error() << std::endl;
			int i = 0;
			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				(*it)->setPosition(Eigen::Vector3f(positions[0][currIndex], positions[1][currIndex], positions[2][currIndex]));
				i++;
			}

		}

	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 3: Implement the uniform weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/

		for (auto it = begin(vertices()); it != end(vertices()); ++it) {
			int currIndex = (*it)->index();
			for (int i = 0; i < 3; i++)
				(positions[i])[currIndex] = (*it)->position()[i];
		}
		for (int t = 0; t < iteration; t++) {
			std::vector<T> tripletList;

			for (int i = 0; i < size; i++) {
				tripletList.push_back(T(i, i, 1 + lambda)); // The weightings that need no updates for each iteration
			}

			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				if ((*it)->isBoundary())
				{
					tripletList.push_back(T((*it)->index(), (*it)->index(), 1));
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
						weights.push_back(1);
						sumOfWeights += 1;

						currHEdge = currHEdge->twin()->next();
					} while (currHEdge->end() != InitialAdjacent);

					currHEdge = (*it)->halfEdge();
					int i = 0;
					do {
						int index = currHEdge->end()->index();
						tripletList.push_back(T(currIndex, index, -weights[i] / sumOfWeights));
						currHEdge = currHEdge->twin()->next();
						i++;
					} while (currHEdge->end() != InitialAdjacent);
				}

			}

			Laplacian.setFromTriplets(tripletList.begin(), tripletList.end());
			Eigen::BiCGSTAB<SpMat> solver;
			solver.compute(Laplacian); // Actually, the Laplacian here stands for I-lambda*Laplacian
			for (int i = 0; i < 3; i++)
				positions[i] = solver.solve(positions[i]);
			std::cout << "#iterations:     " << solver.iterations() << std::endl;
			std::cout << "estimated error: " << solver.error() << std::endl;
			int i = 0;
			for (auto it = begin(vertices()); it != end(vertices()); ++it) {
				int currIndex = (*it)->index();
				(*it)->setPosition(Eigen::Vector3f(positions[0][currIndex], positions[1][currIndex], positions[2][currIndex]));
				i++;
			}

		}

	}
	// Get ending timepoint
	auto stop = std::chrono::high_resolution_clock::now();

	// Get duration. Substart timepoints to
	// get duration. To cast it to proper unit
	// use duration cast method
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	std::cout << "Time taken by function: "
		<< duration.count() << " microseconds" << std::endl;
	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}
