/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId): point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
	Node* root;

	KdTree(): root(NULL) { }

	void insert(std::vector<float> point, int id) {

		std::cout << "Received (" << point[0] << ", " << point[1] << ")" << std::endl;

		Node** currentNode = &root;

		while(*currentNode != NULL) {
			std::cout << "Comparing on X" << std::endl;
			std::cout << "Is " << point[0] << " < " << (*currentNode)->point[0] << "?" <<std::endl;

			if (point[0] < (*currentNode)->point[0]) {
				std::cout << "Going left" << std::endl;
				*currentNode = (*currentNode)->left;
			} else {
				std::cout << "Going right" << std::endl;
				*currentNode = (*currentNode)->right;
			}

			if  (*currentNode == NULL) {
				break;
			} else {
				std::cout << "Comparing on Y" << std::endl;
				std::cout << "Is " << point[1] << " < " << (*currentNode)->point[1] << "?" <<std::endl;

				if (point[1] < (*currentNode)->point[1]) {
					std::cout << "Going left" << std::endl;
					*currentNode = (*currentNode)->left;
				} else {
					std::cout << "Going right" << std::endl;
					*currentNode = (*currentNode)->right;
				}
			}
		}
		*currentNode = new Node(point, id);
		std::cout << "Final node has id " << (*currentNode)->id << std::endl << std::endl;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;
		return ids;
	}
};




