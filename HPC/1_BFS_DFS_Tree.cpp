#include <iostream>
#include <vector>
#include <queue>
#include <omp.h>

using namespace std;

// Node structure representing a tree node
struct TreeNode {
    int data;
    vector<TreeNode*> children;

    TreeNode(int val) : data(val) {}
};

// Tree class representing the tree structure
class Tree {
    TreeNode* root;

public:
    Tree(int val) {
        root = new TreeNode(val);
    }

    // Add a child to a parent node
    void addChild(TreeNode* parent, int val) {
        TreeNode* newNode = new TreeNode(val);
        parent->children.push_back(newNode);
    }

    // Method to get the root node
    TreeNode* getRoot() {
        return root;
    }

    // Parallel Depth-First Search
    void parallelDFS(TreeNode* node) {
        cout << node->data << " ";

        #pragma omp parallel for
        for (size_t i = 0; i < node->children.size(); ++i) {
            parallelDFS(node->children[i]);
        }
    }

    // Parallel Breadth-First Search
    void parallelBFS() {
        queue<TreeNode*> q;
        q.push(root);

        while (!q.empty()) {
            TreeNode* current = q.front();
            q.pop();
            cout << current->data << " ";

            #pragma omp parallel for
            for (size_t i = 0; i < current->children.size(); ++i) {
                q.push(current->children[i]);
            }
        }
    }
};

int main() {
    // Create a tree
    Tree tree(1);
    TreeNode* root = tree.getRoot();
    tree.addChild(root, 2);
    tree.addChild(root, 3);
    tree.addChild(root, 4);

    TreeNode* node2 = root->children[0];
    tree.addChild(node2, 5);
    tree.addChild(node2, 6);

    TreeNode* node4 = root->children[2];
    tree.addChild(node4, 7);
    tree.addChild(node4, 8);

    /*
               1
             / | \
            2  3  4
           / \    / \
          5   6  7   8
    */

    cout << "Depth-First Search (DFS): ";
    tree.parallelDFS(root);
    cout << endl;

    cout << "Breadth-First Search (BFS): ";
    tree.parallelBFS();
    cout << endl;

    return 0;
}
