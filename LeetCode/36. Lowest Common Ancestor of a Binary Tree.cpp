/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */

/*

    Goal: Find lowest common ancester
    Find the first meeting node = common ancester
    ex)
    3 - 5 - 2 - 4
    3 - 5
    DFS for each node, and if found, save the path => O(2n) - O(n)
    And Compare until they have same node, if different, return the previous node.
    => O(n)
    O(n) time O(n) space

    q = 
    
    save all the parent - child node, and start from the target node, go up by saving the path
    and compare
    O(n) time + O(k+i) => O(n)
    O(n) + O(k+i)
    
    unordered_map<TreeNode*> child - parent;
    
    
    path = 3 5


class Solution {
public:
    unordered_map<TreeNode*, TreeNode*> parent;
    void saveParent(TreeNode* node){
        if(node->left)
        {
            parent[node->left] = node;
            saveParent(node->left);
        }
        if(node->right)
        {
            parent[node->right] = node;
            saveParent(node->right);
        }
    }
    
    TreeNode* lowestCommonAncestor(TreeNode* root, TreeNode* &p, TreeNode* &q) {
        if(!root) return root;
        saveParent(root);
        vector<TreeNode*> pathP;
        vector<TreeNode*> pathQ;


        while(p!=root) // mistake. Forgot to push itself
        {
            pathP.push_back(p); // [5]
            p = parent[p]; // 3
        }
        pathP.push_back(p); //[3]

        while(q!=root)
        {
            pathQ.push_back(q); //[4 2 5]
            q = parent[q];
        }
        pathQ.push_back(q); // [4 2 5 3]
        
        
        // [5 3]    [1 3]
        int i = pathP.size()-1;
        int j = pathQ.size()-1; // mistake. Start from n not n-1

        while(pathP[i] == pathQ[j]) // [5 3] [4 2 5 3]
        {
            if(i==0) return pathP[i];
            if(j==0) return pathQ[j];
            i--;
            j--;
        }    
        
        return pathP[i+1];
        
    }
};
*/

class Solution {
public:
    TreeNode* lowestCommonAncestor(TreeNode* root, TreeNode* &p, TreeNode* &q){
        
        // If target node is not found, just make left or right as NULL
        if(!root || root == p || root == q) return root;
        TreeNode* left = lowestCommonAncestor(root->left, p, q);
        TreeNode* right = lowestCommonAncestor(root->right, p, q);
        
        if(!left) return right;
        if(!right) return left;
        return root;
        
    }
};