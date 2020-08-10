/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode() : val(0), left(nullptr), right(nullptr) {}
 *     TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
 *     TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
 * };
 
 
   15
  3  7
 9 20

     1
   2   3
  4 5 7 6
   8 9

[4 2 8 5 9] 1 [7 3 6]
[4 8 9 5 2] [7 6 3] 1

last element is the root
last second is should be child of the root
first element is the left most element
second element is the parent of the left most

find the root in the inorder
the left one should be right most on left 
the right one should be left most on right

 
 */
class Solution {
public:
    TreeNode* makeTree(vector<int>& inorder, int is, int ie, vector<int>& postorder, int ps, int pe){
        
        if(ps>pe) return nullptr;
        
        int idx;
        for(int i = is; i < ie; i++)
        {
            if(inorder[i] == postorder[pe])
            {
                idx = i;
                break;
            }
        }
            
        TreeNode* node = new TreeNode(postorder[pe]);
        
        node->left = makeTree(inorder,is,idx-1,postorder,ps,ps+idx-is-1);
        node->right = makeTree(inorder,idx+1,ie,postorder,pe-ie+idx, pe -1);
        
        return node;
    }
    
    TreeNode* buildTree(vector<int>& inorder, vector<int>& postorder) {
        
        return makeTree(inorder, 0, inorder.size()-1, postorder, 0, postorder.size()-1);
            
    }
};