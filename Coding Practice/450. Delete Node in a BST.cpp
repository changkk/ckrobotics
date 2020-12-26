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
 
 Find the node and connect it 
 
                8
               / \
              4   9
             / \   \
            1   6   10
               / \
              5   7
 
                8
               / \
              6   9
             / \   \
            5   7   10
           /   
          1   
          
          
      Find the node and
      if the node has the right branch, then replace the target by the right branches
      and go to the far left of the right branch and connect it to the original left branch
 
 
 */
class Solution {
public:
    TreeNode* deleteNode(TreeNode* root, int key) {
        TreeNode* preHead = new TreeNode(0);
        TreeNode* pre;
        preHead->right = root;
        pre = preHead;
        if(!root) return root;
        bool right = true;
        
        TreeNode* cur = root;
        while(cur && cur->val != key){
            if(cur->val > key) right = false, pre = cur, cur = cur->left;
            else right = true, pre = cur, cur = cur->right;
        }
        if(!cur) return root;
        
        if(cur->right){
            if(right) pre->right = cur->right;
            else pre->left = cur->right;
            TreeNode* curLeft = cur->left;
            cur = cur->right;
            while(cur->left) cur = cur->left;
            cur->left = curLeft;
        }
        else{
            if(right) pre->right = cur->left;
            else pre->left = cur->left;
        }
        
        
        return preHead->right;
    }
};