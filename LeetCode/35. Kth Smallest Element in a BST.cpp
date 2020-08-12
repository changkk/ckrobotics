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
 */

/*
    DFS
    Go far left which is the smallest, and once get to the end, check if there is right
    If there is right, go to the left end. Do this until there is not left and right.
    If there is no both left and right, pop it. 
    
    example1
    stack
    3 -> 1


*/
class Solution {
public:
    int kthSmallest(TreeNode* root, int k) {
        int count = 0;

        while(root)
        {
            if(root->left)
            {
                TreeNode* pre = root->left;
                while(pre->right && pre->right != root)
                    pre = pre->right;
                if(!pre->right)
                {
                    pre->right = root;
                    root = root->left;            
                }
                else
                {
                    pre->right = NULL;
                    count++;
                    if(count == k) 
                    {
                        return root->val;
                    }
                    root = root->right;
                }
                    
            }
            else
            {
                count++;
                if(count == k) 
                {
                    return root->val;
                }
                root = root->right;
            }
        }
        
        return 0;
        
    }
};