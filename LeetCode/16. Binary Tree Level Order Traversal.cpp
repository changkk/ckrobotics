/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
class Solution {
public:
    vector<vector<int>> levelOrder(TreeNode* root) {
        vector<vector<int>> ans;
        
        queue<TreeNode*> q;
        TreeNode* cur = root;
        
        if(!cur) return ans;
        
        q.push(cur);
        
        while(!q.empty())
        {
            int qSize = q.size();
            vector<int> tmp;
            for(int i = 0; i < qSize; i++)
            {        
                cur = q.front();
                if(cur->left)
                    q.push(cur->left);
                if(cur->right)
                    q.push(cur->right);
                
                q.pop();
                tmp.push_back(cur->val);
            }
            ans.push_back(tmp);
        }
        

        return ans;
    }
};