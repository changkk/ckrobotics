/*
    1. Backtracking
        []
        [1 2 3]
        [21 31 12 32 31 32]
        [321 321  312 312 213 123]
        
        O(n^n) time
        
    2. DP
        1 + DP(2, 3) = 1 + [2 3] [3 2]
        2 + DP(1, 3) =
        
        [1 2 3 4]
        1 + DP(2 3 4) => 1 + 2 + [3 4], [4,3]
        
        O(n^2) time O(n) space

*/

class Solution {
public:

    vector<vector<int>> permute(vector<int>& nums) {            
        vector<vector<int>> res;
        if(nums.size() < 2)
        {
            res.push_back(nums);
            return res;
        }
        
        if(nums.size()==2) 
        {
            res.push_back(nums);
            swap(nums[0],nums[1]);
            res.push_back(nums);
            return res;
        }
        
        for(int i = 0; i < nums.size(); i++)
        {
            vector<int> tmp(nums.begin(),nums.end());
            tmp.erase(tmp.begin()+i);
            vector<vector<int>> tmp2 = permute(tmp); // [[2 3][3 2]]
        
            for(int j = 0; j < tmp2.size(); j++)
            {
                tmp2[j].push_back(nums[i]); // [1 2 3][2 1 3] [2 3 1] [3 2 1]
                res.push_back(tmp2[j]);                
            }

        }
        
        return res;
    }
};