class Solution {
public:
    vector<int> topKFrequent(vector<int>& nums, int k) {
        unordered_map<int,int> umap;
        for(auto num:nums)
            umap[num]++; // umap[1] = 3, umap[2] = 2, umap[3] = 1
                
        vector<vector<int>> tmp(nums.size()+1); // tmp = {  ,  ,  }
        for(auto it:umap)
            tmp[it.second].push_back(it.first); // tmp[3] = {1} tmp[2] = {2} tmp[1] = {1}
        
        vector<int> res;
        
        for(int i = tmp.size()-1; i >= 0; i--) // i = 2 k = 1 tmp[3]
        {
            if(tmp[i].empty()) continue;
            
            for(int j = 0; j < tmp[i].size(); j++) 
            {
                res.push_back(tmp[i][j]); // res = {1,2}
                k--; // k = 0
                if(k == 0) return res;
            }
        }

        return res;
    }
};