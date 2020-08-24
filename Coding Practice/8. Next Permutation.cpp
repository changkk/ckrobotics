/*
    1 5 3 4 2  => 1 5 3 4 2
    
    1 2 3 4 5
    1 2 3 5 4
    1 2 4 3 5
    1 2 4 5 3 - cur
    1 2 5 3 4
    1 2 5 4 3
    1 3 2 4 5
    1 3 2 5 4
    
    nums.size()-2 i--
    
    nums[i]
    sort(nums.begin()+i,nums.end());
    auto it = upper_bound(nums.begin()+i,nums.end(),nums[i]);
    if(it - nums.begin() == nums.size()) // nothing bigger
        i--;
    else
        swap(nums[i],nums[it-nums.begin()]);
        sort(nums.begin()+(i+1),nums.end());
    


*/

class Solution {
public:
    void nextPermutation(vector<int>& nums) {
        // 1 2 5 4 3
        // 5 4 3 2 1
        int idx = nums.size()-1;

        while(idx-1 >= 0)
        {
            sort(nums.begin()+idx,nums.end()); // idx = 2  // 3 4 5
            auto it = upper_bound(nums.begin()+idx,nums.end(),nums[idx-1]); // 
            if(it - nums.begin() == nums.size()) idx--;
            else
            {
                swap(nums[idx-1],nums[it - nums.begin()]); // 1 3 2 4 5
                sort(nums.begin()+idx,nums.end()); // 1 3 2 4 5
                idx = -1;
            }
        
        }
        
        if(idx == 0) sort(nums.begin(),nums.end());
        
        
        
    }
};