/*

    [2 3 2 1 1] => 3
    
    odd: 2 2 1 -> odd => 2 2 or 1 2 => 4 or 3
    even: 3 1 => 4
    
    compare two sums
    
    [1 2 3 1] 
    
    even: 1 3 => 4
    odd: 2 1 -> 3
    4
    
    
    [1 3 2 1 5 1 7 1 3] => 
    [100 3 1 3 1]


*/


class Solution {
public:
    int helper(vector<int>& nums, int l, int r){
        int pre = 0;
        int cur = 0;
        
        for(int i = l; i < r; i++)
        {
            int tmp = max(pre + nums[i], cur);
            pre = cur;
            cur = tmp;
        }
        
        return cur;
    }
    
    int rob(vector<int>& nums) {
        
        if(nums.size()<2) return nums.size()? nums[0] : 0;
            
        return max(helper(nums,0,nums.size()-1),helper(nums,1,nums.size()));
        
    }
    
    
};