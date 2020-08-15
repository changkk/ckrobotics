/*
    [1, 2, 3]
    4
    Try recursively.
    Using target 4, and try first number. Let's say 1
    
    1, 4 => target = 3
    
    if target == 0 count ++
    else if target < 0 break
    else 
    for all numbers:
        backtrack(target - 3,num)
    
    1. Check if the target is 0, if it is 0, count++
    2. Check if target < 0 break
    3. Check if target > 0
        try all numbers and do the same thing recursively.
    
    O(n^k)

    int helper (count, num, target)
        check the condition
        else
            for loop
                count += helper(0)


class Solution {
public:
    int helper(vector<int> nums, int target){
        int res = 0;
        if(target == 0) return 1;
        else if(target < 0) return 0;
        else
        {
            for(auto num:nums) // 1 2 3
                res += helper(nums, target-num); // 0,1,
        }
        return res;
    }
    int combinationSum4(vector<int>& nums, int target) {
        return helper(nums,target); // [1 2 3] target = 4
    }
};

=> Time Limit

*/

class Solution {
public:
    int combinationSum4(vector<int>& nums, int target){
        vector<unsigned int> dp(target+1);
        dp[0] = 1;
        
        for(int i = 1; i <= target; i++)
        {
            for(auto num:nums)
                if(i-num >= 0) dp[i] += dp[i-num];
        }
        
        return dp[target];
        
    }
    
    
};