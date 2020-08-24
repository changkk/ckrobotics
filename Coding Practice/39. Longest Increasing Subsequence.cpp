/*
    - - + - + + -
    -1 -7 +3 -2 +4 +94 -83
    
    
    3 7 101 18 10 9 2 5
      4 98 -83 -8 -1 -7 -3
    
    10 9 2 5 3 7 101 18
    
    min 10 9 2 2 2
    max 10 9 2 5 3
    
    start = min(min,num) -> 9
    end = 9
    
    start = 2
    end = 2
    
    

class Solution {
public:
    int lengthOfLIS(vector<int>& nums) {
        
        if(nums.size()==0) return 0;
        vector<int> dp(nums.size(),1);
        int max_dp = 1;
        
        for(int i = 1; i < nums.size(); i++)
        {
            for(int j = 0; j < i; j++)
            {
                if(nums[i]>nums[j]) 
                {
                    dp[i] = max(dp[i],dp[j]+1);
                    max_dp = max(max_dp, dp[i]);
                }
            }

        }
        
        return max_dp;
        
    }
};

    
    10 9 2 5 3 7 101  1
    1  1 1 2 2 3 4    1
       

*/


class Solution {
public:
    int lengthOfLIS(vector<int>& nums) {
        vector<int> res;
        for(int i = 0; i < nums.size(); i++)
        {
            auto it = lower_bound(res.begin(),res.end(),nums[i]);
            if(it == res.end()) res.push_back(nums[i]);
            else *it = nums[i];
        }
        
        return res.size();
    }
};
