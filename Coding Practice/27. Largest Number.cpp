/*
    Non-negative int
    Arrange that to form the largets num
    
    Start from the largest number
    If the start num is the same, compare other numbers and if other sort it from the biggest. 
    
    352 35 35235 < 35352
    
    
    34235301 34233301
    
    34 3 30
    
    5 56  => 565 > 556
    343 303 330

    From the first number, check the first number and push it to 0-9 vector
    
    and from the 9, delete the 9, and push it to a 0-9 vector


*/


class Solution {
public:
    static bool helper(int a, int b){
        return to_string(a)+to_string(b) > to_string(b)+to_string(a);
    }
    string largestNumber(vector<int>& nums) {
        sort(nums.begin(),nums.end(),helper);
        
        if(nums[0]==0) return "0";
        
        string res;
        
        for(auto num:nums)
            res += to_string(num);
        
        return res;
        
    }
};