/*
    0101
    101010
    111000
    
    1 -1 = 0
    
    count 1 2 and 0 4
    110000
    101000
    001001
    
    0110110
    
  0 22211
  1 54332
  s -0101
    0   1  1  0  1  1  1  0
sum -1  0  1  0  1  2  3  2
lon 4 
    
*/

class Solution {
public:
    int findMaxLength(vector<int>& nums) {
        unordered_map<int,int> mem;
        
        int sum = 0;
        int res = 0;
        
        for(int i = 0; i < nums.size(); i++)
        {
            if(nums[i] == 0) sum--; // 1
            else sum++;
            
            if(sum == 0) res = i+1; // 4
            else
            {
                if(mem.find(sum) == mem.end()) mem[sum] = i; // -1,0 1,2
                else res = max(res, i - mem[sum]); // 4-2         
            }
            
        }
        
        return res;
    }
};