/*
    Goal; Any substring that has length k, what is the maximum number of vowels in the substring 

    leetcode
   v01100101
   n01221112
   max n
    n[i-1]+v[i]-v[i-3]
    2-1 = 1
    k = 3
    output : 'length' -> number
    
    deque for v [0 1 1 ]
    pVow = 
    maxVow
    
    1. check if cur is vowels
    2. vowels++
    
    O(n) time, O(1) space
*/

class Solution {
public:
    int maxVowels(string s, int k) {
        deque<int> dq;
        int curVow = 0, maxVow = INT_MIN;
        
        // leetcode 
        // dq=[1 0 1]
        // curVow = 2
        // maxVow = 2
        int vowels[26] = {1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1};
        
        for(int i = 0; i < s.length(); i++){
            curVow += vowels[s[i]-'a'];
            if(i>=k)
                curVow -= vowels[s[i-k]-'a'];
            maxVow = max(maxVow,curVow);
        }
        
        return maxVow;
    }
};