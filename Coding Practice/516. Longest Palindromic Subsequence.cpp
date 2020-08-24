/*

    Case 1
        cabbac
        
    Case 2
        dcabacd


    1. Starting from idx 0, see if the next one is the same, or both side are the same.
    2. If the condition above is satisfied
        case1. left = idx; right = idx+1
        case2. left = idx; right = idx
    3. After that, left-- right++ until the both side are the same.
    4. If end, update res = max(res, right-left+1)
    
    faebbbcaef
    
    f a e c b    b    c a e c f
    
    res = 3, 3

*/

class Solution {
public:
    int longestPalindromeSubseq(string s) {
        vector<int> dp(s.size(),0);
        vector<int> dpPre(s.size(),0);
        
        for(int l = s.size()-1; l >= 0; l--)
        {
            dp[l] = 1;
            for(int r = l+1; r < s.size(); r++)
            {
                if(s[l] == s[r]) dp[r] = dpPre[r-1] + 2;
                else dp[r] = max(dpPre[r],dp[r-1]);
            }
            dp.swap(dpPre);
            dp.clear();
        }
        
        return dpPre[s.size()-1];
    }
};