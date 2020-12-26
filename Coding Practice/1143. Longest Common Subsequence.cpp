/*

       abcdc
      000000
    a 011111
    c 011222
    c 011223


*/


class Solution {
public:
    int longestCommonSubsequence(string st1, string st2) {
        int m = st1.length();
        int n = st2.length();
        vector<vector<int>> dp(m+1,vector<int>(n+1,0));
        
        
        for(int i = 1; i <= m; i++){
            for(int j = 1; j <= n; j++){
                dp[i][j] = st1[i-1] == st2[j-1]? dp[i-1][j-1] + 1 : max(dp[i][j-1],dp[i-1][j]);
            }
        }
        
        return dp[m][n];
    }
};


/* 1D version
class Solution {
public:
    int longestCommonSubsequence(string st1, string st2) {
        int m = st1.length();
        int n = st2.length();
        vector<int> dp(n+1,0);
        
        
        for(int i = 1; i <= m; i++){
            vector<int> tmp(n+1,0);
            for(int j = 1; j <= n; j++){
                tmp[j] = st1[i-1] == st2[j-1]? dp[j-1] + 1 : max(tmp[j-1],dp[j]);
            }
            dp = tmp;
        }
        
        return dp[n];
    }
};

*/