/*
    customers = [1,0,7,2,1,1,1,5]
                [0,1,1,1,0,1,0,1]
        s       12 + c[i](!g[i]) = 12
                4 + 5 - 0 = 9
                (c[i] - c[i-X]*g[i-X]) > 0
        hide  = [1 1 1 - - - - -]
                
        O(N) time, O(1) space

*/

class Solution {
public:
    int maxSatisfied(vector<int>& c, vector<int>& g, int X) {
        int cur, res = 0;
        
        for(int i = 0; i < X; i++) res += c[i];
        
        cur = res;
        
        for(int i = X; i < c.size(); i++){
            cur += c[i] - c[i-X]*g[i-X];
            res += c[i]*(!g[i]);
            res = max(res, cur);
        }
        
        return res;
    }
};