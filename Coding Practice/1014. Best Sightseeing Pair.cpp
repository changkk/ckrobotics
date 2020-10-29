/*
    Goal: return the maximum score
    1. DP

    max: 8->7
    [7 1] max: 7 / res : 8
    max: 6 
    [6 0 5] max: 6 vs 5 / res: 8 vs 11

    [5 -1 4 2] max: 5 vs 2 / res: 11 vs 7
    [4 -2 3 1 6] max: 4 vs 6  // res: 11 vs 10



*/




class Solution {
public:
    int maxScoreSightseeingPair(vector<int>& A) {
        int maxNum = 0, res = 0;
        for(auto a:A){
            res = max(res, maxNum+a);
            maxNum = max(maxNum, a) - 1;
        }
        
        return res;
    }
};