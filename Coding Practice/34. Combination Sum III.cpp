/*
    Backtracking?
    
    7, k= 3
    1 2 4
    
    12, k=5 
    1 2 3 6
    1 2 4 5
    
    From the small number, start from the smallest number
    and then add+ 
    
    1 3 4 4 x
    1 3 5 3 x
    1 3 6 2 x
    
    
    1 2 6
    1 3 5
    2 3 4
    3 4 2



*/


class Solution {
public:
    vector<vector<int>> res;
    
    void backtrack(vector<int> &sol, int k, int n){
        
        if(sol.size()==k && n == 0) res.push_back(sol);
        
        for(int i = sol.empty()? 1: sol.back()+1; i <= 9; i++)
        {
            if(n<i) break;
            sol.push_back(i);
            backtrack(sol,k,n-i);
            sol.pop_back();
        }
    }
    
    
    vector<vector<int>> combinationSum3(int k, int n) {
        vector<int> sol;
        backtrack(sol, k, n); // [] 3 7
        
        return res;
    }
};