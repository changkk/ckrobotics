/*
    [1 0 2]
    1 0 2
    lo++
    
    [4 2 0 3]
    l: 2
    g: 3 + 1 
    1 2 0 


    [0 2 4 1]

*/

class Solution {
public:
    bool isIdealPermutation(vector<int>& A) {
        if(A.size()<2) return true;
        
        int maxA = A[0]; // max = 2
        for(int i = 1; i < A.size()-1; i++){            
            if(A[i+1] < maxA) return false; // 0<2, 4!=2
            maxA = max(maxA,A[i]); // max = 2
        }
        
        return true;
    }
};