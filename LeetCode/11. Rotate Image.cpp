/*
    (0,0) -> (0,3)
    (0,1) -> (1,3)
    (0,2) -> (2,3)
    (0,3) -> (3,3)
    
    (0,3) -> (3,3) -> (3,0)
    (1,3) -> (3,2) -> (1,0)
    (2,3) -> (3,1) -> (2,0)
    (3,3) -> (3,0) -> (0,0)
    
    ()
    
    (2,1) -> (1,1)
    (1,1) -> (1,2)
    (1,2) -> (2,2)
    
    1 2 3 4 5 
    5 6 7 8 9
    9 9 9 9 1
    5 2 3 8 2
    1 8 8 6 9
    
    Let's say the outmost length of the matrix is n
    
    while i<n
    
    for start:n

    int tmp = matrix[i][j]
    for(1:4)
        if(4) matrix[i][j] = tmp;
        else matrix[i][j] = matrix[n-j][i];
        isave = i, jsave = j
        i = n-jsave, j = isave;
    
    j++
    
    n--
    i++
    
    O(n^2)
    
    

*/


class Solution {
public:
    void rotate(vector<vector<int>>& matrix) {
        
        int s = 0, n = matrix.size()-1;
        
        while(s < n)
        {
            for(int j = s; j < n; j++)
            {
                int tmp = matrix[s][j]; // 1,1 -> 4
                int i = s;
                for(int k = 0; k < 4; k++)
                {
                    if(k == 3) matrix[i][j] = tmp;  // 1,1 
                    else matrix[i][j] = matrix[n-j+s][i]; // 1,2 = 1,1
                    
                    cout<<i<<" "<<j<<endl;
                    int iSave = i;
                    int jSave = j; // 0 0
                    i = n - jSave + s;
                    j = iSave; // 3 0
                }
            }
            n--; // 2
            s++; // 1
        }

        
    }
};