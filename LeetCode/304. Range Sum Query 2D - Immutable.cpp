/*

    2 0 1
    1 0 1
    0 3 0
    
    
    3 0 1 4 2
    5 6 3 2 1
    1 2 0 1 5
    4 1 0 1 7
    1 0 3 0 5
    
    3  3  4  8  10
    8 14 18 24 27
    9 17 21 28 36
    13 20 24 
    6 25 46  77 122


*/


class NumMatrix {
public:
    vector<vector<int>> sumMat;
    NumMatrix(vector<vector<int>>& matrix) {
        if(!matrix.empty())
        {
            sumMat.resize(matrix.size());
            for(int i = 0; i < sumMat.size(); i++)
                sumMat[i].resize(matrix[0].size());


            // Compute sumMatrix
            for(int i = 0; i < matrix.size(); i++)
            {
                for(int j = 0; j <matrix[0].size(); j++)
                {
                    sumMat[i][j] = a(i,j-1) + a(i-1,j) - a(i-1,j-1) + matrix[i][j];
                }
            }



        }
    }
    
    int sumRegion(int row1, int col1, int row2, int col2) {
        
        if(sumMat.empty()) return 0;

        return a(row2,col2) - a(row1-1,col2) - a(row2,col1-1) + a(row1-1,col1-1);

    }
    
    private:
    int a(int i, int j){
        return i >= 0 && j >= 0 ? sumMat[i][j] : 0;
    }
};

/**
 * Your NumMatrix object will be instantiated and called as such:
 * NumMatrix* obj = new NumMatrix(matrix);
 * int param_1 = obj->sumRegion(row1,col1,row2,col2);
 */