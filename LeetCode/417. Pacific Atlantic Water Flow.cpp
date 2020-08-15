/*
    Find element that can reach to both j = 0 || i = 0 (Pacific) and i = m || j = n (Atlantic)
    
    * Optimization tip
    If a cell that can go both, then all cells in the path should be able to go.
    
    * Approach
    0. Make a bool matrix to save the status of the cell (if it can reach or not)
    1. Do BFS starting using linear search.
    2. While BFS, save the path, also check the bool mat. (if reached cell is 1, change the bool val of cells in path.)
    3. If the cell can reach both ocean, change all the bool value of the cells in the path.
    4. Keep doing linear search

    * Approach 2
    1. From the left bot corner, and right top corner (which is basic possible cell), Do BFS.
    2. If the next cell is bigger or equal, save it to res, and change the bool to 1.
    3. Do linear search from now. (except for bool 1)
    4. Check four branches if they could reach pacific or atalantic
        1. if j = 0, i= 0 or i =m j =n 
    5. If both could reach, then do the same thing for left bot and right top and change to bool to 1.
    6. return res;
    
    => O(m*n)


*/


class Solution {
public:
    vector<vector<int>> res;
    vector<vector<int>> visited;
    int m, n;
    

    void bfs(int i, int j, vector<vector<int>> matrix, int preval, int label){
        //                                      if val is smaller,     if already propagated and if not different label(1-2, 2-1), stop.
        if(i < 0 || j < 0 || i > m-1 || j > n-1 || matrix[i][j] < preval || visited[i][j] == 3 || visited[i][j] == label) return;
        
        visited[i][j]+=label;
        if(visited[i][j] == 3) res.push_back({i,j});
        bfs(i+1,j,matrix,matrix[i][j],label);
        bfs(i,j+1,matrix,matrix[i][j],label);
        bfs(i-1,j,matrix,matrix[i][j],label);
        bfs(i,j-1,matrix,matrix[i][j],label);
           
    }
    vector<vector<int>> pacificAtlantic(vector<vector<int>>& matrix) {
        if(matrix.empty()) return res;
        m = matrix.size(), n = matrix[0].size();
        
        visited.resize(m,vector<int>(n,0));
        

        for(int j=0;j<n;j++){
            bfs(0,j,matrix,INT_MIN,1);
            bfs(m-1,j,matrix,INT_MIN,2);
        }

        for(int i=0;i<m;i++){
            bfs(i,0,matrix,INT_MIN,1);
            bfs(i,n-1,matrix,INT_MIN,2);
        }
        return res;
        
    }
};