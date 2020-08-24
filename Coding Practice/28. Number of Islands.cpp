class Solution {
public:
    
        
    void bfs(vector<vector<char>>& grid, int r, int c){
        
        
        grid[r][c] = 0;
        int row_size = grid.size();
        int col_size = grid[0].size();
        
        if(r+1<row_size && grid[r+1][c]=='1')
            bfs(grid,r+1,c);
        if(r-1>=0 && grid[r-1][c]=='1')
            bfs(grid,r-1,c);
        if(c+1<col_size && grid[r][c+1]=='1')
            bfs(grid,r,c+1);
        if(c-1>=0 && grid[r][c-1]=='1')
            bfs(grid,r,c-1);
    }
    
    
    int numIslands(vector<vector<char>>& grid) {
        int row_size = grid.size();
        int col_size = grid[0].size();
        int num_of_island = 0;

        //Edge cases (Important)
        if(row_size)
        {
            for(int r=0; r<row_size; r++)
            {
                for(int c=0; c<col_size; c++)
                {
                    if(grid[r][c]=='1')
                    {
                        bfs(grid,r,c);
                        num_of_island++;
                    }
                }
            }
        }
        
        return num_of_island;
        
    }

};