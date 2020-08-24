class Solution {
public:
    int orangesRotting(vector<vector<int>>& grid) {
        int count = 0;
        int res = 0; 
        queue<vector<int>> q;
        
        vector<vector<int>> dir = {{-1,0},{1,0},{0,1},{0,-1}};
        
        // To count the number of the entire oranges
        for(int i=0; i<grid.size(); i++){
            for(int j=0; j<grid[0].size(); j++){
                if(grid[i][j] > 0) count++;
                if(grid[i][j] == 2) q.push({i,j});
                
            }
        }

        while(!q.empty())
        {
            
            res++;
            int q_size = q.size();
            // Because it is BFS, every rotten orange should be propagated.
            for(int i=0; i<q_size; i++)
            {
                

                for(int i=0; i<4; i++)
                {
                    int x = q.front()[0] + dir[i][0];
                    int y = q.front()[1] + dir[i][1];

                    if(x>=0 && x<grid.size() && y>=0 && y<grid[0].size() && grid[x][y] == 1)
                    {
                        grid[x][y] = 2;
                        q.push({x,y});
                    }
                }
                q.pop();
                count--;

            }

        }

        // The last iteration is not counted.
        // Because the last iteration will be ended with just a rotton orange, not propagated.
        res -= 1;
        
        // If all oranges are rotton
        if(count == 0) return max(0,res);
        else return -1; // If not, impossible
        
    }
};