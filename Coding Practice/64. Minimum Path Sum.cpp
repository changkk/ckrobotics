    /*
    
    1. Once the position is propagted, the position should go to the closed set
    2. previous cost should be added to the next level
    3. The cheapest way at the current level should be propogated first. (Queue, Map?)
    
    (0,0) cost 1 
    -> propagte -> put in the closed set (unordered_set)
    
    map = [ 2 (1,0)  //  4, (0,1) ]
    map.begin() -> propogated -> erase
    
    
    q.pop();
    minPathSum(q.top());
    -> recursively!
    
    cost 2 (1,0)
    (1,1) cost 5    //  (2,0)   cost 4
    min (cost 5, cost 4)   -> (2,0) cost 4
    
 
    Q = [ 4, (0,1)   //  6, (2,0)    // 7, (1,1) ]
    
    q = [ 6, (2,0)   // 7, (1,1)    //  5 (2,0)  // 9 (1,1) ]
    vector -> sort -> 
    
    q.pop();


    Tried to solve using Djkstra, but Dp is the easiest way.
    
        int minPathSum(vector<vector<int>>& grid) {
        map<int,vector<int>> cm;
        unordered_set<pair<int,int>> cs;
        cs.insert({0,0});
        cm.insert(grid[0][1]+grid[0][0],{0,1});
        cm.insert(grid[1][0]+grid[0][0],{1,0});


        return bfs(cs,cm,cm.begin(),grid);
        
 
    }
    
    int bfs(unordered_set<vector<int>>& cs, int cost, map<int,vector<int>>& cm, vector<int> cur, vector<vector<int>> grid)
    {
        
        if(cs.count({grid.size(),grid[0],size()})) return cost;
        
        vector<vector<int>> dir = {{0,1},{0,-1},{1,0},{-1,0}};
        for(int i=0; i<4; i++)
        {
            int x = cur[0] + dir[i][0];
            int y = cur[1] + dir[i][1];
            
            if(x>=0 && x<grid.size() && y>=0 && y<grid[0.size()] && cs.count({x,y})==0)
            {
                cm.insert(grid[x][y]+cost,{x,y});
            }
            
        }
        
        return bfs(cs.cm,cm.begin(),grid);
        
        
    }

    
    */
    

class Solution {
public:
    int minPathSum(vector<vector<int>>& grid) {
        int m = grid.size();
        int n = grid[0].size();
        
        vector<vector<int>> sum(m,vector<int>(n,grid[0][0]));
        
        // Start from the first row
        for(int i=1; i<n; i++)
            sum[0][i] = sum[0][i-1] + grid[0][i];
        
        // Star from the first column
        for(int i=1; i<m; i++)
            sum[i][0] = sum[i-1][0] + grid[i][0];
        
        for(int i=1; i<m; i++)
        {
            for(int j=1; j<n; j++)
            {
                sum[i][j] = min(sum[i-1][j],sum[i][j-1]) + grid[i][j];
            }
        }
        
        return sum[m-1][n-1];
        
    }

};