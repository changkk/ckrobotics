/*

    Goal: Find a shortest path from the top left to bot right
    0 0 0
    1 1 0
    1 1 0
    
    m n matrix
    
    1 2 3
    0 0 3
        4

            0
         1     2    numberlevel 
    3      4      5

    q = 3 queue<pair<int,int>>
    visited = [0,0][0,1][1,0][1,1][0,2][1,2] visit = unordered_map<pair<int,int>>
    
    O(mn) time, O(mn) space

*/

class Solution {
public:
    static bool compare(vector<int> a, vector<int> b)
    {
        return a[0]<b[0];
    }
    
    int shortestPathBinaryMatrix(vector<vector<int>>& grid) {
        int ans = 1;
        priority_queue<vector<int>,vector<vector<int>>,greater<vector<int>>> q; // Remember
        
        // Mistake : Edge cases!!!!
        if(grid[0][0] == 1) return -1;
        
        int m = grid.size();
        int n = grid[0].size(); // m and n should be the size of matrix to avoid the confusion!!!

        vector<vector<int>> visited(m,vector<int>(n,0)); // Mistake: map -> set -> vector

        vector<vector<int>> dir = {{-1,0},{-1,-1},{-1,1},{0,-1},{1,-1},{1,0},{0,1},{1,1}}; // Big mistake: all direction should be included
        q.push({m+n,0,0,1});
        
        while(!q.empty()) // [2,0,2,3][1,1,2,3]    
        {
 
            vector<int> cur = q.top(); // queue -> front!!!!!!!
            q.pop(); 

            for(int j = 0; j < 8; j++)
            {
                int x = cur[1] + dir[j][0];
                int y = cur[2] + dir[j][1];
                if(x == m-1 && y == n-1) return cur[3]+1;
                
                if(x>=0 && y>= 0 && x < m && y < n)
                {
                    if(grid[x][y] == 0 && visited[x][y] == 0) 
                    {
                        q.push({(m-x+n-y),x,y,cur[3]+1}); // q = [2,0,2,3][1,1,2,3]  [0,2,2,4] 
                        visited[x][y] = 1; // [0,0][0,1][0,2][1,2]
                    }
                }
            }

            
        }
        
        // Mistake : forgot final condition !!!!!
        return -1;        
        
    }
};