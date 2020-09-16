/*
    Goal : Find the cheapest way to get to the dst with maximum k stops
    1. BFS k
        make a map O(i) i - length of vector
        stop if I get to the dst
        stop if k is 0
        update the minimum cost
        O(k*n+i) n - # of branches O(1) space
    2.  [0,1,100] - cost: 100
        [1,2,100] - 
        [0,2,500]
        
        [1,100] -> a vector of nodes
        idx 0 [1,100] [2,500]
        idx 1 [2,100]
        
        unordered_map<int,vector<pair<
        vector<vector<pair<

*/

class Solution {
public:
    vector<vector<pair<int,int>>> save; // MISTAKE
    int minCost = INT_MAX; // 
    void bfs(int k, int src, int dst, int cost){
        if(src == dst){
            minCost = min(minCost, cost); // mincost = 200
            return;
        }
        if(k < 0) return;
        
        for(int i = 0; i < save[src].size(); i++){// src 0 k=-1 dst 1
            if(cost + save[src][i].second >= minCost) continue; // IMPORTANT
            bfs(k-1, save[src][i].first, dst, cost + save[src][i].second); // 0,2,2,200
        }
    }
    int findCheapestPrice(int n, vector<vector<int>>& flights, int src, int dst, int& K) {
        save.resize(100);
        for(int i = 0; i < flights.size(); i++){
            save[flights[i][0]].push_back(make_pair(flights[i][1],flights[i][2])); // dst,cost
        }
        // 0 -> 1 -> 2
        // 0 -> 2 
        bfs(K, src, dst, 0); // 1,0,2,0
        
        if(minCost == INT_MAX) return -1; //MISTAKE
        return minCost;
    }
};