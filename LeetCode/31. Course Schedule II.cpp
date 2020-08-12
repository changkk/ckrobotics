/*
    [1 0][2 0][3 1][3 2]
    
    1->0 2->0 3->1 3->2
    
            0
        1       2
            3

    1. Draw a graph
    
    1.prev => 0
    2.prev => 0
    3.prev => 1
    3.prev => 2
    
    0.next => 1
    0.next => 2
    1.next => 3
    2.next => 3
    
    arr[1] = 0
    arr[2] = 0
    
    class Node {
    
    Node* next;
    Node* 
    
    }

*/


class Solution {
public:
    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
        
        vector<vector<int>> graph(numCourses,vector<int>());
        vector<int> indegree(numCourses);
        
        for(auto p:prerequisites)
        {
            graph[p[1]].push_back(p[0]); // graph[0] = 1
            indegree[p[0]]++;            // indegree[1] = 1
        }
                
        queue<int> q;
        for(int i = 0; i < indegree.size(); i++)
        {
            if(indegree[i] == 0) q.push(i);
        }
                
        vector<int> res;
        while(!q.empty())
        {
            int cur = q.front();
            q.pop();
            res.push_back(cur);
            for(auto g:graph[cur])
            {
                indegree[g]--;
                if(indegree[g] == 0) q.push(g);
            }
        }

        if(res.size() == numCourses) return res;
        else return {};
        
    }
};