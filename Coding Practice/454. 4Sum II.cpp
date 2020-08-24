/*


    [1 2 3 4v]
    [-2 -1 4v 6]
    [-3 1 2 3v]
    [-6v -2 -1 0]
    -10 
    
    queue 2b 3b 4(d)
    sum -9 -8 -7 -6 -4 -3 -2 1
    
    

    1. Sort them
    2. Compute the difference between cur and next for each array
    3. Push the next idx to a priority queue in order of the different (from smallest difference)
    4. In the next iteration, add the idx top of the pq. and then update the queue
    5. By doing that, once a order is found, 


*/
class Solution {
public:
    int fourSumCount(vector<int>& A, vector<int>& B, vector<int>& C, vector<int>& D) {
        unordered_map<int,int> AB;
        
        for(auto a:A)
            for(auto b:B)
                AB[a+b]++;
        
        int res = 0;
        for(auto c:C)
            for(auto d:D)
                res += AB[-c-d];

        return res;
    }
};