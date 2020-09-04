/*

    Goal: Find the sum of minimums of subarrays of A
    1. How to find subarraies?
        - Ac1, Ac2, Ac3, ... AcA
        - 1: 3 1 2 4 O(n)
        - 2: [3 1] [3 2] [3 4] [1 2] [1 4] [2 4] O(n)
        - 3: [3 1 2] [3 1 4] [3 2 4]
        - O(n^2)
        
        - 1: 3 1 2 4  -> 3
        - 2: 1 2 3 4 -> [1 2] [1 3] [1 4] : 1 x 3 => arr[i] * pC1 -> 3
                     -> [2 3] [2 4] : 2 x 2 => arr[i] * pC1 -> 2
                     -> [3 4] : 3 x pC1 -> 1
        - 3: 1 2 3 4 -> [123] [124] [134] => 1 x pC2 -> 3C2 => 3*2/2 => 3
                     -> [2 3 4] => 2 x pC2 => 2 x 2*1/2 => 1
                     
                     
        - 1: [1 2] [1 3] [1 4] 3C1    3 
             [1 2 3] [1 2 4] [1 3 4] 3C2  3
             [1 2 3 4] 3C3   1      => 1 * 7
        - 2: [2 3] [2 4] 2C1 2 
             [2 3 4] 2C2 1    => 2 * 3
        - 3: [3 4] 1C1   1   => 3  * 1

        
        1        1
        2 1 =>   3
        3 3 1  => 7
        4 6 4 1 => 15
        5 10 10 5 1  => 31
        6 15 20 15 6 1   => 12 + 30 + 20 = 63
        
    2. O(N)


    3 1 2 4
    31 12 24 312 124 3124
    1-> [1] [1 2] [1 2 4] [3 1 2] [3 1 2 4]
    2-> [2] [2 4] []
    3-> [3]
    4-> [4]
    
    7 5 2 2 2 8
    
   7 5 2 4 2 8 v
       v  
   4
   4
   4
   
   O(n*n) O(1)
    1. Find the first small number on the left and right
    2. Compute the number of elements
    3. multiply the numbers
    
    4 3 2 4
     
    
    71 * 1
    55 * 
    82
    55
*/


class Solution {
public:
    int sumSubarrayMins(vector<int>& A) {
        int sum = 0;
        stack<pair<int,int>> ls, rs;
        vector<int> left(A.size()), right(A.size());
        
        for(int i = 0; i < A.size(); i++)
        {
            while(!ls.empty() && ls.top().first > A[i]) ls.pop();
            
            if(ls.empty()) left[i] = i+1;
            else left[i] = i - ls.top().second;
            ls.push(make_pair(A[i],i));
        }
        
        for(int i = A.size()-1; i >= 0; i--)
        {
            while(!rs.empty() && rs.top().first >= A[i]) rs.pop();
            
            if(rs.empty()) right[i] = A.size()-i;
            else right[i] = ls.top().second - i;
            rs.push(make_pair(A[i],i));
        }
        
        for(int i = 0; i < A.size(); i++) // 4 / 3
        {
            int l = i-1, r = i+1; // 2 4
                
            sum = (sum + A[i] * left[i] * right[i])%(1000000000+7); // 4 * (1) * (1) = 3 + 6 + 4 + 4 = 17
                        
        }
        
        return sum;
    }
};