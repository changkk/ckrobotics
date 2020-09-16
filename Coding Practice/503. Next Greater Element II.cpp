/*
    Goal: Find the next greadter number in the circular array.
    [4 2 1 3 1 6 5]
     6 3 3 6 6     
     3 3 6 6     
     cnt = 2
     for cnt
     push_back(-1)
     
    [1 1 1 1 1 1 1]
    
    [1 1 3 5 5 5]
     3 3 5 
    
     
    Brute: O(n*n) time O(n) space
    O(n*k+nlogn) time O(n) space

*/

class Solution {
public:
    vector<int> nextGreaterElements(vector<int>& nums) {
        vector<int> res(nums.size(),-1);
        stack<int> st;

        // [4 2 1 3 1 6 5]
        // st = 0 
        // res =[6 3 3 6 6 -1 -1]
        for(int i = 0; i < nums.size(); i++){
            
            while(!st.empty() && nums[i] > nums[st.top()] ){ // MISTAKE, 
                res[st.top()] = nums[i];
                st.pop();
            }

            st.push(i);
        }
        // pq = [6,5]                

        for(int i = 0; i < nums.size(); i++){ // MISTAKE: 0, -1 -> LOOP GOES ON
            while(!st.empty() && nums[i] > nums[st.top()] ){
                res[st.top()] = nums[i];
                st.pop();
            }
        }

        return res;
    }
};