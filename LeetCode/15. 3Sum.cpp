class Solution {
public:
    vector<vector<int>> threeSum(vector<int>& nums) {
        vector<vector<int>> res;
        sort(nums.begin(),nums.end());

        for(int i=0; i<nums.size(); i++)
        {
            int target = -nums[i];
            int front = i+1;
            int back = nums.size()-1;

            // Just using this while loop, everything concerning about the size is solved.
            while(front<back)
            {
                int sum = nums[front] + nums[back];
                // Even though using same nubers, that can be answer, 
                // so all the numbers should be scanned.
                if(target>sum) front++;
                else if(target<sum) back--;
                else
                {
                    vector<int> ans(3,0);
                    ans[0] = -target;
                    ans[1] = nums[front];
                    ans[2] = nums[back];
                    res.push_back(ans);

                    // Once the answer is found, scanning same number is not meaningful.
                    // In order to go to the next number while passing the duplicate answer
                    while(front<back && ans[1]==nums[front]) front++;
                    while(front<back && ans[2]==nums[back]) back--;
                }
                
                while(i+1<nums.size() && nums[i]==nums[i+1]) i++;

            }

        }
        
        return res;

        
        
     
        
    }
};