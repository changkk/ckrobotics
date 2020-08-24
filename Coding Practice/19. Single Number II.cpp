/*

    Count each number and if a number is 3 then we can remove?
    [2 - 3]
    [3 - 1]
    [2 2 2 3]
    [9 9 8 9 8 6 8]
    => 9*3 + 8*3 + 6
    => 9, 8, 6 => 9*3 + 8*3 + 6*3 -> diff = 12 /2 = 6
    => set = 



class Solution {
public:
    int singleNumber(vector<int>& nums) {
        unordered_set<int> numbers; // O(n/3)
        long sum = 0;
        long comp = 0;
        for(auto num:nums)
        {
            if(numbers.find(num)==numbers.end()) // 3
            {
                comp += num; // 6 + 9
                comp += num;
                comp += num;
                numbers.insert(num); // [2 3]
            }
            
            sum += num; // 9
        }
        
        return (comp-sum)/2; // 15 - 9 / 2 = 3
    }
};

*/

class Solution {    
    public:
    int singleNumber(vector<int>& nums) {
        int ones = 0, twos = 0;
        for(int i = 0; i < nums.size(); i++){
            ones = (ones ^ nums[i]) & ~twos;
            cout<<ones<<endl;
            twos = (twos ^ nums[i]) & ~ones;
            cout<<twos<<endl;
            cout<<" "<<endl;
        }
        return ones;
    }
};