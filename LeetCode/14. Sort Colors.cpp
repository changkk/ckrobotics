/*
    


*/

class Solution {
public:
    void sortColors(vector<int>& nums) {
        int red = 0;
        int white = 0;
        int blue = 0;
        for(int i = 0; i < nums.size(); i++)
        {
            if(nums[i] == 0) red++; // 2
            if(nums[i] == 1) white++; // 2
            if(nums[i] == 2) blue++; // 2
        }
        
        
        for(int i = 0; i < nums.size(); i++)
        {
            if(red == 0 && white == 0)
            {
                if(blue > 0)
                    nums[i] = 2; blue--;
            }
                
            if(red == 0)
            {
                if(white>0)
                    nums[i] = 1, white--;
            }
                
            if(red > 0)
                nums[i] = 0, red--; // 0 0 1 1 2 2
            
            cout<<i<<endl;
        }
    }
};