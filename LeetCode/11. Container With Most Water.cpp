/*
    Goal: Find the largest water container
    1. Start from the widest one
    2. Change the x and see if the height is higher


*/


class Solution {
public:
    int maxArea(vector<int>& height) {
        int i = 0, j = height.size()-1;
        int water=0;
        
        while(i<j)
        {
            int min_h = min(height[i],height[j]); // 7
            water = max(min_h * (j-i), water); // 7*(7) = 49
            while(height[i]<=min_h && i<j) i++; // i 1
            while(height[j]<=min_h && i<j) j--; // if height[j] is larger than min_h, stop, else j--
        }
        
        return water;
    }
};