/*
    Binary search the number
    1. Start with left = 0 right = n and center = (left+right)/2
        - See if center > right => the pivot is on the right 
        - See if center < left  => the pivot is on the left or at the center
        - else, the pivot is at left
        
    2. See if the target is smaller than the center
        
    
    
        - If pivot is on the right, see if target <= right
            - if it is, repeat with center - right
        - if not, see if target >= left
            - if it is, repeat with left - center
        - if not return -1
    5. Compute the new center
    6. Repeat until 
    
    0 1 2 3 4 5 6
    5 6 7 0 1 3 4
    
    7 0 1 2 3 4 5
    
    0 1 2 3 4 5 6
    3 4 5 6 7 0 1
    

*/


class Solution {
public:
    int search(vector<int>& nums, int target) {
       
        int low = 0;
        int high = nums.size()-1;
        int mid;
        // Find the pivot (smallest number)
        while(low < high)
        {
            mid = (low+high)/2;
            if(nums[mid] > nums[high]) low = mid + 1;
            else high = mid;
        }
        int pivot = low;
        low = 0; high = nums.size()-1;
        
        while(low <= high)
        {
            mid = (low+high)/2;
            // Find the actual middle point from the pivot
            int virtualMid = (mid+pivot)%(nums.size());
            if(nums[virtualMid] == target) return virtualMid;
            else if(nums[virtualMid] < target) low = mid + 1;
            else high = mid-1;
        }
        
        return -1;
        
    }
};