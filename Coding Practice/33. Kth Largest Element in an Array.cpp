/*
    3 2 1 5 6 4
    kth largest
    
    Update first and second every time
    push first element and second element
    vector<int> = {6 5 4 2 3}
    unordered_set<int> {6 5 2 4}
    
    
    largest  [3 2]
    smallest [-2 -3]
    
    if()
    

*/


class Solution {
public:
    int quicksort(vector<int> nums, int left, int right){
        
        if(left == right) return right;
        
        int l = left + 1, r = right;
        int pivotidx = rand()%(r-l+1) + l;
        swap(nums[pivotidx],nums[left]);
        // [5] 3 2 1   6 4 
        int pivot = nums[left];
        while(l <= r)
        {
            if(nums[l] >= pivot) l++; // l = 2
            if(nums[r] <= pivot) r--; // r = 2
            if(nums[l] < pivot && nums[r] > pivot) swap(nums[l++],nums[r--]); // 5 6 2 1   3 4 
        }
        swap(nums[left],nums[r]);
        return r;
    }
    
    int findKthLargest(vector<int>& nums, int k) {
        int left = 0, right = nums.size()-1;
        int pivot;
        
        while(left<right)
        {
            pivot = quicksort(nums,left,right);

            if(pivot == k-1) return nums[pivot];
            if(pivot < k-1) left = pivot + 1;
            if(pivot > k-1) right = pivot - 1;            
        }
        
        return nums[left];
        
    }
};