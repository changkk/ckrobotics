/*
    [1 4 2 5 3 8 
    1 4 2 5 3 
    
    1 4 2
    4 2 5
    2 5 3
    5 3 8

    1 4 2 5 3
    4 2 5 3 8

    
    5-> 1 3 5
    1+3 * 3 = 12
    4+5 * 4 = 36
    2 * 5 = 10
    58
    
    [1,2]
    1 -> 1
    1+2 * 1 = 3
    
    [10,11,12]
    3 -> 1 ,3
    10+12 * 2 = 22
    11 * 2 == 22
    44
    
    
    1 ~ 3 5 < n - lenth of arr
    
    1, 1 4 2, 1 4 2 5 3
    4, 4 2 5
    2, 2 5 3
    5
    3
    

*/


class Solution {
public:
    int sumOddLengthSubarrays(vector<int>& arr) {
        
        int n = arr.size();
        int t;
        
        int l = 0, r = n-1;
        int sum = 0;
        
        while(l<=r){
            t = ((l+1)*(n-l)+1)/2;
            if(l==r) sum+= arr[l]*t;
            else sum+= (arr[l]+arr[r])*t;
            l++, r--;
        }
        
        return sum;
    }
};