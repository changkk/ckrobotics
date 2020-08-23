/*
    Find k closest number 
    1 2 3 4 5
    v v v v
    
    which means find sorted array of abs(k-num)
    1. 2 1 0 1 2
    2. set -> I cannot save the number
    3. priority_queue<vector<int,int>> 
    4. map<int,set<int>> res nlogn
        0 3
        1 2 4
        2 1 5
        
        
    Approach (priority_queue)
    1. map<int,set<int>> res;
    2. Compute abs(x-num)
    3. Insert the difference,num to map
    4. From the first element of map, push the number to res until k is 0, k--
    5. sort the res vector. return
    6. (nlogn + klogk) time, O(n) space





class Solution {
public:
    vector<int> findClosestElements(vector<int>& arr, int& k, int x) {
        
        map<int,vector<int>> m;
        
        for(auto num:arr)
        {
            int diff = abs(x-num); // [2 1 0 1 2]
            m[diff].push_back(num); // [3] [2 4] [1 5]
        }
                
        vector<int> res;
        auto it = m.begin();
        while(k>0) // k = 1
        {
            vector<int> tmp = it->second; // [1 5]
            int vecSize = tmp.size(); // 2
            if(k < vecSize)
            {
                sort(tmp.begin(),tmp.end());
                for(int i = 0; i < k; i++)
                    res.push_back(tmp[i]);
                
                k -= vecSize; // k = 0
            }
            else
            {
                for(int i = 0; i < vecSize; i++)
                    res.push_back(tmp[i]); // res = [1 2 3 4]

                k -= vecSize; // 0                
            }
            
            it = next(it,1); // 
        }
        
        sort(res.begin(),res.end());
        return res; // [1 2 3 4]
            
    }
};*/

class Solution {
public:
    vector<int> findClosestElements(vector<int>& arr, int& k, int x) {
        
        int idx = lower_bound(arr.begin(),arr.end(),x) - arr.begin();
                
        int l = idx-1;
        int r = idx;
        
        // [2 1 0 1 2] //diff
        // [1 2 3 4 5]

        while(k--) // 0
        {
            if(l<0) r++;
            else if(r>=arr.size()) l--;
            else if(abs(arr[l]-x) > abs(arr[r]-x)) r++;
            else l--;
        }
        
        
        return vector<int>(arr.begin()+l+1,arr.begin()+r);
    }
};