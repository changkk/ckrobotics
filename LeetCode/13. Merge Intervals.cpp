/*
    Goal: Merge all overlapping intervals
    
    1. Sort the vector by the first value.
    2. Check from the first one if the next key is smaller than the end key.
    3. If it is, change the end to new end.
    4. If not, do the same thing for the next one.
    5. keep doing that until the end.
    6. O(nlogn+n^2) time O(n) space



class Solution {
public:
    static bool sortcol(vector<int> a, vector<int> b){
        return a[0] < b[0];
    }
    
    
    vector<vector<int>> merge(vector<vector<int>>& intervals) {
        unordered_map<int,int> ans;
        vector<vector<int>> res;

        if(intervals.size() == 0) return res;
        if(intervals.size() == 1) return intervals;
        
        sort(intervals.begin(),intervals.end(),sortcol);
        int a = 0, b = 1;
        while(a < intervals.size() && b < intervals.size())
        {
            // If there is an intersection
            if(intervals[a][0] == intervals[b][0] || intervals[a][1] >= intervals[b][0])
            {
                intervals[a][1]=max(intervals[a][1],intervals[b][1]);
                ans[intervals[a][0]] = max(intervals[a][1],intervals[b][1]); // [1, 6]
                b++; // a =0, b=2
            }
            else // If not 
            {
                ans.insert({intervals[a][0],intervals[a][1]}); //[1,6] [8, 10] [15,18]
                if(b==intervals.size()-1) ans.insert({intervals[b][0],intervals[b][1]}); 
                a=b; // a = 2
                b=a+1; // b =3
            }
                
        }
        
        for(auto i:ans)
        {
            res.push_back({i.first,i.second});
        }
        
        return res;
        
    }
};

 => Slow
*/



class Solution {
public:
    static bool sortcol(vector<int> a, vector<int> b){
        return a[0] < b[0];
    }
    
    
    vector<vector<int>> merge(vector<vector<int>>& intervals) {
        vector<vector<int>> res;

        if(intervals.size() == 0) return res;
        if(intervals.size() == 1) return intervals;
        
        sort(intervals.begin(),intervals.end(),sortcol);
        res.push_back(intervals[0]); // res = [1,3]
        
        for(int i = 1; i<intervals.size(); i++)
        {
            if(res.back()[1] < intervals[i][0]) res.push_back(intervals[i]); // [1,6] [8,10], [15,18]
            else
            {
                res.back()[1] = max(res.back()[1],intervals[i][1]); // res = [1,6]
            }
        }
        
        return res;
        
    }
};