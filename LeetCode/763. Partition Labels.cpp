/*
    Make splits to each substring does not have letter from other substrings, and return the length of each.
    
    1. Save the latest position for each letter for a loop.
    2. Second loop: Save the maximum position of the letter we have seen so far.
    
    ababcbaca defegde hijhklij
    012345678 9012345 6789
    
    888888888 /
    
    10
    


*/


class Solution {
public:
    vector<int> partitionLabels(string S) {
        vector<int> maxs(26,0);
        
        for(int i = 0; i < S.length(); i++)
            maxs[S[i]-'a'] = i; // letters[a] = 8
        
        int maxPos = 0, prevMaxPos = -1;
        vector<int> res;
        for(int i = 0; i < S.length(); i++)
        {
            maxPos = max(maxPos,maxs[S[i]-'a']); // 19
            if(maxPos == i) // 15
            {
                res.push_back(maxPos-prevMaxPos); // 15-8 = 7
                prevMaxPos = maxPos; // 15
            }        
        }
        
        return res;
    }
};