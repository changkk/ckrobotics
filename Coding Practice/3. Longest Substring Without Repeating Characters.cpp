/*
    Goal: Find a substring that does not have repetitive chars.
    1. uset a b c b b c b b
            1 2 3 1 2 3 2 3 3 3 3
            b b b b b
            1 1 1 1 1
            p w x k w k
            1 2 3 4 3

*/

class Solution {
public:
    int lengthOfLongestSubstring(string s) {
        unordered_set<char> temp;
        int i = 0, j = 0, count = 0;
        int slen = s.length();
        
        while (j < slen)
        {
            if(temp.find(s[j]) == temp.end())
            {
                temp.insert(s[j]);
                j++;
                count = max(count,j - i);
            }
            else
            {
                // Erase front elements until seeing the repetitive char.
                temp.erase(s[i]);
                i++;
            }
        }
        
        return count;

    }
};