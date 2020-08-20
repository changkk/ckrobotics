/*

    Change k letters to make a longest repetitve substring
    1. start = 0 end = 0
    2. Once encouter to an alphabet, save the number of the alphabet in the window.
    3. Check the condition:
        end-start+1 - MaxFreAl <= k
    4. If not, start++, count[s[start]]--;
    5. Update maxFreAl = max(MaxFreAl, count[s[end]]);
    6. After the loop return MaxFreqAl + k;

    A ABABBA
            A B
    count   2 2
    condi   1  > k
    max     3
    start   1
    end     4
*/


class Solution {
public:
    int characterReplacement(string s, int k) {
        int start = 0, end = 0, MaxFreqAl = 0;
        vector<int> count(26,0);
        int res = 0;
        
        while(end < s.length())
        {
            count[s[end]-'A']++; // count the encountered alphabet 
            MaxFreqAl = max(MaxFreqAl,count[s[end]-'A']); // Update the maxfreqal of the current window
            // If the window cannot satisfy with the current MaxFreqAl, that means the previous window is the longest one, so just start++
            if(end-start+1 - MaxFreqAl > k) // Check the window satisfy the window
            {
                // If it is not, decrease the window, and make the window to satsify.
                res = max(res,end-start);
                count[s[start]-'A']--;
                start++;
            }
            end++;
        }
        return max(res,end-start);
    }
};