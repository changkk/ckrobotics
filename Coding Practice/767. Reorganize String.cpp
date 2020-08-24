/*
    aab -> aba
    
    aaab
    
    aabbccc
    aba
    
    cccaabb
    c 3 a 3 b 1
    c 1 a 2 b 1
    cacba
    ca ca ca aa aa
    
    c aca ca c aca
    


*/

class Solution {
public:
    string reorganizeString(string& S) {
        vector<int> cnt(26,0);
        int maxNum = 0;
        int maxIdx = -1;
        
        for(auto s:S)
        {
            cnt[s-'a']++;
            maxNum = max(maxNum,cnt[s-'a']);
            if(maxNum == cnt[s-'a']) maxIdx = s-'a';
        }
        
        if(maxNum*2 - 1 > S.size()) return {};
        
        int i = 0;
        while(cnt[maxIdx] > 0)
        {
            S[i] = maxIdx + 'a';
            cnt[maxIdx]--;
            i += 2;
        }
        
        for(int j = 0; j < 26; j++)
        {
            while(cnt[j] > 0)
            {
                if(i>=S.size()) i = 1;
                S[i] = j + 'a';
                cnt[j]--;
                i += 2;
            }
        }
        
        return S;
    }
};