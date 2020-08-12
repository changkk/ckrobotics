/*
    {e a t} {e e t} [tea]
    tan nat atn ant


*/


class Solution {
public:
    string bucketSort(string str){
        vector<int> cs(26,0);
        for(auto c:str)
            cs[c-'a']++;
        
        string res = "";
        for(int i = 0; i < 26; i++)
        {
            for(int j = 0; j < cs[i]; j++)
                res += (i+'a');
        }
        
        return res;
    }
        
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        unordered_map<string, vector<string>> anagrams;
        for(auto str:strs)
            anagrams[bucketSort(str)].push_back(str);
        
        vector<vector<string>> res;
        
        for(auto ana:anagrams)
            res.push_back(ana.second);
        
        return res;
    }
};