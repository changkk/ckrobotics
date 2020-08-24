/*
    Goal: Find repetitive words and sort them in ascending order
    
    BF: Find and push front or push back
    Using umap
    1. if the word is not inside the map, add
    2. if the word is inside the map, count++
    3. After complete the map, rotate map and push front(bigger) or push back (smaller)


*/


class Solution {
public:
    vector<string> topKFrequent(vector<string>& words, int k) {
        map<string, int> map;
        // Map automatically save the word in alphabetically
        for(auto& word:words)
            map[word]++;
        
        vector<vector<string>> bucket(words.size());
        for(auto key:map)
            bucket[key.second].push_back(key.first);
           
        vector<string> ans;
        for(int i=bucket.size()-1; i>=0 && k>0; i--)
        {
            if(bucket[i].empty()) continue;
            
            int n = min(k,(int)bucket[i].size());
            ans.insert(ans.end(), bucket[i].begin(), bucket[i].begin()+n);
            k -= n;
        }
        
        return ans;
    }
};