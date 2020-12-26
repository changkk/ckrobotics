class Solution {
public:
    bool closeStrings(string word1, string word2) {
        if(word1.length() != word2.length()) return false;
        vector<int> a(27);
        vector<int> b(27);

        for(int i = 0; i < word1.length(); i++){
            a[word1[i]-'a']++;
            b[word2[i]-'a']++;
        }
        
        for(int i = 0; i < 27; i++)
            if(min(a[i], b[i]) ==0 && max(a[i],b[i]) != 0) return false;
        
        sort(a.begin(),a.end());
        sort(b.begin(),b.end());
        
        return a==b;
    }
};