/*
    number -> combinations of letter
    numbers: 2 - 9
    3^n? DFS?
    
    [2 3 4 5 6 7 8 9]
    [0 - 0 - 0 - - -]
    
    for(size)
    word += (umap[i][num[i]]);
    
    res.push_back(word);
    
    num[end]++;
    
    for(size)
        if(num[i]>4) num[i-1]++
        
    if num[0] == 3 return res;
    
    umap<int,string> umap = {{2,abc},{3,def},{4,ghi}...};
    --string bfs (num)--
    if(!num)
    for(i)
        st[i] + bfs()
    
    



*/


class Solution {
public:
    vector<string> letterCombinations(string digits) {
        unordered_map<char,string> umap({{'2',"abc"},{'3',"def"},{'4',"ghi"},{'5',"jkl"},{'6',"mno"},{'7',"pqrs"},{'8',"tuv"},{'9',"wxyz"}});
        
        vector<string> tmp;
        vector<string> tmp2;

        if(digits.length() < 1) return tmp;
        
        tmp.push_back("");  // 234
        for(int i = 0; i < digits.length(); i++) // 3 - def
        {
            for(int j = 0; j < tmp.size(); j++) // 3
            {
                for(int k = 0; k < umap[digits[i]].length(); k++) // ad ae af bd be bf cd ce cf
                    tmp2.push_back(tmp[j]+umap[digits[i]][k]); // d e f                   
            }
            tmp2.swap(tmp);
            tmp2.clear();
        }
        
        
        return tmp;
    }
};