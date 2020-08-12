/*
    backtracking
    ((()))
    ( -> remove from the candidate
    (( -> remove from the candidate
    vector<int> candi
    candi[0] = 3
    candi[1] = 3
    
    (
    )
    
    ((
    ()

    (((
    ()(
    (()
    
    ()((
    
    
    
    

*/

class Solution {
public:
    vector<string> generateParenthesis(int n) {
        
        unordered_map<string,vector<int>> tmp;
        tmp.insert({"",{0,0}}); // vec[0] => '(' vec[1] => ')'
        vector<string> tmpVec;

        for(int i = 0; i < 2*n; i++) // ["",0,0]
        {
            tmpVec.clear();
            unordered_map<string,vector<int>> tmp2;
            for(auto it:tmp) // ()(( 31 (()( 31 ((() 31 ()() 22 (()) 22
            {
                if(it.second[0]+1 <= n && it.second[0]-it.second[1]>=0) 
                {
                    tmp2.insert({it.first+'(',{it.second[0]+1,it.second[1]}}); // ()()( 32 (())( 32
                    tmpVec.push_back(it.first+'(');              
                }
                if(it.second[1]+1 <= n && it.second[0]-it.second[1]>0)
                {
                    tmp2.insert({it.first+')',{it.second[0],it.second[1]+1}}); // ()(() 32 (()() 32 ((()) 32
                    tmpVec.push_back(it.first+')');              
                }
            }
            
            tmp.swap(tmp2); //
        }
        
        return tmpVec;
        
    }
};