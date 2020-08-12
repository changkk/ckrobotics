/*
    Find word not " "
    Start from the back of the string
    if we find a complete word (before next " ")
    push it to string
    and add " "
    and repeat

    O(n) time O(1) space


*/


class Solution {
public:
    string reverseWords(string s) {
        
        string res;
        
        if(s.length()==0) return res;
        
        int i = s.length()-1;

        while(i>=0)
        {

            while(s[i]==' ' && i>=0) 
            {
                i--;
                if(i<0) break;
             
            }

            if(i<0) return res;
            
            int endChar = i; // o
            
            while(s[i]!=' ' && i>=0) 
            {
                i--;
                if(i<0) break;      
            }

            if(res.length()!=0) res+=' ';
            
            for(int j = i+1; j <=endChar; j++)
                res += s[j]; // world! hello

        }        
     
        return res;
    }
};