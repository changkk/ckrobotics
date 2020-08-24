/*
    1/2 -> 0.5
    
    2/1 -> 2
    
    1/3 -> 0.(3)
    
    

*/

class Solution {
public:
    string fractionToDecimal(int numerator, int denominator) {
        
        if(numerator == 0) return "0";
       string res;
 
        // sign check
        if(numerator>0 ^ denominator>0) res += '-';
 
        long n = labs(numerator);
        long d = labs(denominator);
 
        long r = n%d; // 1/2   -> r = 1
               
        res += to_string(n/d); // res = 0.
        if(r != 0)
            res += '.';
        
        // Umap is for saving the locaton of the end of number
        unordered_map<long,long> umap;
        while(r != 0)
        {
            if(umap.find(r) != umap.end()) //
            {
                res.insert(umap[r],"(");
                res += ')';
                return res;
            }
            
            umap[r] = res.length(); //
            r *= 10; // 10
            res += to_string(r/d); // res = 0.5
            r = r%d; // r = 0
        }
        
        
    
        
        return res;
    }
};