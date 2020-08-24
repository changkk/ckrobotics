/*
    


*/


class Solution {
public:
    int divide(int dividend, int divisor) {

        if(dividend == INT_MIN && divisor == -1) return INT_MAX;
        if(dividend == INT_MAX && divisor == 1) return INT_MAX;

            
        long int a = labs(dividend);
        long int b = labs(divisor);
        
        int i = 0;
        for(i = 0; a - (b<<i) >= 0; i++){}
        
        int res;
        if(i == 0) res = 0;
        else res = 1<<(i-1), a -= b<<(i-1);

        
        
        while(a>=b)
        {    
            a -= b;
            res++;
        }
        
        if(signbit(dividend) != signbit(divisor)) return -res;
        
        return res;
    }
};