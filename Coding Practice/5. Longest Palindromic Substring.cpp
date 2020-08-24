class Solution {
public:
    string longestPalindrome(string s) {
        
        int i = 0;
        int max_len = 1;
        int start_idx = 0;
        if(s.size()==1) return s;
        
        while(i<s.size()) // For scanning all the idxs
        {
            int l_ptr = i, r_ptr = i;
            
            // Check if there are continuous same letters
            // if there are, move the pointer to the right and check again
            while(s[l_ptr]==s[r_ptr + 1] && r_ptr < s.size()-1) r_ptr++; 
            
            // If finding continuous letter, then go next to idx
            i = r_ptr + 1; 
            
            // Check the both side
            while(l_ptr>0 && s[l_ptr-1] == s[r_ptr+1] && r_ptr < s.size()-1){
                l_ptr--;
                r_ptr++;
            }
            
            // Compute the length between pointer
            int len = r_ptr - l_ptr + 1; 
            
            // If length is larger than max_len, save it.
            if(len > max_len)
            {
                start_idx = l_ptr;
                max_len = len;
            }
            
        }
        
        return s.substr(start_idx, max_len);
        
    }
};