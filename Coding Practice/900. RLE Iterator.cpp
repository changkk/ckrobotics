/*
    [3,8,0,9,2,5]
    q = [8 8 8 5 5]
    
    
    
    front. pop.
    if(size < 0) return -1

        for(int i = 0; i < A.size(); i = i+2){ // This works
            for(int j = 0; j < A[i]; j++)
                q.push(A[i+1]);
        }


*/
class RLEIterator {
public:
    vector<int> q;
    int idx;
    
    RLEIterator(vector<int>& A) {
        
        q = A; //???????????
        idx = 0;
    }
    
    int next(int n) {  //???????????
        
        while(idx<q.size()){
            
            if(q[idx] >= n){
                q[idx] -= n;
                return q[idx+1]; // ??????????????????
            }
            else{
                n -= q[idx];
                idx += 2;
            }
        }
        return -1;
        
    }
};

/**
 * Your RLEIterator object will be instantiated and called as such:
 * RLEIterator* obj = new RLEIterator(A);
 * int param_1 = obj->next(n);
 */