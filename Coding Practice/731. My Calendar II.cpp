/*
    [10 50 10 25 
    [20 60 40 55
    
    [10 
    [20
    
    
    [0 10 20 30 40 50 60]
        [  ]
        [        ]
                   [    ]
        [         ]


*/


class MyCalendarTwo {
public:
    map<int,int> mp;
    MyCalendarTwo() {
        
    }
    
    bool book(int start, int end) {
        mp[start]++, mp[end]--;
        
        int cnt = 0;
        for(auto it:mp){
            cnt += it.second;
            if(cnt == 3){
                mp[start]--, mp[end]++;
                return false;
            } 
        }
        return true;
    }
};

/**
 * Your MyCalendarTwo object will be instantiated and called as such:
 * MyCalendarTwo* obj = new MyCalendarTwo();
 * bool param_1 = obj->book(start,end);
 */