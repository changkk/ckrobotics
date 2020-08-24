/*
    User 1 = [5-1 7-3] 
    User 2 = [6-2 10-4]
    User 3 
    User 4 
    User 5 
    
    
    unordered_map<int,vector<int>> UserFriend = [ [1, [2]] [2, [3]] [3, [5 1]]  ]
    unordered_map<int,vector<pair<int,int>>> UserTweets = [ [1, [1,5][3,7]] [2, [2,3]] [3, [4,5] [6,1]]  ]

    priority_queue<pair<int,int>> pq = [[2,3] [1,5]] - 10
    res.push_back
    return.
    


*/



class Twitter {
public:
    unordered_map<int,unordered_set<int>> UserFriend;
    unordered_map<int,vector<pair<int,int>>> UserTweets; //Mistake
    int time = 0;
    
    /** Initialize your data structure here. */
    Twitter() {
        
    }
    
    /** Compose a new tweet. */
    void postTweet(int userId, int tweetId) {
        time++; // Mistake
        UserTweets[userId].push_back(make_pair(time,tweetId));    
    }
    
    /** Retrieve the 10 most recent tweet ids in the user's news feed. Each item in the news feed must be posted by users who the user followed or by the user herself. Tweets must be ordered from most recent to least recent. */
    vector<int> getNewsFeed(int userId) {
        priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>> pq; // Mistake
        
        for(auto tweets:UserTweets[userId])
        {            
            pq.push(tweets);
            if(pq.size()>10) // Mistake
                pq.pop();
        }
        
        for(auto f:UserFriend[userId])
        {
            if(f == userId) continue;
            for(auto tweets:UserTweets[f])
            {            
                pq.push(tweets);
                if(pq.size()>10)
                    pq.pop();
            }

        }
        
        vector<int> res;

        int pq_size = pq.size(); // Mistake
        for(int i = 0; i < pq_size; i++) // Mistake
        {
            res.push_back(pq.top().second);
            pq.pop();
        }

        reverse(res.begin(),res.end());
        return res;
            
    }
    
    /** Follower follows a followee. If the operation is invalid, it should be a no-op. */
    void follow(int followerId, int followeeId) {
        UserFriend[followerId].insert(followeeId);
    }
    
    /** Follower unfollows a followee. If the operation is invalid, it should be a no-op. */
    void unfollow(int followerId, int followeeId) {
        UserFriend[followerId].erase(followeeId);
    }
};

/**
 * Your Twitter object will be instantiated and called as such:
 * Twitter* obj = new Twitter();
 * obj->postTweet(userId,tweetId);
 * vector<int> param_2 = obj->getNewsFeed(userId);
 * obj->follow(followerId,followeeId);
 * obj->unfollow(followerId,followeeId);
 */	