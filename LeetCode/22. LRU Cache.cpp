class LRUCache {
    
private:
        int size;
        list<int> lru; // The age order of the keys: lru.back() is the oldeset
        unordered_map<int,int> kv;  // Key - value
        unordered_map<int,list<int>::iterator> ki;  // key - iterator
public:
    LRUCache(int capacity) : size(capacity) {}
    
    int get(int key) {
        if(kv.count(key)==0)
            return -1;
        update_lru(key);
        return kv[key];
    }
    
    void put(int key, int value) {
        if(kv.size() == size && kv.count(key)==0)
        {
            kv.erase(lru.back()); // Erase the oldest key with value
            ki.erase(lru.back()); // Erase the oldest key with iterator
            lru.pop_back(); // Erase the oldest key from the age order
        }
        update_lru(key);
        kv[key] = value; // **kv.insert(make_pair(key,value)); does not work for the same key.
        
    }
    
    void update_lru(int key)
    {
        if(kv.count(key))
            lru.erase(ki[key]); // Delete from the list
        lru.push_front(key);  // Add to the list for the newest one.
        ki[key] = lru.begin(); // ***If I push new begin, then previous value is changed to 2,3,4...
    }
    
};

/**
 * Your LRUCache object will be instantiated and called as such:
 * LRUCache* obj = new LRUCache(capacity);
 * int param_1 = obj->get(key);
 * obj->put(key,value);
 */