struct Tree {
    int val;
    Tree *next, *prev;
    Tree(){
        val = 0;
        next = nullptr;
        prev = nullptr;
    }
    Tree(int k){
        val = k;
        next = nullptr;
        prev = nullptr;
    }
};

class MyCircularDeque {
public:
    Tree* front = nullptr;
    Tree* last = nullptr;
    int ctnSize;
    int curSize = 0;
    /** Initialize your data structure here. Set the size of the deque to be k. */
    MyCircularDeque(int k) {
        ctnSize = k;
    }
    
    /** Adds an item at the front of Deque. Return true if the operation is successful. */
    bool insertFront(int value) {
        if(curSize >= ctnSize) return false;
        if(!front){
            front = new Tree(value);
            last = front;
            curSize++;
            return true;
        }
        front->prev = new Tree(value);
        Tree* tmpNext = front;
        front = front->prev;
        front->next = tmpNext;
        curSize++;
        return true;
    }
    
    /** Adds an item at the rear of Deque. Return true if the operation is successful. */
    bool insertLast(int value) {
        if(curSize >= ctnSize) return false;
        if(!front){
            front = new Tree(value);
            last = front;
            curSize++;
            return true;
        }
        last->next = new Tree(value);
        Tree* tmpPrev = last;
        last = last->next;
        last->prev = tmpPrev;
        curSize++;
        return true;
    }
    
    /** Deletes an item from the front of Deque. Return true if the operation is successful. */
    bool deleteFront() {
        if(curSize == 0) return false;
        if(!front->next){
            curSize--;
            front = nullptr;
            last = nullptr;
            return true;
        }
        Tree* tmpNext = front->next;
        tmpNext->prev = nullptr;
        front->next = nullptr;
        curSize--;
        front = tmpNext;
        return true;
    }
    
    /** Deletes an item from the rear of Deque. Return true if the operation is successful. */
    bool deleteLast() {
        if(curSize == 0) return false;
        if(!last->prev){
            curSize--;
            front = nullptr;
            last = nullptr;
            return true;
        }
        Tree* tmpPrev = last->prev;
        tmpPrev->next = nullptr;
        last->prev = nullptr;
        curSize--;
        last = tmpPrev;
        return true;
    }
    
    /** Get the front item from the deque. */
    int getFront() {
        return front? front->val : -1;
    }
    
    /** Get the last item from the deque. */
    int getRear() {
        return last? last->val : -1;
    }
    
    /** Checks whether the circular deque is empty or not. */
    bool isEmpty() {
        return curSize == 0;
    }
    
    /** Checks whether the circular deque is full or not. */
    bool isFull() {
        return curSize == ctnSize;
    }
};

/**
 * Your MyCircularDeque object will be instantiated and called as such:
 * MyCircularDeque* obj = new MyCircularDeque(k);
 * bool param_1 = obj->insertFront(value);
 * bool param_2 = obj->insertLast(value);
 * bool param_3 = obj->deleteFront();
 * bool param_4 = obj->deleteLast();
 * int param_5 = obj->getFront();
 * int param_6 = obj->getRear();
 * bool param_7 = obj->isEmpty();
 * bool param_8 = obj->isFull();
 */