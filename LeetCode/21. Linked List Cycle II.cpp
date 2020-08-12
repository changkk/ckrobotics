/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 
    Find a tail
    
    We can traverse all node and check the vistied node
    And if we come back to the visited node, then that's going to be the cycle?

    slow pointer and fast pointer
    
      v
    1 2 3 4 5 
    - - - - -
      -   - 
    -   -   -
    
    slow = i
    fast = 2*i
    
    cycle = i
 
 */
class Solution {
public:
    ListNode *detectCycle(ListNode *head) {
        ListNode* slow;
        ListNode* fast;
        
        if(head)
        {
            slow = head->next;
            if(head->next)
                fast = head->next->next;            
        }    
                
        while(slow != fast && slow && fast)
        {
            slow = slow -> next;
            if(fast->next)
                fast = fast -> next -> next;
            else fast = nullptr;
        }
        
        if(!slow || !fast) return nullptr;
        
        ListNode* entry = head;
        
        while(entry != slow)
        {
            slow = slow -> next;
            entry = entry -> next;
        }
        
        return entry;
        
    }
};