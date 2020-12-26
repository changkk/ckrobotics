/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 
 3 2 0 -4
 BF: push each node so far and check
 
 
 
 */
class Solution {
public:
    bool hasCycle(ListNode *head) {
        if(!head || !head->next) return false;
        ListNode* fast = head->next->next;
        ListNode* slow = head->next;
        
        
        while(1){
            if(fast == slow) return true;
            if(!fast || !fast->next) return false; 
            
            fast = fast->next->next;
            slow = slow->next;
        }
        return false;
        
    }
};