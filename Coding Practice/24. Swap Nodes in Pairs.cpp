/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    ListNode* swapPairs(ListNode* head) {
        // Should include edge cased like NULL
        if(head && head->next)        
        {
            if(head->next->next)
            {
                swapPairs(head->next->next);
            }
            swap(head->val,head->next->val);        
        }    
        return head;
    }
};