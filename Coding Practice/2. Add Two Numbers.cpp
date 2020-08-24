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
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
        ListNode preHead(0);
        ListNode* p = &preHead;
        int extra = 0;
        while(l1 || l2 || extra>0)
        {
            int sum = 0;
            if(l1) sum += l1->val, l1 = l1->next;
            if(l2) sum += l2->val, l2 = l2->next;
            sum += extra;
            
            if(sum>=10){
                sum = sum-10;
                extra = 1;
            }
            else extra = 0;
            
            p->next = new ListNode(sum);
            p = p->next;
        }
        
        return preHead.next;
        
        
    }
    
    
};