/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    ListNode* oddEvenList(ListNode* head) {
        ListNode* oddHead = nullptr;
        ListNode* oddTail = nullptr;
        ListNode* evenTail = nullptr;
        ListNode* evenHead = nullptr;
        ListNode* cur = head;
        
        while(cur)
        {              
            if((cur->val)%2 == 1)
            {

                if(!oddTail) 
                {
                    oddHead = cur; 
                    oddTail = cur; 
                    cur = cur->next;
                }
                else
                {
                    oddTail->next = cur;
                    oddTail = oddTail->next;
                    cur = cur->next;
                } 

            }
            else
            {

                if(!evenTail) 
                {
                    evenHead = cur; 
                    evenTail = cur; 
                    cur = cur->next;
                }
                else
                {
                    evenTail->next = cur;
                    evenTail = evenTail->next;
                    cur = cur->next;    
                }
            }
        }
        
        oddTail->next = evenHead;
        return oddTail;
    }
};