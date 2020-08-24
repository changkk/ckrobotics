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
    ListNode* sortList(ListNode* head) {
        ListNode* cur = head;
        vector<int> list;
        while(cur)
        {
            list.push_back(cur->val);
            cur = cur->next;
        }
        sort(list.begin(),list.end());
        ListNode newHead(0);
        cur = &newHead;
        
        for(int i=0; i<list.size(); i++)
        {
            cur->next = new ListNode(list[i]);
            cur = cur->next;
        }
        
        return newHead.next;
    }
};