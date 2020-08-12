/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 
    
    6 5 3 1 8 7 2 4
    
    snd->val > cur->val
    
    cur->next = snd->next
    snd->next = cur
    
    5 6 3 1 8 7 2 4
    
    cur = cur->next
    snd = cur->next
    
    if snd->val < cur->val
    do the same thing for()
    

    5 6 3 1 8 7 2 4
    3 5 6 
    1 3 5 6 8 7
    
    1 3 5 6 8

class Solution {
public:
    static bool helper(ListNode* a, ListNode* b)
    {
        return a->val<b->val;
    }
    ListNode* insertionSortList(ListNode* head) {
        
        if(!head) return head;
        
        vector<ListNode*> list;
        ListNode* cur = head;
        while(cur)
        {
            list.push_back(cur);
            cur = cur->next;
        }
        
        sort(list.begin(),list.end(),helper);
        
        for(int i = 0; i < list.size()-1; i++)
        {
            list[i]->next = list[i+1];
        }
        
        list.back()->next = nullptr;
        
        return list[0];
    }
};


 
 */

class Solution {
public:
    ListNode* insertionSortList(ListNode* head){
        ListNode* cur = head;
        ListNode* preHead = new ListNode(0);
        ListNode* first = preHead;
        ListNode* next;
        

        while(cur) // cur = 2 first 4
        {
            next = cur->next;
            
            while(first->next && first->next->val < cur->val) // 4 < 2
                first = first->next;
            
            cur->next = first->next;
            first->next = cur;
            first = preHead;
            cur = next;            
        }
        
        return preHead->next;
        
        
    }
};