/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* next;
    Node* random;
    
    Node(int _val) {
        val = _val;
        next = NULL;
        random = NULL;
    }
};
*/

class Solution {
public:
    Node* copyRandomList(Node* head) {
        Node preHead(0); // For stamp
        Node* p = &preHead;
        Node* head_cp = head;
        
        while(head_cp)
        {
            if(!p->next && head_cp->next) p->next = new Node(head_cp->next->val);
            p = p->next;
            head_cp = head_cp->next;
        }     
        
        p = &preHead;
        head_cp = head;
        
        while(head->random)
        {
            p->random = head->random;
            cout<<head->random->val<<endl;
            p = p->next;
            head = head->next;
        }      
        p = &preHead;
        return p;
        
    }
};