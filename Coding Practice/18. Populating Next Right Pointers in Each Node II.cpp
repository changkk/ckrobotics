/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* left;
    Node* right;
    Node* next;

    Node() : val(0), left(NULL), right(NULL), next(NULL) {}

    Node(int _val) : val(_val), left(NULL), right(NULL), next(NULL) {}

    Node(int _val, Node* _left, Node* _right, Node* _next)
        : val(_val), left(_left), right(_right), next(_next) {}
};


    input: root
    
    function
    
    if(root->left)
        root->left->next = root->right        
    
    if(!root->right)
        if(!root->next == nullptr)
            root->left->next = root->next->left or right
        else root->left->next = nullptr;
    else
        if(!root->next == nullptr)
            root->right->next = root->next->left or right
        else root->right->next = nullptr;
            
    O(n) time O(1) space

*/

class Solution {
public:
    void helper(Node* root){
        
        if(root->left && root->right)
            root->left->next = root->right;
        
        if(root->left && !root->right)
        {
            Node* tmp = root->next;
            while(tmp)
            {
                if(tmp->left || tmp->right) break; 
                tmp = tmp->next;
            }
                
            if(tmp)
            {
                if(tmp->left) root->left->next = tmp->left;
                else if(tmp->right) root->left->next = tmp->right;                
            }
            else root->left->next = nullptr;     
        }
        
        if(root->right)
        {
            Node* tmp = root->next;
            while(tmp)
            {
                if(tmp->left || tmp->right) break;                
                tmp = tmp->next;
            }
                
            if(tmp)
            {
                if(tmp->left) root->right->next = tmp->left;
                else if(tmp->right) root->right->next = tmp->right;                
            }
            else root->right->next = nullptr;  
            helper(root->right);
        }
        if(root->left)
            helper(root->left);
             
        
    }
    Node* connect(Node* root) {
        if(!root) return root;
        root -> next = nullptr;
        helper(root);
        
        return root;
    }
};