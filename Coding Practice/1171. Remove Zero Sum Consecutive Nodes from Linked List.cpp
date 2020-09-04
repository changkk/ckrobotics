/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 
    1 2 -3 3 1
    1 3 0  3 4
 
 
    1 2 3 -3 -2
    1 3 6 3 1 ->next

    
    1. Compute the sum of number seen (1 loop) 
    2. From the first sum, save it to stack and umap
    3. If I encounter 0, connect it directly to the head, clear umap
    4. If I encounter same number in umap, then pop stack until the same number
    5. Also, delete the number in the map
    6. And connect the st.top() -> cur->next
    
 */
class Solution {
public:
    ListNode* removeZeroSumSublists(ListNode* head) {
        if(!head) return nullptr;
        ListNode* cur = head;
        int sum = 0;
        unordered_map<int,ListNode*> umapSum;
        deque<int> dqSum;
        // 1 2 -3 3 1
        // 1 3 0  3 4
        
        while(cur)
        {
            sum += cur->val; // sum = 4
            if(sum == 0)
            {
                // Empty everything
                umapSum.clear(); // []
                dqSum.clear(); // []
            }
            else if(umapSum.find(sum) != umapSum.end())
            {
                // stack = [1] // Map = [1, node]
                while(dqSum.back() != sum)
                {
                    umapSum.erase(dqSum.back()); //Big mistake 1 // This should be done first
                    dqSum.pop_back();
                }

                umapSum[dqSum.back()]->next = cur->next;
            }
            else
            {
                umapSum.insert(make_pair(sum,cur)); // [3, 3] [4 1]
                dqSum.push_back(sum); // [3 4]

            }
            cur = cur->next;
        }
        
        if(dqSum.empty()) return nullptr; // Mistake 2
        
        return umapSum[dqSum.front()]; //
        
        
    }
};