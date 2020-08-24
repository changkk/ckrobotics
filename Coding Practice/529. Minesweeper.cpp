/*
    If clicked position has a mine around it, it will show the number of mine
    If not, it will unreveal all the cells around it. This will keep going.
    
    1. If clicked, we need to check if there is a mine in adjecent cells.
        - There is mines
            How many mines?
            - Check left right up down 4 diangoals
        - There is no mine
            - change the cell to 'B'
            - Go recursively.
    
    O(9 m*n) time, O(1)
    

*/


class Solution {
public:
    vector<vector<int>> dir = {{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1}};

    int numberOfMines(vector<vector<char>>& board, int i, int j){ // Large mistake: bool -> int
        int count = 0;

        for(int k = 0; k < 8; k++)
        {
            int x = i+dir[k][0];
            int y = j+dir[k][1];

            if(x < 0 || x >= board.size() || y < 0 || y >= board[0].size()) continue;

            if(board[x][y] == 'M') count++;
        }
        return count;
    }
    void changeBoard(vector<vector<char>>& board, vector<int>& click){ // Big mistake: go again to visited position
        int m = board.size();
        int n = board[0].size(); // mistake: size() -> size

        int numMine = numberOfMines(board,click[0],click[1]);
        
        if(numMine > 0) board[click[0]][click[1]] = numMine + '0'; // Mistake : forgot to change to click from i and j
        else
        {
            board[click[0]][click[1]] = 'B';
            for(int k = 0; k < 8; k++)
            {
                int x = click[0]+dir[k][0];
                int y = click[1]+dir[k][1];

                if(x < 0 || x >= board.size() || y < 0 || y >= board[0].size() || (board[x][y] != 'E' && board[x][y] != 'M')) continue; // Mistake: Shouldn't be same as the board size.
                vector<int> tmp = {x,y};
                changeBoard(board,tmp); // Mistake: tmp -> {x,y}
            }
        }
    }
    
    vector<vector<char>> updateBoard(vector<vector<char>>& board, vector<int>& click) {
        if(board[click[0]][click[1]] == 'M')
        {
            board[click[0]][click[1]] = 'X';
            return board;
        }
        
        changeBoard(board, click);
        return board;
    }
};