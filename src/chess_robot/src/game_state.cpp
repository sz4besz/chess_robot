
#include "game_state.h"

#include <array>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <unordered_map>
#include <utility>
#include <string>
#include <iostream>
#include <sstream>

#include "chess_robot/engine_client.h"

using std::string;

static string startingPos = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

// Game state handling 


GameState::GameState(){
    reset();
}

void GameState::reset(){
    FENtoBoard(startingPos);
    fen.pos = startingPos;
    fen.activeColor = 'w';
    fen.castling = updateCastling();
    fen.enPassant = "-";
    fen.halfMove = 0;
    fen.fullMove = 1;
}

// Getters and setters
void GameState::setPiece(Tile tile, Piece piece){
    int idx = tileToIndex(tile);
    if (idx < 0 || idx >= 64) return;
    board[idx] = piece;
}

Piece GameState::getPieceAtTile(Tile tile){
    int idx = tileToIndex(tile);
    if (idx < 0 || idx >= 64) return Piece();
    return board[idx];
}

Piece GameState::getPieceAtIndex(int idx){
    if (idx < 0 || idx >= 64) return Piece();
    return board[idx];
} 

// Algebraic notation builder
string GameState::tileToAlgebraic(Tile t){
    int idx = static_cast<int>(t);
    // 0 is a8, so we start the count at 'a' and '8'
    char file = ('a' + idx % 8);
    char rank = ('8' - idx/8);
    return string(1, file) + string(1, rank);
}

Tile GameState::AlgebraicToTile(string alg){

    // (8 - alg[1]): rank
    // rank * 8: starting tile of rank
    // alg[0] - 'a': file
    // Added up they are a number 0-63, corresponding to 
    // the tiles idx
    return indexToTile(( 8 - alg[1] ) * 8 + alg[0] - 'a');

}


// FEN possition builder
string GameState::boardToFEN(){
    string fen = "" ;
    for(int i =0; i<64; i++){
        fen += getPieceAtIndex(i).FENsymbol;
        if(i % 8 == 7 && i != 63) fen += "/";
    }

    string numberedFen = "";
    int cnt = 0;
    for(char c: fen){
        if(c=='-'){
            cnt++;
        }
        else{
            if(cnt > 0){
                numberedFen += std::to_string(cnt);
                cnt=0;
            }
            numberedFen += c;
        }
    }
    if (cnt > 0) numberedFen += std::to_string(cnt);

    return numberedFen;
    
}

// Set board
void GameState::FENtoBoard(string fen){

    for(int i = 0; i<64; i++){
        setPiece(indexToTile(i),Piece());
    }

    int idx = 0;
    for(char c: fen){
        if(c == '/') continue;
        else if (std::isdigit(c)){
            // Skip empty tiles
            idx+= (c - '0');
        }
        else{
            setPiece(indexToTile(idx),Piece(c));
            idx++;
        }
    }
}

void GameState::applyMove(Tile fromTile, Tile toTile){
    Piece fromPiece = getPieceAtTile(fromTile);
    fromPiece.moved = true;
    setPiece(toTile, fromPiece);
    setPiece(fromTile, Piece());
}

void GameState::move(Tile fromTile, Tile toTile, char promotion){
    
    Piece fromPiece = getPieceAtTile(fromTile);
    Piece toPiece = getPieceAtTile(toTile);
    int diff = static_cast<int>(fromTile) - static_cast<int>(toTile);
    string ep = "-";

    // Clock logic
    // Half clock: resets after pawn move or capture
    if(fromPiece.type == PAWN || toPiece.type != NONE){
        fen.halfMove = 0;
    } else{
        fen.halfMove++;
    }

    // Full clock: increments after blacks turn
    if(fen.activeColor == "b") fen.fullMove++; 



    // If move was a castle: the king has moved 2 tiles
    // Queen side castle: diff is 2
    if(fromPiece.type == KING && diff == 2){
        // king moves to C1 or C8 (White/Black)
        applyMove(fromTile, toTile);

        // rook moves to D1 or D8 (White/Black)
        Tile rookFrom = fromPiece.color == WHITE ? Tile::A1 : Tile::A8;
        Tile rookTo = fromPiece.color == WHITE ? Tile::D1 : Tile::D8;

        applyMove(rookFrom, rookTo);
    }

    // Queen side castle: diff is -2
    else if(fromPiece.type == KING && diff == -2){
        // king moves to G1 or G8 (White/Black)
        applyMove(fromTile, toTile);

        // rook moves to F1 or F8 (White/Black)
        Tile rookFrom = fromPiece.color == WHITE ? Tile::H1 : Tile::H8;
        Tile rookTo = fromPiece.color == WHITE ? Tile::F1 : Tile::F8;

        applyMove(rookFrom, rookTo);
    }
    //  En passant: diff is +-16 (two rows of 8)
    else if(fromPiece.type==PAWN && std::abs(diff) == 16){
        applyMove(fromTile, toTile);
        // set the en passsant flag, its behind the pawn (1 row of 8)
        int epIndex = static_cast<int>(fromTile) + (fromPiece.color == WHITE ? -8: 8);
        ep = tileToAlgebraic(indexToTile(epIndex));
    }
    // If pawn captures with en passsant, the other pawn isnt removed aautomatically 
    // En passant when pawn moves diagonally to an empty tile
    else if(fromPiece.type == PAWN && diff % 8 != 0 && getPieceAtTile(toTile).type == NONE){
        applyMove(fromTile, toTile);
        int captured = static_cast<int>(toTile) + (fromPiece.color == WHITE ? 8 : -8);
        setPiece(indexToTile(captured), Piece());
    }
    // Normal move
    else{ applyMove(fromTile,toTile); }

    // Promotion handling
    Piece pawn = getPieceAtTile(toTile);
    if(pawn.type == PAWN){
        int idx = static_cast<int>(toTile);
        if(idx <= 7 || idx >=56){
            Color c = pawn.color;
            if(promotion == ' ') promotion = 'q';
            promotion = c == WHITE ? std::toupper(promotion): promotion;
            Piece promoted(promotion);
            promoted.moved =true;
            setPiece(toTile, promoted);
        }
    }

    fen.activeColor = fen.activeColor == "w" ? "b": "w"; // Toggle
    fen.castling = updateCastling();
    fen.pos = boardToFEN();
    fen.enPassant = ep;
    
}

MoveType GameState::getMoveType(Tile fromTile, Tile toTile){
    
    Piece fromPiece = getPieceAtTile(fromTile);
    Piece toPiece = getPieceAtTile(toTile);
    int diff = static_cast<int>(fromTile) - static_cast<int>(toTile);
    Piece pawn = getPieceAtTile(toTile);
    
    // If move was a castle: the king has moved 2 tiles
    // Queen side castle: diff is 2

    if(fromPiece.type == KING && abs(diff) == 2){
        return CASTLE;
        
    }

    // If pawn captures with en passsant, the other pawn isnt removed aautomatically 
    // En passant when pawn moves diagonally to an empty tile
    else if(fromPiece.type == PAWN && diff % 8 != 0 && getPieceAtTile(toTile).type == NONE){
        return EN_PASSANT_CAPTURE;
    }

    // Promotion is when a pawn stands on the first or last rank
    else if(pawn.type == PAWN){
        int idx = static_cast<int>(toTile);
        if(idx <= 7 || idx >=56){
            return PROMOTION;
        }
    }

    // Normal move is when a piece moves to an empty tile
    else{
        return NORMAL;
    }
    
}

string GameState::updateCastling(){
    string rights = "";

    // Tiles the kings and rooks start on
    // Get the pieces that stand on them
    Piece wKing = getPieceAtTile(Tile::E1);
    Piece bKing = getPieceAtTile(Tile::E8);
    // white/black king/queen rooks
    Piece wkRook = getPieceAtTile(Tile::H1);
    Piece wqRook = getPieceAtTile(Tile::A1);
    Piece bkRook = getPieceAtTile(Tile::H8);
    Piece bqRook = getPieceAtTile(Tile::A8);
    
    // Checks if rooks havent been captured or moved, and if king hasnt moved
    // Rights are added in this strict order
    if(wKing.color == WHITE && wKing.type == KING && !wKing.moved)
    {
        if(wkRook.color==WHITE && wkRook.type == ROOK && !wkRook.moved) rights += "K";
        if(wqRook.color==WHITE && wqRook.type == ROOK && !wqRook.moved) rights += "Q";
    }
    if(bKing.color == BLACK && bKing.type == KING && !bKing.moved)
    {
        if(bkRook.color==BLACK && bkRook.type == ROOK && !bkRook.moved) rights += "k";
        if(bqRook.color==BLACK && bqRook.type == ROOK && !bqRook.moved) rights += "q";
    }
    
    // return "-" if no rights remain
    return rights == "" ? "-" : rights;

}

void GameState::detectMove(string newPos){
    
    GameState detected;
    detected.FENtoBoard(newPos);
    
    Tile from = Tile::OFF_BOARD;
    Tile to = Tile::OFF_BOARD;

    char promotion = ' ';

    for(int i = 0; i<64; i++){

        Piece oldp = this->getPieceAtIndex(i);
        Piece newp = detected.getPieceAtIndex(i);
    
        // Look for changes
        // From tile: was a piece, now NONE
        // To tile: was a black piece or empty, now WHITE
        if(oldp.FENsymbol != newp.FENsymbol){
            
            // From tile
            if(oldp.color == WHITE && oldp.type != NONE){
                // Prioritize Kings over Rooks for better castling handling
                // During castles, there are two from and to tiles
                // if from is off board, this is the first we found
                if(oldp.type == KING || from == Tile::OFF_BOARD ){
                    from = indexToTile(i);
                }
            }

            // To tile
            if(newp.color == WHITE && newp.type != NONE){
                // Same logic, during castless the primary mover is the king
                if(newp.type == KING || to == Tile::OFF_BOARD){
                    to = indexToTile(i);
                    promotion = newp.FENsymbol;
                }
            }
        }
    }

    move(from,to, promotion);

}

string GameState::buildFEN(){
        return fen.pos + " " + fen.activeColor + " " + fen.castling + " " 
        + fen.enPassant + " " + std::to_string(fen.halfMove) + " " + std::to_string(fen.fullMove);
    }

std::string GameState::getBoard(){
    return this->fen.pos;
}




void runTests() {
    GameState gs;
    int passed = 0;
    int total = 0;

    auto assertTest = [&](string title, string expectedBoard, string expectedFull) {
        total++;
        string actualBoard = gs.boardToFEN();
        string actualFull = gs.buildFEN();
        bool boardMatch = (actualBoard == expectedBoard);
        bool fullMatch = (actualFull == expectedFull);

        std::cout << "[" << (boardMatch && fullMatch ? "PASS" : "FAIL") << "] " << title << "\n";
        if (!boardMatch) {
            std::cout << "   Board Mismatch!\n   Exp: " << expectedBoard << "\n   Act: " << actualBoard << "\n";
        }
        if (!fullMatch) {
            std::cout << "   Full FEN Mismatch!\n   Exp: " << expectedFull << "\n   Act: " << actualFull << "\n";
        }
        if (boardMatch && fullMatch) passed++;
    };

    std::cout << "====================================================\n";
    std::cout << "                CHESS LOGIC VERIFICATION            \n";
    std::cout << "====================================================\n\n";

    // --- 1. STARTING POSITION ---
    gs.reset();
    assertTest("01: Starting Position", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");

    // --- 2. WHITE PAWN DOUBLE PUSH (EP TARGET) ---
    gs.reset();
    gs.move(Tile::E2, Tile::E4);
    assertTest("02: White Pawn e2-e4", 
               "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR", 
               "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1");

    // --- 3. BLACK PAWN DOUBLE PUSH (EP TARGET) ---
    // Manually setting turn to black before moving
    gs.fen.activeColor = "b"; 
    gs.move(Tile::D7, Tile::D5);
    assertTest("03: Black Pawn d7-d5", 
               "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR", 
               "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2");

    // --- 4. STANDARD CAPTURE & CLOCK RESET ---
    gs.move(Tile::E4, Tile::D5);
    assertTest("04: Capture e4xd5", 
               "rnbqkbnr/ppp1pppp/8/3P4/8/8/PPPP1PPP/RNBQKBNR", 
               "rnbqkbnr/ppp1pppp/8/3P4/8/8/PPPP1PPP/RNBQKBNR b KQkq - 0 2");

    // --- 5. WHITE KINGSIDE CASTLE ---
    gs.reset();
    // Clear path and ensure rights are fresh
    gs.setPiece(Tile::F1, Piece()); gs.setPiece(Tile::G1, Piece());
    gs.move(Tile::E1, Tile::G1);
    assertTest("05: White Kingside Castle", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQ1RK1", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQ1RK1 b kq - 1 1");

    // --- 6. WHITE QUEENSIDE CASTLE ---
    gs.reset();
    gs.setPiece(Tile::B1, Piece()); gs.setPiece(Tile::C1, Piece()); gs.setPiece(Tile::D1, Piece());
    gs.move(Tile::E1, Tile::C1);
    assertTest("06: White Queenside Castle", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/2KR1BNR", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/2KR1BNR b kq - 1 1");

    // --- 7. BLACK KINGSIDE CASTLE ---
    gs.reset();
    gs.setPiece(Tile::F8, Piece()); gs.setPiece(Tile::G8, Piece());
    gs.fen.activeColor = "b";
    gs.move(Tile::E8, Tile::G8);
    assertTest("07: Black Kingside Castle", 
               "rnbq1rk1/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR", 
               "rnbq1rk1/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQ - 1 2");

    // --- 8. WHITE EN PASSANT EXECUTION ---
    gs.reset();
    gs.FENtoBoard("8/8/8/3pP3/8/8/8/8");
    gs.fen.activeColor = "w";
    gs.fen.enPassant = "d6";
    gs.move(Tile::E5, Tile::D6);
    assertTest("08: White EP Capture e5xd6", 
               "8/8/3P4/8/8/8/8/8", 
               "8/8/3P4/8/8/8/8/8 b - - 0 1");

    // --- 9. BLACK EN PASSANT EXECUTION ---
    gs.reset();
    gs.FENtoBoard("8/8/8/8/4Pp2/8/8/8");
    gs.fen.activeColor = "b";
    gs.fen.enPassant = "e3";
    gs.move(Tile::F4, Tile::E3);
    assertTest("09: Black EP Capture f4xe3", 
               "8/8/8/8/8/4p3/8/8", 
               "8/8/8/8/8/4p3/8/8 w - - 0 2");

    // --- 10. WHITE PROMOTION (QUEEN) ---
    gs.reset();
    gs.FENtoBoard("8/P7/8/8/8/8/8/8");
    gs.fen.activeColor = "w";
    gs.move(Tile::A7, Tile::A8, 'q');
    assertTest("10: White Promotion a7-a8=Q", 
               "Q7/8/8/8/8/8/8/8", 
               "Q7/8/8/8/8/8/8/8 b - - 0 1");

    // --- 11. BLACK PROMOTION (KNIGHT) ---
    gs.reset();
    gs.FENtoBoard("8/8/8/8/8/8/7p/8");
    gs.fen.activeColor = "b";
    gs.move(Tile::H2, Tile::H1, 'n');
    assertTest("11: Black Promotion h2-h1=n", 
               "8/8/8/8/8/8/8/7n", 
               "8/8/8/8/8/8/8/7n w - - 0 2");

    // --- 12. RIGHTS LOSS: KING MOVE ---
    gs.reset();
    gs.move(Tile::E1, Tile::E2);
    // Note: FEN rank 7 (white) is index 48-55. King moved from 60 to 52. 
    // Starting Rank 2 (index 48-55) is PPPPPPPP. At 52 (E2) it's now PPPPKPPP.
    assertTest("12: King Move (Rights Loss)", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPKPPP/RNBQ1BNR", 
               "rnbqkbnr/pppppppp/8/8/8/8/PPPPKPPP/RNBQ1BNR b kq - 0 1");

    // --- 13. RIGHTS LOSS: ROOK CAPTURED ---
    gs.reset();
    gs.FENtoBoard("r3k2r/8/8/8/8/8/8/R3K2R");
    gs.fen.castling = "KQkq";
    gs.setPiece(Tile::D4, Piece('B')); // Add a white bishop
    gs.fen.activeColor = "w";
    gs.move(Tile::D4, Tile::A8); // Bishop takes Black Rook on a8
    // Resulting castling: White can castle (KQ), Black lost Queenside (q) but kept Kingside (k)
    assertTest("13: Rook Captured (Rights Loss)", 
               "B3k2r/8/8/8/8/8/8/R3K2R", 
               "B3k2r/8/8/8/8/8/8/R3K2R b KQk - 0 1");

    std::cout << "\n====================================================\n";
    std::cout << " FINAL VERDICT: " << passed << "/" << total << " PASSED\n";
    std::cout << "====================================================\n";

    }

