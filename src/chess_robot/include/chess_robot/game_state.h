#ifndef GAME_STATE_HPP
#define GAME_STATE_HPP

#include <string>
#include <array>
#include <unordered_map>

enum class Tile: int{

    A8=0,B8,C8,D8,E8,F8,G8,H8,
    A7,B7,C7,D7,E7,F7,G7,H7,
    A6,B6,C6,D6,E6,F6,G6,H6,
    A5,B5,C5,D5,E5,F5,G5,H5,
    A4,B4,C4,D4,E4,F4,G4,H4,
    A3,B3,C3,D3,E3,F3,G3,H3,
    A2,B2,C2,D2,E2,F2,G2,H2,
    A1,B1,C1,D1,E1,F1,G1,H1,
    OFF_BOARD
};

enum Color {
    BLACK,
    WHITE
};

enum PieceType {
    NONE = 0,
    KING,
    QUEEN,
    ROOK,
    BISHOP,
    KNIGHT,
    PAWN
};

enum MoveType{
    NORMAL,
    CAPTURE,
    EN_PASSANT_CAPTURE,
    CASTLE,
    PROMOTION
};

struct Piece {
    Color color;
    PieceType type;
    bool moved = false;
    char FENsymbol = '-';

    Piece(Color color, PieceType type, char ch) : color(color), type(type), moved(false), FENsymbol(ch) {}
    Piece() : color(WHITE), type(NONE), moved(false), FENsymbol('-') {}
    Piece(char piece){

        static const std::unordered_map<char, PieceType> pieceMap = {
            {'p', PAWN}, {'n', KNIGHT}, {'b', BISHOP}, 
            {'r', ROOK}, {'q', QUEEN},  {'k', KING}
        };

        type = pieceMap.at(std::tolower(piece));
        color = std::isupper(piece) ? WHITE : BLACK;
        moved  = false;
        FENsymbol = piece;

    }

    void captured(){
        type = NONE;
    }

};

struct FEN{
    std::string pos;
    std::string activeColor;
    std::string castling;
    std::string enPassant;
    int halfMove;
    int fullMove;
};

class GameState {
private:
    std::array<Piece, 64> board;

    // Internal mapping helpers
    int tileToIndex(Tile tile) { return static_cast<int>(tile); }
    Tile indexToTile(int i) { return static_cast<Tile>(i); }

public:
    FEN fen;

    GameState();
    
    void reset();
    
    // Board access
    void setPiece(Tile tile, Piece piece);
    Piece getPieceAtTile(Tile tile);
    Piece getPieceAtIndex(int idx);
    std::string getBoard();
    
    // Logic & FEN Processing
    std::string tileToAlgebraic(Tile t);
    Tile AlgebraicToTile(std::string alg);
    std::string boardToFEN();
    void FENtoBoard(std::string fen_pos);
    std::string buildFEN();
    
    // Move Handling
    void applyMove(Tile fromTile, Tile toTile);
    void move(Tile fromTile, Tile toTile, char promotion = ' ');
    std::string updateCastling();
    void detectMove(std::string newPos);
    MoveType getMoveType(Tile fromTile, Tile toTile);

};


#endif // GAME_STATE_HPP