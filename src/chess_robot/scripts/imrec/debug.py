#!/usr/bin/env python3

import os
import torch
import torch.nn as nn
import cv2 as cv
import numpy as np
from torchvision import transforms, models
from PIL import Image

# --- CONFIGURATION ---
BASE_PATH = "/home/mates/onlab/src/chess_robot/scripts/imrec"
IMAGE_DIR = os.path.join(BASE_PATH, "testImages")
RESULTS_DIR = os.path.join(BASE_PATH, "results")
MODEL_PATH = os.path.join(BASE_PATH, "chess_brain.pth")

TEST_IMAGES = ["test1.png", "test2.png", "test3.png"]

EXPECTED_FENS = {
    "test1.png": "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR",        
    "test2.png": "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR",        
    "test3.png": "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR",      
    "test4.png": "rnbqkbnr/pppp1ppp/8/4p3/3PP3/8/PPP2PPP/RNBQKBNR",
    "test5.png": "8/8/rnbqkbnr/pppppppp/PPPPPPPP/RNBQKBNR/8/8",
    "test6.png": "2b1k1n1/rn1q1b1r/p1p1p2P/1pp2pp1/1PPPP1Pp/P2K1P2/R1B2B1R/1N2Q1N1",
    "test7.png": "4k3/rnbq1bnr/p3p3/1ppp1ppP/1PPPPPPp/P2K4/RNB2BNR/4Q3" 
}

class ChessResNet(nn.Module):
    def __init__(self, num_classes=13):
        super(ChessResNet, self).__init__()
        self.model = models.resnet18()
        self.model.fc = nn.Linear(self.model.fc.in_features, num_classes)
    def forward(self, x):
        return self.model(x)

# --- THE GEOMETRIC DEFINITIONS (CLEANED UP) ---

def calculate_tile_centers(corners, ids):
    """
    Computes all 64 tile centers sequentially (A8 down to H1) and scales using rail algebra.
    Returns: (payload_string, scale_h, scale_v) or (None, 0, 0) if verification fails.
    """
    marker_map = {ids[i][0]: np.mean(corners[i][0], axis=0) for i in range(len(ids))}
    try:
        # Markers: 0=A_top, 1=A_bot, 2=H_top, 3=H_bot
        m0, m1, m2, m3 = marker_map[0], marker_map[1], marker_map[2], marker_map[3]
    except KeyError:
        return None, 0, 0

    tile_centers = []
    for r in range(8):          # r=0 is Rank 8, r=7 is Rank 1
        r_logical = 7.0 - r     # Maps loop index to logical rank coordinates (7 down to 0)
        for c in range(8):      # c=0 is File a, c=7 is File h
            f_logical = float(c)
            
            # Rail & Bridge vector tracking percentages
            u = (r_logical + 1.0) / 9.0
            v = f_logical / 7.0
            
            a_rail_pt = m1 + u * (m0 - m1)
            h_rail_pt = m3 + u * (m2 - m3)
            pixel_pt = a_rail_pt + v * (h_rail_pt - a_rail_pt)
            
            tile_centers.append(f"{pixel_pt[0]:.1f},{pixel_pt[1]:.1f}")
            
    payload = " ".join(tile_centers)
    scale_horizontal = np.linalg.norm(m1 - m3)  # Distance from a1-bot to h1-bot (across 7 files)
    scale_vertical = np.linalg.norm(m1 - m0)    # Distance from a1-bot to a8-top (across 9 ranks)
    
    return payload, scale_horizontal, scale_vertical

def get_board_corners_algebraic(corners, ids):
    """Uses the Rail & Bridge method to find the 8x8 board corners"""
    marker_map = {ids[i][0]: np.mean(corners[i][0], axis=0) for i in range(len(ids))}
    try:
        m0, m1, m2, m3 = marker_map[0], marker_map[1], marker_map[2], marker_map[3]
    except KeyError: return None

    def get_pt(r, f):
        u = (r + 1.0) / 9.0
        v = f / 7.0
        a_rail_pt = m1 + u * (m0 - m1)
        h_rail_pt = m3 + u * (m2 - m3)
        return a_rail_pt + v * (h_rail_pt - a_rail_pt)

    # Re-ordered to put Rank 8 at the TOP of the warped image output
    return np.float32([
        get_pt(7.5, -0.5),   # a8 corner -> maps to (0,0) Top-Left
        get_pt(7.5, 7.5),    # h8 corner -> maps to (900,0) Top-Right
        get_pt(-0.5, 7.5),   # h1 corner -> maps to (900,900) Bottom-Right
        get_pt(-0.5, -0.5)   # a1 corner -> maps to (0,900) Bottom-Left
    ])

# --- UTILITIES ---
def generate_fen(matrix):
    fen_rows = []
    for row in matrix:
        empty_count = 0
        fen_row = ""
        for piece in row:
            if piece == 'empty': empty_count += 1
            else:
                if empty_count > 0:
                    fen_row += str(empty_count)
                    empty_count = 0
                fen_row += piece
        if empty_count > 0: fen_row += str(empty_count)
        fen_rows.append(fen_row)
    return "/".join(fen_rows)

def fen_to_matrix(fen_string):
    rows = fen_string.split('/')
    matrix = []
    for row in rows:
        matrix_row = []
        for char in row:
            if char.isdigit(): matrix_row.extend(['empty'] * int(char))
            else: matrix_row.append(char)
        matrix.append(matrix_row)
    return matrix

# --- PROCESSING ENGINE ---
def process_batch_image(img_name, model, classes, device, transform):
    img_id = os.path.splitext(img_name)[0]
    out_path = os.path.join(RESULTS_DIR, img_id)
    os.makedirs(out_path, exist_ok=True)
    
    src = cv.imread(os.path.join(IMAGE_DIR, img_name))
    if src is None: return

    # 1. ARUCO DETECTION (Legacy API)
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    aruco_params = cv.aruco.DetectorParameters_create()
    corners, ids, _ = cv.aruco.detectMarkers(src, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) < 4:
        print(f"[{img_name}]: FAILED - Missing Markers")
        return

    # 2. RUN GEOMETRIC UTILITY DEF
    payload, scale_h, scale_v = calculate_tile_centers(corners, ids)
    if payload is None:
        print(f"[{img_name}]: FAILED - Required marker IDs missing")
        return

    # 3. ALGEBRAIC WARP
    rect = get_board_corners_algebraic(corners, ids)
    M = cv.getPerspectiveTransform(rect, np.float32([[0,0], [900,0], [900,900], [0,900]]))
    warped = cv.warpPerspective(src, M, (900, 900))

    # 4. DEBUG PICTURES
    grid_pts = np.linspace(0, 900, 9).astype(int)
    grid_viz = warped.copy()
    for pt in grid_pts:
        cv.line(grid_viz, (pt, 0), (pt, 900), (255, 0, 0), 2)
        cv.line(grid_viz, (0, pt), (900, pt), (255, 0, 0), 2)
    cv.imwrite(os.path.join(out_path, "debug_algebra_grid.png"), grid_viz)

    # 5. CNN INFERENCE LOOP 
    board_matrix = []
    diag_data = []
    expected_matrix = fen_to_matrix(EXPECTED_FENS[img_name]) if img_name in EXPECTED_FENS else None

    for r in range(8):
        row_pieces = []
        for c in range(8):
            sq = f"{chr(ord('a')+c)}{8-r}"
            y1, y2 = grid_pts[r], grid_pts[r+1]
            x1, x2 = grid_pts[c], grid_pts[c+1]
            
            # Pre-process the tile for the CNN 
            tile = cv.bilateralFilter(warped[y1:y2, x1:x2], d=9, sigmaColor=75, sigmaSpace=75)
            pil_img = Image.fromarray(cv.cvtColor(tile, cv.COLOR_BGR2RGB))
            input_tensor = transform(pil_img).unsqueeze(0).to(device)
            
            with torch.no_grad():
                logits = model(input_tensor)
                probs = torch.softmax(logits, dim=1)[0]
                confs, idxs = torch.topk(probs, 2)
                final_label = classes[idxs[0].item()]
                runner_up = classes[idxs[1].item()]
                
                row_pieces.append(final_label)
                exp = expected_matrix[r][c] if expected_matrix else "N/A"
                diag_data.append((sq, final_label, confs[0].item(), runner_up, confs[1].item(), exp))

        board_matrix.append(row_pieces)

    # --- LOGGING ---
    final_fen = generate_fen(board_matrix)
    is_correct = (img_name in EXPECTED_FENS and final_fen == EXPECTED_FENS[img_name])
    print(f"[{img_name}]: {'CORRECT' if is_correct else 'FAILED'}")
    if not is_correct:
        print(f"   Detected: {final_fen}")
        
    # Print clean calibration measurements to terminal
    print("\n--- GEOMETRIC SYSTEM CALIBRATION DATA ---")
    print(f"Tile Centers Payload (A8 -> H1):\n{payload[:120]}... [TRUNCATED]")
    print(f"Adjacent Marker Horizontal Scale (m1 <-> m3): {scale_h:.2f} pixels")
    print(f"Adjacent Marker Vertical Scale   (m1 <-> m0): {scale_v:.2f} pixels")
    print("-----------------------------------------\n")

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(MODEL_PATH, map_location=device)
    classes = checkpoint['classes']
    model = ChessResNet(num_classes=len(classes)).to(device)
    model.load_state_dict(checkpoint['model_state'])
    model.eval()
    
    tf = transforms.Compose([
        transforms.Resize((64, 64)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])
    
    for img in TEST_IMAGES:
        process_batch_image(img, model, classes, device, tf)