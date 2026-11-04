import os
import torch
import torch.nn as nn
import cv2 as cv
import numpy as np
from torchvision import transforms, models
from PIL import Image

class ChessResNet(nn.Module):
    def __init__(self, num_classes=13):
        super(ChessResNet, self).__init__()
        self.model = models.resnet18()
        self.model.fc = nn.Linear(self.model.fc.in_features, num_classes)
    def forward(self, x):
        return self.model(x)

# --- GEOMETRIC HELPERS ---
def calculate_tile_centers(corners, ids):
    marker_map = {ids[i][0]: np.mean(corners[i][0], axis=0) for i in range(len(ids))}
    try:
        m0, m1, m2, m3 = marker_map[0], marker_map[1], marker_map[2], marker_map[3]
    except KeyError:
        return None, 0, 0

    tile_centers = []
    for r in range(8):
        r_logical = 7.0 - r
        for c in range(8):
            f_logical = float(c)
            u = (r_logical + 1.0) / 9.0
            v = f_logical / 7.0
            
            a_rail_pt = m1 + u * (m0 - m1)
            h_rail_pt = m3 + u * (m2 - m3)
            pixel_pt = a_rail_pt + v * (h_rail_pt - a_rail_pt)
            
            tile_centers.append(f"{pixel_pt[0]:.1f},{pixel_pt[1]:.1f}")
            
    payload = " ".join(tile_centers)
    scale_h = np.linalg.norm(m1 - m3)
    scale_v = np.linalg.norm(m1 - m0)
    
    return payload, scale_h, scale_v

def get_board_corners_algebraic(corners, ids):
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

    return np.float32([
        get_pt(7.5, -0.5),   # a8
        get_pt(7.5, 7.5),    # h8
        get_pt(-0.5, 7.5),   # h1
        get_pt(-0.5, -0.5)   # a1
    ])

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


# --- THE MAIN CLASS ---
class imrec:
    def __init__(self, model_path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = torch.load(model_path, map_location=self.device)
        classes = checkpoint['classes']
        self.model = ChessResNet(num_classes=len(classes)).to(self.device)
        self.model.load_state_dict(checkpoint['model_state'])
        self.model.eval()
        self.classes = classes
        print(f"Model classes loaded: {self.classes}")
        
        self.transform = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def get_board(self, src):
        if src is None: return None
        
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        aruco_params = cv.aruco.DetectorParameters_create()
        corners, ids, _ = cv.aruco.detectMarkers(src, aruco_dict, parameters=aruco_params)

        if ids is None or len(ids) < 4: return None

        rect = get_board_corners_algebraic(corners, ids)
        if rect is None: return None

        M = cv.getPerspectiveTransform(rect, np.float32([[0,0], [900,0], [900,900], [0,900]]))
        warped = cv.warpPerspective(src, M, (900, 900))

        grid_pts = np.linspace(0, 900, 9).astype(int)
        board_matrix = []

        for r in range(8):
            row_pieces = []
            for c in range(8):
                y1, y2 = grid_pts[r], grid_pts[r+1]
                x1, x2 = grid_pts[c], grid_pts[c+1]
                
                tile = cv.bilateralFilter(warped[y1:y2, x1:x2], d=9, sigmaColor=75, sigmaSpace=75)
                pil_img = Image.fromarray(cv.cvtColor(tile, cv.COLOR_BGR2RGB))
                input_tensor = self.transform(pil_img).unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    logits = self.model(input_tensor)
                    probs = torch.softmax(logits, dim=1)[0]
                    _, idxs = torch.topk(probs, 1)
                    row_pieces.append(self.classes[idxs[0].item()])
            board_matrix.append(row_pieces)

        return generate_fen(board_matrix)

    def get_centers(self, src):
        if src is None: return None, 0, 0
        
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        aruco_params = cv.aruco.DetectorParameters_create()
        corners, ids, _ = cv.aruco.detectMarkers(src, aruco_dict, parameters=aruco_params)

        if ids is None or len(ids) < 4: return None, 0, 0
        
        return calculate_tile_centers(corners, ids)