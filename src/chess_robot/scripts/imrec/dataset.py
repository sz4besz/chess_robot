import cv2 as cv
import numpy as np
import os
import random
import shutil

# --- CONFIG ---
BASE_PATH = "/home/mates/onlab/src/chess_robot/scripts/imrec"
PIECES_DIR = os.path.join(BASE_PATH, "chessPieces")
BG_TILES_DIR = os.path.join(BASE_PATH, "results/background_tiles") 
DATASET_DIR = os.path.join(BASE_PATH, "dataset")

SAMPLES_PER_PIECE = 1500 
TILE_SIZE = 64

def setup_dirs():
    if os.path.exists(DATASET_DIR): shutil.rmtree(DATASET_DIR)
    classes = ['P','N','B','R','Q','K','p','n','b','r','q','k','empty']
    for c in classes: os.makedirs(os.path.join(DATASET_DIR, c), exist_ok=True)

def apply_perspective_warp(img):
    """Simulates real camera angles by warping tile corners."""
    pts1 = np.float32([[0,0],[TILE_SIZE,0],[0,TILE_SIZE],[TILE_SIZE,TILE_SIZE]])
    p = 5 # max pixel warp
    pts2 = np.float32([[random.randint(0,p), random.randint(0,p)],
                       [TILE_SIZE-random.randint(0,p), random.randint(0,p)],
                       [random.randint(0,p), TILE_SIZE-random.randint(0,p)],
                       [TILE_SIZE-random.randint(0,p), TILE_SIZE-random.randint(0,p)]])
    M = cv.getPerspectiveTransform(pts1, pts2)
    return cv.warpPerspective(img, M, (TILE_SIZE, TILE_SIZE), borderValue=(255,255,255))

def overlay_transparent(bg_img, img_to_overlay_t, x, y):
    bg_img = bg_img.copy()
    b, g, r, a = cv.split(img_to_overlay_t)
    overlay_color = cv.merge((b, g, r))
    mask = a
    h, w = overlay_color.shape[:2]
    y1, y2 = max(0, y), min(TILE_SIZE, y + h)
    x1, x2 = max(0, x), min(TILE_SIZE, x + w)
    y1o, y2o = max(0, -y), min(h, TILE_SIZE - y)
    x1o, x2o = max(0, -x), min(w, TILE_SIZE - x)
    if y1 >= y2 or x1 >= x2: return bg_img
    roi = bg_img[y1:y2, x1:x2]
    mask_part = mask[y1o:y2o, x1o:x2o]
    img1_bg = cv.bitwise_and(roi, roi, mask=cv.bitwise_not(mask_part))
    img2_fg = cv.bitwise_and(overlay_color[y1o:y2o, x1o:x2o], overlay_color[y1o:y2o, x1o:x2o], mask=mask_part)
    bg_img[y1:y2, x1:x2] = cv.add(img1_bg, img2_fg)
    return bg_img

def main():
    setup_dirs()
    bg_files = [os.path.join(BG_TILES_DIR, f) for f in os.listdir(BG_TILES_DIR) if f.endswith('.png')]
    fen_map = {'bishop': 'B', 'king': 'K', 'knight': 'N', 'pawn': 'P', 'queen': 'Q', 'rook': 'R'}
    
    for f in os.listdir(PIECES_DIR):
        if not f.lower().endswith('.png'): continue
        p_name = f.lower().replace('white','').replace('black','').replace('.png','').strip('_')
        char = fen_map.get(p_name, '?')
        if not f.lower().startswith('white'): char = char.lower()
        raw_icon = cv.imread(os.path.join(PIECES_DIR, f), cv.IMREAD_UNCHANGED)
        
        for i in range(SAMPLES_PER_PIECE):
            bg = cv.imread(random.choice(bg_files))
            bg = cv.resize(bg, (TILE_SIZE, TILE_SIZE))
            
            # Scale & 360 Rotation
            current_sz = int(52 * random.uniform(0.85, 1.15))
            token = cv.resize(raw_icon, (current_sz, current_sz))
            M = cv.getRotationMatrix2D((current_sz//2, current_sz//2), random.uniform(-180, 180), 1.0)
            rotated = cv.warpAffine(token, M, (current_sz, current_sz), borderValue=(0,0,0,0))
            
            # Position Jitter (Clipping)
            off = (TILE_SIZE - current_sz) // 2
            final = overlay_transparent(bg, rotated, off + random.randint(-12, 12), off + random.randint(-12, 12))
            
            # Final Effects: Warp, Blur, Lighting
            final = apply_perspective_warp(final)
            if random.random() > 0.8: final = cv.GaussianBlur(final, (3,3), 0)
            final = cv.convertScaleAbs(final, alpha=random.uniform(0.7, 1.3), beta=random.randint(-30, 30))
            
            cv.imwrite(os.path.join(DATASET_DIR, f"{char}/{char}_{i}.jpg"), final)

    for i in range(SAMPLES_PER_PIECE):
        bg = apply_perspective_warp(cv.resize(cv.imread(random.choice(bg_files)), (TILE_SIZE, TILE_SIZE)))
        bg = cv.convertScaleAbs(bg, alpha=random.uniform(0.7, 1.3), beta=random.randint(-30, 30))
        cv.imwrite(os.path.join(DATASET_DIR, f"empty/empty_{i}.jpg"), bg)

if __name__ == "__main__": main()