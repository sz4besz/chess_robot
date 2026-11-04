import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms, models

# --- CONFIG ---
BASE_PATH = "/home/mates/onlab/src/chess_robot/scripts/imrec"
DATASET_DIR = os.path.join(BASE_PATH, "dataset")
MODEL_SAVE_PATH = os.path.join(BASE_PATH, "chess_brain.pth")
BATCH_SIZE = 64
EPOCHS = 50
LEARNING_RATE = 0.001

class ChessResNet(nn.Module):
    def __init__(self, num_classes=13):
        super(ChessResNet, self).__init__()
        # Pre-trained ResNet18 for deeper feature extraction
        self.model = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        self.model.fc = nn.Linear(self.model.fc.in_features, num_classes)

    def forward(self, x): return self.model(x)

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    train_tf = transforms.Compose([
        transforms.Resize((64, 64)),
        transforms.ColorJitter(0.2, 0.2),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    base_ds = datasets.ImageFolder(root=DATASET_DIR, transform=train_tf)
    classes = base_ds.classes
    num_images = len(base_ds)
    
    indices = torch.randperm(num_images).tolist()
    split = int(0.8 * num_images)
    train_loader = DataLoader(Subset(base_ds, indices[:split]), batch_size=BATCH_SIZE, shuffle=True, num_workers=4)
    val_loader = DataLoader(Subset(base_ds, indices[split:]), batch_size=BATCH_SIZE, shuffle=False, num_workers=4)

    model = ChessResNet(num_classes=len(classes)).to(device)
    
    # Weighting: Prioritize 'empty' class accuracy
    weights = torch.ones(len(classes)).to(device)
    if 'empty' in classes: weights[classes.index('empty')] = 2.0 
    
    criterion = nn.CrossEntropyLoss(weight=weights, label_smoothing=0.1)
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=15, gamma=0.1)

    # Early Stopping params
    best_loss = float('inf')
    patience, trigger = 7, 0

    print(f"[START] Training ResNet18 on {device}...")
    for epoch in range(EPOCHS):
        model.train()
        for imgs, lbls in train_loader:
            imgs, lbls = imgs.to(device), lbls.to(device)
            loss = criterion(model(imgs), lbls)
            optimizer.zero_grad(); loss.backward(); optimizer.step()

        model.eval()
        v_loss = 0.0
        with torch.no_grad():
            for imgs, lbls in val_loader:
                imgs, lbls = imgs.to(device), lbls.to(device)
                v_loss += criterion(model(imgs), lbls).item()
        
        avg_v_loss = v_loss / len(val_loader)
        scheduler.step()
        print(f"Epoch {epoch+1} Val Loss: {avg_v_loss:.4f}")

        if avg_v_loss < best_loss:
            best_loss = avg_v_loss
            trigger = 0
            torch.save({'model_state': model.state_dict(), 'classes': classes}, MODEL_SAVE_PATH)
        else:
            trigger += 1
            if trigger >= patience: 
                print("Early Stopping triggered."); break

if __name__ == "__main__": main()