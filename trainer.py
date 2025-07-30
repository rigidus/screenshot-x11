import os
import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader, Subset
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt

# Hyperparameters
batch_size        = 64
learning_rate     = 1e-3
num_epochs        = 20
max_train_samples = 1028700
max_val_samples   = 10287
data_dir          = './data'
save_path         = './model.pth'
top_k_confused    = 20    # число топ-K часто путаемых классов

# -----------------------------------------------------------------------------
# 1) Data transforms and loaders
# -----------------------------------------------------------------------------
transform = transforms.Compose([
    transforms.Grayscale(num_output_channels=1),
    transforms.ToTensor(),
    transforms.Normalize((0.5,), (0.5,)),
])

full_train = datasets.ImageFolder(os.path.join(data_dir, 'train'), transform=transform)
full_val   = datasets.ImageFolder(os.path.join(data_dir, 'val'),   transform=transform)

rng = np.random.default_rng(42)
train_idxs = rng.choice(len(full_train), size=min(max_train_samples, len(full_train)), replace=False)
val_idxs   = rng.choice(len(full_val),   size=min(max_val_samples,   len(full_val)),   replace=False)

train_loader = DataLoader(Subset(full_train, train_idxs),
                          batch_size=batch_size, shuffle=True,  num_workers=0)
val_loader   = DataLoader(Subset(full_val,   val_idxs),
                          batch_size=batch_size, shuffle=False, num_workers=0)

print(f"[INFO] Train samples={len(train_loader.dataset)}, Val samples={len(val_loader.dataset)}")

# -----------------------------------------------------------------------------
# 2) Define a “light” CNN
# -----------------------------------------------------------------------------
class SimpleCNN(nn.Module):
    def __init__(self, num_classes: int):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=5, padding=2, bias=False),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1, bias=False),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1, bias=False),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 4 * 4, 128, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(128, num_classes, bias=False),
        )

    def forward(self, x):
        return self.classifier(self.features(x))


# -----------------------------------------------------------------------------
# 3) Создание модели, loss, optimizer, инициируем стартовую эпоху
# -----------------------------------------------------------------------------
num_classes = len(full_train.classes)
model       = SimpleCNN(num_classes)
criterion   = nn.CrossEntropyLoss()
optimizer   = optim.Adam(model.parameters(), lr=learning_rate)

start_epoch = 1
if os.path.isfile(save_path):
    print(f"[INFO] Загрузка чекпоинта из {save_path}")
    checkpoint = torch.load(save_path, map_location='cpu')
    model.load_state_dict(checkpoint['model_state'])
    optimizer.load_state_dict(checkpoint['optim_state'])
    # если в файле есть число epoch, начинаем с него + 1
    start_epoch = checkpoint.get('epoch', 0) + 1

# -----------------------------------------------------------------------------
# 4) Training loop
# -----------------------------------------------------------------------------
prev_train_loss = float('inf')
prev_val_loss   = float('inf')

for epoch in range(start_epoch, num_epochs + 1):
    # — Train —
    model.train()
    total_train_loss = 0.0
    correct_train    = 0
    total_train      = 0

    for inputs, targets in tqdm(train_loader,
                                desc=f"Epoch {epoch}/{num_epochs} [Train]",
                                leave=False):
        optimizer.zero_grad()
        outputs = model(inputs)
        loss    = criterion(outputs, targets)
        loss.backward()
        optimizer.step()

        total_train_loss += loss.item() * inputs.size(0)
        _, preds = outputs.max(1)
        correct_train += preds.eq(targets).sum().item()
        total_train   += targets.size(0)

    train_loss = total_train_loss / total_train
    train_acc  = 100.0 * correct_train / total_train

    # — Validate —
    model.eval()
    total_val_loss = 0.0
    correct_val    = 0
    total_val      = 0
    # Prepare confusion matrix array
    cm = np.zeros((num_classes, num_classes), dtype=int)

    with torch.no_grad():
        for inputs, targets in tqdm(val_loader,
                                    desc=f"Epoch {epoch}/{num_epochs} [Val] ",
                                    leave=False):
            outputs = model(inputs)
            loss    = criterion(outputs, targets)
            total_val_loss += loss.item() * inputs.size(0)

            _, preds = outputs.max(1)
            # update confusion matrix counts
            t = targets.cpu().numpy()
            p = preds.cpu().numpy()
            for true_label, pred_label in zip(t, p):
                cm[true_label, pred_label] += 1

            correct_val += preds.eq(targets).sum().item()
            total_val   += targets.size(0)

    val_loss = total_val_loss / total_val
    val_acc  = 100.0 * correct_val / total_val

    # Normalize confusion matrix by row (true label) sums
    cm_norm = cm.astype(np.float32)
    row_sums = cm_norm.sum(axis=1, keepdims=True)
    cm_norm = np.divide(cm_norm, row_sums, where=row_sums != 0)

    # Сохраняем полную нормированную матрицу для последующего анализа
    try:
        np.save(f'cm_norm_epoch_{epoch}.npy', cm_norm)
    except Exception as e:
        print(f"[WARN] Не удалось сохранить cm_norm: {e}")

    # Выбираем топ-K классов с наибольшим уровнем ошибок (1 - точность по диагонали)
    error_rates = 1.0 - np.diag(cm_norm)
    top_idxs    = np.argsort(error_rates)[-top_k_confused:][::-1]
    sub_cm      = cm_norm[np.ix_(top_idxs, top_idxs)]
    sub_labels  = [full_train.classes[i] for i in top_idxs]

    # Рисуем уменьшенную матрицу только для топ-K
    try:
        plt.figure(figsize=(8, 8))
        im = plt.imshow(sub_cm, interpolation='nearest', cmap='Blues')
        plt.title(f'Top-{top_k_confused} Confused Classes — Epoch {epoch}')
        plt.colorbar(im, format='%.2f')
        tick_marks = np.arange(len(top_idxs))
        plt.xticks(tick_marks, sub_labels, rotation=90)
        plt.yticks(tick_marks, sub_labels)
        plt.xlabel('Predicted label')
        plt.ylabel('True label')
        plt.tight_layout()
        plt.savefig(f'top_{top_k_confused}_confusion_epoch_{epoch}.png', bbox_inches='tight')
        plt.close()
    except Exception as e:
        print(f"[WARN] Не удалось отрисовать top-K confusion matrix: {e}")


    # — Logging —
    print(f"[INFO] Epoch {epoch}/{num_epochs}  "
          f"Train Loss: {train_loss:.4f}  | Train Acc: {train_acc:5.2f}%  |  "
          f"Val Loss: {val_loss:.4f}  |  Val Acc: {val_acc:5.2f}%")

    # — Overfitting warning —
    eps = 1e-3
    if (val_loss - prev_val_loss) > eps and (prev_train_loss - train_loss) > eps:
        print(f"[WARN] Возможное переобучение на epoch {epoch}")

    prev_train_loss, prev_val_loss = train_loss, val_loss

    # — Save checkpoint —
    torch.save({
        'epoch': epoch,
        'model_state': model.state_dict(),
        'optim_state': optimizer.state_dict(),
    }, save_path)
    print(f"[INFO] Сохранён чекпоинт epoch {epoch} → {save_path}")
