import os
from collections import Counter

base_dir = 'data'

# Считаем файлы
counts = Counter()
for split in ('train', 'val'):
    split_dir = os.path.join(base_dir, split)
    if not os.path.isdir(split_dir):
        continue
    for class_name in os.listdir(split_dir):
        class_dir = os.path.join(split_dir, class_name)
        if os.path.isdir(class_dir):
            # считаем только .bmp
            n = sum(1 for f in os.listdir(class_dir) if f.lower().endswith('.bmp'))
            counts[class_name] += n

# Вычисляем среднее
total = sum(counts.values())
num_classes = len(counts)
if num_classes == 0:
    print("Нет классов для подсчёта.")
    exit()

mean_count = total / num_classes

# Печатаем таблицу
header = f"{'class':20s} {'count':>7s} {'deviation (%)':>14s}"
print(header)
print('-' * len(header))
for cls in sorted(counts):
    cnt = counts[cls]
    deviation = (cnt - mean_count) / mean_count * 100
    print(f"{cls:20s} {cnt:7d} {deviation:14.2f}")
