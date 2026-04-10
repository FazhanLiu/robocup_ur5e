import os
import shutil
import yaml
from ultralytics import YOLO

# ================= 1. 基础配置 =================
base_dir = "/Users/hrone/Downloads/group project/yolo_dataset"

# ================= 2. 整理文件夹结构 =================
print("🧹 正在自动整理 YOLO 所需的文件夹结构...")
for split in ['train', 'valid', 'test']:
    split_dir = os.path.join(base_dir, split)
    if not os.path.exists(split_dir):
        continue

    images_dir = os.path.join(split_dir, "images")
    labels_dir = os.path.join(split_dir, "labels")

    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)

    # 遍历当前目录，把图片和标注文件分别归拢
    for f in os.listdir(split_dir):
        src = os.path.join(split_dir, f)

        if f.lower().endswith(('.jpg', '.jpeg', '.png')):
            dst = os.path.join(images_dir, f)
            if src != dst:
                shutil.move(src, dst)

        if f.lower().endswith('.txt') and f != 'classes.txt':
            dst = os.path.join(labels_dir, f)
            if src != dst:
                shutil.move(src, dst)

# ================= 3. 自动生成 data.yaml =================
print("📝 正在生成 YOLO 专属的数据集配置文件 data.yaml...")

# 优先读取 dataset 根目录的 classes.txt，如果不存在就自动根据 label 推断类别数
classes_file = os.path.join(base_dir, "classes.txt")

if os.path.exists(classes_file):
    with open(classes_file, 'r') as f:
        class_names = [line.strip() for line in f.readlines() if line.strip()]
else:
    print("⚠️ 未找到 classes.txt，正在从 labels 自动推断类别...")
    class_ids = set()

    for split in ['train', 'valid', 'test']:
        labels_dir = os.path.join(base_dir, split, "labels")
        if not os.path.exists(labels_dir):
            continue

        for file in os.listdir(labels_dir):
            if file.endswith(".txt"):
                with open(os.path.join(labels_dir, file)) as lf:
                    for line in lf:
                        parts = line.strip().split()
                        if parts:
                            class_ids.add(int(parts[0]))

    max_id = max(class_ids) if class_ids else 0
    class_names = [f"class_{i}" for i in range(max_id + 1)]

print(f"📦 检测到 {len(class_names)} 个类别")

yaml_data = {
    'path': base_dir,
    'train': 'train/images',
    'val': 'valid/images',
    'test': 'test/images',
    'nc': len(class_names),
    'names': class_names
}

yaml_path = os.path.join(base_dir, "data.yaml")
with open(yaml_path, 'w') as f:
    yaml.dump(yaml_data, f, sort_keys=False)
print(f"✅ 配置文件已生成: {yaml_path}")

# ================= 4. 开始 YOLO26 训练 =================
print("🚀 开始下载并加载 YOLO26 预训练模型...")

# 加载最新的 YOLO26 纳米版预训练权重 (yolo26n.pt)，它速度最快，适合用来跑通流程。
# 如果你的数据很复杂且电脑配置足够，后续可以改为 yolo26s.pt (小模型) 或 yolo26m.pt (中模型)
model = YOLO('yolo26n.pt')

print("🔥 训练引擎点火，开始运算！")
results = model.train(
    data=yaml_path,
    epochs=100,  # 训练轮数（先跑 50 轮试试水，最终项目可能需要 100-300 轮）
    # 跑30轮需要20分钟
    imgsz=640,  # 图像输入尺寸
    batch=4,  # 每次喂给显卡的图片数（如果 Mac 提示内存不足/Killed，请改成 8 或 4）
    device='mps',  # 🍎 召唤苹果硬件底层加速！(如果运行报错，可以删掉这行让它默认用 CPU 跑)
    project='YOLO26_Results',  # 结果会保存在你当前运行代码的目录下的这个文件夹里
    name='group_project_v1'  # 本次训练的版本命名
)

print("🎉 恭喜！训练代码已成功启动！")