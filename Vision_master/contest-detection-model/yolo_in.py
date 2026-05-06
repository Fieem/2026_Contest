from ultralytics import YOLO

if __name__ == "__main__":
    model = YOLO("./yolo11n.pt")	# 加载预训练模型

    # 启动训练流程
    results = model.train(
        data="./data.yaml",  # 【关键】这里是数据集配置文件的路径，而非文件夹路径
        epochs=30,
        imgsz=640,          # 输入图像的尺寸，此处统一缩放为720x720像素
        batch=4,
        project="detection_results",
        deterministic=False,
    )