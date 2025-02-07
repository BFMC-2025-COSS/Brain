import torch

# 모델 로드
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model_path = "/home/seame/Brain/pt/640.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, device=device)
model.eval()

# 랜덤 입력 생성 (640x640 해상도, RGB)
dummy_input = torch.rand(1, 3, 640, 640).to(device)

# TorchScript 변환 (Tracing 방식)
traced_model = torch.jit.script(model, dummy_input)

# 변환된 모델 저장
traced_model_path = "/home/seame/Brain/pt/traced_640.pt"
torch.jit.save(traced_model, traced_model_path)

print(f"TorchScript 변환 완료! 저장된 모델: {traced_model_path}")