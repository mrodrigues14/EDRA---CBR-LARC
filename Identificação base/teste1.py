import cv2
import torch
from yolov5.models.experimental import attempt_load
from yolov5.utils.torch_utils import select_device

# Carregar o modelo treinado YOLOv5
weights = 'path/to/your/yolov5/weights.pt'
device = select_device('')  # Use uma GPU disponível, se disponível
model = attempt_load(weights, map_location=device)
stride = int(model.stride.max())  # Stride máximo para redimensionar corretamente as coordenadas

# Configurações para supressão não máxima
conf_threshold = 0.3
iou_threshold = 0.4

# Inicializar a câmera
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Desenhar ponto vermelho no centro da câmera
    camera_center_x = frame.shape[1] // 2  # Coordenada x do centro da câmera
    camera_center_y = frame.shape[0] // 2  # Coordenada y do centro da câmera
    cv2.circle(frame, (camera_center_x, camera_center_y), 5, (0, 0, 255), -1)  # Vermelho

    # Detecção de objetos com o YOLOv5
    img = torch.from_numpy(frame).to(device)
    img = img.float() / 255.0
    img = img.permute(2, 0, 1).unsqueeze(0)

    pred = model(img)[0]
    pred = non_max_suppression(pred, conf_threshold, iou_threshold)[0]

    if pred is not None and len(pred):
        for det in pred:
            x1, y1, x2, y2, conf, cls = det.tolist()
            label = f'{model.names[int(cls)]} {conf:.2f}'

            # Desenhar caixa delimitadora e rótulo
            color = (0, 255, 0)  # Cor da caixa delimitadora (verde)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Calcular e exibir o centro do objeto
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            cv2.circle(frame, (center_x, center_y), 5, color, -1)

            # Calcular distância relativa horizontal entre o centro do objeto e o centro da câmera
            relative_distance = center_x - camera_center_x
            cv2.putText(frame, f'Dist: {relative_distance}', (int(x1), int(y2) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 2)

    cv2.imshow('YOLOv5 Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
