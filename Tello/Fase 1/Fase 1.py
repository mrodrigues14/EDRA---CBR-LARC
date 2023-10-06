import cv2
import torch
from djitellopy import Tello
import threading
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import non_max_suppression
from yolov5.utils.torch_utils import select_device

# Carregar o modelo treinado YOLOv5
weights = 'best.pt'
device = select_device('')  # Use uma GPU disponível, se disponível
model = attempt_load(weights, map_location=device)
model = model.to(device)
stride = int(model.stride.max())  # Stride máximo para redimensionar corretamente as coordenadas

# Configurações para supressão não máxima
conf_threshold = 0.3
iou_threshold = 0.4

def streaming():
    tello.streamon()
    while True:
        frame = tello.get_frame_read().frame

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

                # Controle do drone com base na detecção
                if label.startswith("base"):
                    base_center_x = (int(x1) + int(x2)) // 2
                    base_center_y = (int(y1) + int(y2)) // 2
                    img_center_x = frame.shape[1] // 2
                    img_center_y = frame.shape[0] // 2

                    threshold = 50  # Ajuste conforme necessário

                    if abs(base_center_x - img_center_x) > threshold:
                        if base_center_x < img_center_x:
                            tello.move_left(20)
                        else:
                            tello.move_right(20)

                    if abs(base_center_y - img_center_y) > threshold:
                        if base_center_y < img_center_y:
                            tello.move_up(20)
                        else:
                            tello.move_down(20)

                    # Se a base estiver centralizada, pouse o drone
                    if abs(base_center_x - img_center_x) <= threshold and abs(base_center_y - img_center_y) <= threshold:
                        tello.land()
                        return

        cv2.imshow('YOLOv5 Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    streaming_thread.join()
