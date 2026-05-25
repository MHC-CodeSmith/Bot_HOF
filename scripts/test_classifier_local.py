#!/usr/bin/env python3
import cv2
import numpy as np
import sys

def main():
    print("==================================================")
    print("   Vision Classifier - Local Webcam Debug Tool    ")
    print("==================================================")
    print("Controles:")
    print("  - Use os Sliders na tela para calibrar as cores HSV")
    print("  - Pressione 'q' na janela do vídeo para SAIR")
    print("==================================================")

    # Tenta abrir a câmera buscando índices disponíveis de forma robusta
    cap = None
    for idx in [0, 2, 4, 1, 6]:
        print(f"Tentando abrir câmera no índice {idx}...")
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"[OK] Câmera iniciada com sucesso no índice {idx}!")
                break
            else:
                cap.release()
                cap = None
        else:
            if cap is not None:
                cap.release()
                cap = None

    if cap is None or not cap.isOpened():
        print("[ERRO] Não foi possível abrir nenhuma câmera local (/dev/video0, 2, 4, 1, 6).")
        print("Dica: Certifique-se de que a webcam não está sendo usada por outro aplicativo ou contêiner Docker.")
        sys.exit(1)

    # Cria janela interativa
    window_name = "Classificador - Video de Teste"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)

    # Valores iniciais de calibração padrão (baseados no simple classifier)
    # Vermelho
    r_h1_low, r_h1_high = 0, 8
    r_h2_low, r_h2_high = 170, 179
    r_s_low, r_v_low = 90, 60
    # Azul
    b_h_low, b_h_high = 88, 130
    b_s_low, b_v_low = 80, 60

    # Adiciona controles deslizantes (Trackbars) para calibração em tempo real!
    cv2.createTrackbar("R_H_Low", window_name, r_h1_low, 180, lambda x: None)
    cv2.createTrackbar("R_H_High", window_name, r_h1_high, 180, lambda x: None)
    cv2.createTrackbar("R_S_Low", window_name, r_s_low, 255, lambda x: None)
    cv2.createTrackbar("R_V_Low", window_name, r_v_low, 255, lambda x: None)

    cv2.createTrackbar("B_H_Low", window_name, b_h_low, 180, lambda x: None)
    cv2.createTrackbar("B_H_High", window_name, b_h_high, 180, lambda x: None)
    cv2.createTrackbar("B_S_Low", window_name, b_s_low, 255, lambda x: None)
    cv2.createTrackbar("B_V_Low", window_name, b_v_low, 255, lambda x: None)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Falha ao ler frame da webcam.")
            break

        # Lê os valores atuais dos trackbars
        rh_l = cv2.getTrackbarPos("R_H_Low", window_name)
        rh_h = cv2.getTrackbarPos("R_H_High", window_name)
        rs_l = cv2.getTrackbarPos("R_S_Low", window_name)
        rv_l = cv2.getTrackbarPos("R_V_Low", window_name)

        bh_l = cv2.getTrackbarPos("B_H_Low", window_name)
        bh_h = cv2.getTrackbarPos("B_H_High", window_name)
        bs_l = cv2.getTrackbarPos("B_S_Low", window_name)
        bv_l = cv2.getTrackbarPos("B_V_Low", window_name)

        # Cópia para desenhar os overlays de debug
        debug_vis = frame.copy()
        h, w = frame.shape[:2]

        # Define a ROI central (ex: 20% a 80% horizontal, 15% a 85% vertical)
        x0, x1 = int(w * 0.20), int(w * 0.80)
        y0, y1 = int(h * 0.15), int(h * 0.85)
        
        # Desenha a caixa de ROI (Verde)
        cv2.rectangle(debug_vis, (x0, y0), (x1, y1), (0, 255, 0), 2)

        # Corta a ROI para processamento
        roi = frame[y0:y1, x0:x1]
        roi_h, roi_w = roi.shape[:2]

        # Corta a parte inferior da ROI (onde ficaria o texto da esteira)
        y_cut = int(roi_h * 0.78) # Corta os 22% inferiores
        cv2.line(debug_vis, (x0, y0 + y_cut), (x1, y0 + y_cut), (100, 255, 100), 2)

        # Máscara útil
        valid_mask = np.ones((roi_h, roi_w), dtype=np.uint8) * 255
        cv2.rectangle(valid_mask, (0, y_cut), (roi_w, roi_h), 0, -1)

        # Processamento HSV
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Máscara Vermelha (com ranges invertidos do HSV)
        m1 = cv2.inRange(hsv, np.array([rh_l, rs_l, rv_l], dtype=np.uint8), np.array([rh_h, 255, 255], dtype=np.uint8))
        m2 = cv2.inRange(hsv, np.array([170, rs_l, rv_l], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        red_mask = cv2.bitwise_or(m1, m2)

        # Máscara Azul
        blue_mask = cv2.inRange(hsv, np.array([bh_l, bs_l, bv_l], dtype=np.uint8), np.array([bh_h, 255, 255], dtype=np.uint8))

        # Aplica a máscara útil (ignorando fundo de texto)
        red_mask = cv2.bitwise_and(red_mask, valid_mask)
        blue_mask = cv2.bitwise_and(blue_mask, valid_mask)

        # Limpeza morfológica
        k = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, k, iterations=1)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, k, iterations=2)

        # Função auxiliar para extrair informações do melhor blob
        def get_best_blob(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            
            best_cnt = None
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Área mínima (1% da ROI)
                if area > 0.01 * (roi_w * y_cut) and area > max_area:
                    max_area = area
                    best_cnt = cnt
            return best_cnt

        red_cnt = get_best_blob(red_mask)
        blue_cnt = get_best_blob(blue_mask)

        label = "unknown"
        # Prioriza o maior blob detectado
        if red_cnt is not None and blue_cnt is None:
            label = "red"
        elif blue_cnt is not None and red_cnt is None:
            label = "blue"
        elif red_cnt is not None and blue_cnt is not None:
            if cv2.contourArea(red_cnt) > cv2.contourArea(blue_cnt):
                label = "red"
            else:
                label = "blue"

        # Desenha contornos se encontrados
        if red_cnt is not None:
            approx = cv2.approxPolyDP(red_cnt, 0.04 * cv2.arcLength(red_cnt, True), True)
            approx_shifted = approx + np.array([[[x0, y0]]], dtype=np.int32)
            cv2.drawContours(debug_vis, [approx_shifted], -1, (0, 0, 255), 3)
            
        if blue_cnt is not None:
            approx = cv2.approxPolyDP(blue_cnt, 0.04 * cv2.arcLength(blue_cnt, True), True)
            approx_shifted = approx + np.array([[[x0, y0]]], dtype=np.int32)
            cv2.drawContours(debug_vis, [approx_shifted], -1, (255, 0, 0), 3)

        # Overlay de Status
        if label == "red":
            color = (0, 0, 255)
            text = "PRODUTO VERMELHO"
        elif label == "blue":
            color = (255, 0, 0)
            text = "PRODUTO AZUL"
        else:
            color = (128, 128, 128)
            text = "AGUARDANDO PRODUTO..."

        cv2.putText(debug_vis, f"Status: {text}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 3)
        cv2.putText(debug_vis, f"Area R: {int(cv2.contourArea(red_cnt)) if red_cnt is not None else 0}px | B: {int(cv2.contourArea(blue_cnt)) if blue_cnt is not None else 0}px", 
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Exibe a imagem principal
        cv2.imshow(window_name, debug_vis)

        # Exibe as máscaras menores de calibração
        cv2.imshow("Filtro Vermelho (HSV)", cv2.resize(red_mask, (320, 240)))
        cv2.imshow("Filtro Azul (HSV)", cv2.resize(blue_mask, (320, 240)))

        # Tecla de saída
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Processo finalizado pelo usuário.")

if __name__ == "__main__":
    main()
