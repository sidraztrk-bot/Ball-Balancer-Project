import numpy as np
import cv2
import time
import serial

class STM32BallTracker:
    def __init__(self, camera_source=0, stm32_port='COM9', baudrate=38400, 
                 real_width_cm=16, real_height_cm=16):
        """
        STM32 Ball Tracker - ROI seçimi + CM dönüşüm + Beyaz top algılama
        """
        self.camera_source = camera_source
        self.stm32_port = stm32_port
        self.baudrate = baudrate
        self.ser = None

        # ROI parametreleri
        self.roi = None
        self.roi_selected = False
        self.real_width_cm = real_width_cm
        self.real_height_cm = real_height_cm
        
        # Ball detection parametreleri
        self.min_area = 300
        self.max_area = 50000     
        self.min_circularity = 0.4
        
        # Beyaz renk HSV aralıkları
        self.lower_white = np.array([0, 0, 120])     
        self.upper_white = np.array([180, 80, 255])
        
        self.frame_center = None
        self.running = False
        self.last_sent_coords = None
        
        # Performans için ön-hesaplama
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.pixel_to_cm_scale_x = None
        self.pixel_to_cm_scale_y = None
        self.roi_offset_x = 0
        self.roi_offset_y = 0
        
        # Koordinat değişim threshold (spam önleme)
        self.coord_threshold = 0.5  # cm cinsinden minimum değişim

    def init_stm32_connection(self):
        try:
            self.ser = serial.Serial(self.stm32_port, self.baudrate, timeout=1)
            print(f"✅ Connected to STM32 on {self.stm32_port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to STM32: {e}")
            return False

    def select_roi(self, frame):
        """İlk frame'de manuel ROI seçimi"""
        print("\n📐 ROI SELECTION MODE")
        print("=" * 50)
        print("Instructions:")
        print("  1. Click and drag to select the tracking area")
        print("  2. Press ENTER to confirm selection")
        print("  3. Press 'c' to cancel and reselect")
        print("=" * 50)
        
        roi = cv2.selectROI("Select Tracking Area (Press ENTER)", frame, 
                           fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Tracking Area (Press ENTER)")
        
        if roi[2] > 0 and roi[3] > 0:
            self.roi = roi
            self.roi_selected = True
            
            x, y, w, h = roi
            self.roi_offset_x = x
            self.roi_offset_y = y
            
            # Dönüşüm scale'lerini ön-hesapla
            self.pixel_to_cm_scale_x = self.real_width_cm / w
            self.pixel_to_cm_scale_y = self.real_height_cm / h
            
            print(f"\n✅ ROI Selected:")
            print(f"   Position: ({x}, {y})")
            print(f"   Size: {w}x{h} pixels")
            print(f"   Real world: {self.real_width_cm}x{self.real_height_cm} cm")
            print(f"   Scale: {w/self.real_width_cm:.2f} pixels/cm")
            
            self.frame_center = (x + w//2, y + h//2)
            print(f"   Center: {self.frame_center}")
            
            return True
        else:
            print("❌ No valid ROI selected!")
            return False

    def pixel_to_cm(self, pixel_x, pixel_y):
        """Piksel koordinatlarını cm'ye dönüştür (ROI içinde)"""
        if not self.roi_selected:
            return None
        
        roi_x, roi_y, roi_w, roi_h = self.roi
        
        # ROI içindeki relatif pozisyon (0-1 arası)
        rel_x = (pixel_x - roi_x) / roi_w
        rel_y = (pixel_y - roi_y) / roi_h
        
        # CM'ye dönüştür (sol üst köşe origin)
        cm_x = rel_x * self.real_width_cm
        cm_y = rel_y * self.real_height_cm
        
        # Merkez origin'e dönüştür (isteğe bağlı)
        cm_x_centered = cm_x - (self.real_width_cm / 2)
        cm_y_centered = cm_y - (self.real_height_cm / 2)
        
        return {
            'pixel': (pixel_x, pixel_y),
            'cm_absolute': (round(cm_x, 2), round(cm_y, 2)),
            'cm_centered': (round(cm_x_centered, 2), round(cm_y_centered, 2))
        }
        
    def send_coordinates_to_stm32(self, coords_dict):
        """CM koordinatlarını STM32'ye gönder"""
        if self.ser and self.ser.is_open and coords_dict:
            try:
                # Merkez tabanlı koordinatları gönder
                cm_x, cm_y = coords_dict['cm_absolute']

                cm_x_int = int(cm_x * 100)
                cm_y_int = int(cm_y * 100)

                msg = f"{cm_x_int},{cm_y_int}\n"
                self.ser.write(msg.encode())

                print(f"📤 Sent to STM32 (x100): {msg.strip()}")

            except Exception as e:
                print(f"❌ Error sending data: {e}")

    def detect_ball(self, frame):
        """Beyaz top algılama (optimize edilmiş)"""
        # ROI crop işlemi
        if self.roi_selected:
            x, y, w, h = self.roi
            search_frame = frame[y:y+h, x:x+w]
            roi_offset = (x, y)
        else:
            search_frame = frame
            roi_offset = (0, 0)
        
        # HSV dönüşümü
        hsv = cv2.cvtColor(search_frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        
        # Optimize edilmiş morphological işlemler (1 iterasyon yeter)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel)
        
        # Contour bulma
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        ball_center = None
        best_contour = None
        coords_dict = None
        
        # En uygun contour'u bul
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > self.min_circularity:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"]) + roi_offset[0]
                            cy = int(M["m01"] / M["m00"]) + roi_offset[1]
                            ball_center = (cx, cy)
                            
                            # Global koordinatlara çevir
                            global_contour = contour.copy()
                            global_contour[:, :, 0] += roi_offset[0]
                            global_contour[:, :, 1] += roi_offset[1]
                            best_contour = global_contour
                            
                            coords_dict = self.pixel_to_cm(cx, cy)
                            break
        
        return ball_center, best_contour, coords_dict, white_mask

    def draw_visualization(self, frame, ball_center, best_contour, coords_dict):
        """Görselleştirme (ayrı fonksiyon)"""
        # ROI çiz
        if self.roi_selected:
            x, y, w, h = self.roi
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 255), 2)
            cv2.putText(frame, f"ROI: {self.real_width_cm}x{self.real_height_cm}cm", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Top görselleştirmesi
        if ball_center and best_contour is not None and coords_dict:
            cv2.circle(frame, ball_center, 6, (0, 255, 0), -1)
            cv2.circle(frame, ball_center, 25, (0, 255, 0), 2)
            cv2.drawContours(frame, [best_contour], -1, (255, 0, 0), 2)
            
            # Koordinat bilgileri (optimize string format)
            px, py = coords_dict['pixel']
            cm_x, cm_y = coords_dict['cm_absolute']
            
            info = f"P:({px},{py}) CM:({cm_x},{cm_y})"
            cv2.putText(frame, info, (ball_center[0] + 30, ball_center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        
        # Merkez noktası
        if self.frame_center:
            cv2.drawMarker(frame, self.frame_center, (0, 0, 255), 
                          cv2.MARKER_CROSS, 20, 2)
        
        return frame

    def run(self):
        """Ana döngü"""
        cap = cv2.VideoCapture(self.camera_source)
        if not cap.isOpened():
            print(f"❌ Camera connection failed: {self.camera_source}")
            return False
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"✅ Camera connected: {self.camera_source}")
        
        # İlk frame'i al ve ROI seç
        ret, first_frame = cap.read()
        if not ret:
            print("❌ Could not read first frame!")
            cap.release()
            return False
        
        if not self.select_roi(first_frame):
            cap.release()
            return False
        
        stm32_connected = self.init_stm32_connection()
        
        self.running = True
        print("\n🎯 STM32 Ball Tracker Started")
        print("Controls: Q - Quit | R - Reselect ROI | S - Toggle STM32")
        print("=" * 50)
        
        # FPS sayacı
        fps_counter = 0
        fps_start_time = time.time()
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Camera read error")
                    break
                
                # Top algılama (frame kopyası YOK!)
                ball_center, best_contour, coords_dict, mask = self.detect_ball(frame)
                
                # STM32'ye gönder (sadece değişim varsa)
                if stm32_connected and coords_dict:
                    self.send_coordinates_to_stm32(coords_dict)
                
                # Görselleştirme
                frame = self.draw_visualization(frame, ball_center, best_contour, coords_dict)
                
                # FPS gösterimi
                fps_counter += 1
                if fps_counter % 30 == 0:
                    elapsed = time.time() - fps_start_time
                    fps = 30 / elapsed if elapsed > 0 else 0
                    fps_start_time = time.time()
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.imshow('STM32 Ball Tracker', frame)
                cv2.imshow('White Mask', mask)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("🛑 Stopped by user")
                    break
                elif key == ord('r'):
                    if self.select_roi(frame):
                        print("✅ ROI reselected")
                elif key == ord('s'):
                    stm32_connected = not stm32_connected
                    print(f"{'📡 STM32 enabled' if stm32_connected else '🔌 STM32 disabled'}")
        
        except KeyboardInterrupt:
            print("\n⏹ Stopped with Ctrl+C")
        finally:
            self.running = False
            cap.release()
            cv2.destroyAllWindows()
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("🔌 STM32 connection closed")
            print("✅ System cleaned up")
        
        return True


def main():
    print("🤖 STM32 Ball Tracker - Optimized Version")
    print("=" * 50)
    
    CAMERA_SOURCE = "http://172.20.10.4:8080/video"
    # CAMERA_SOURCE = 0  # Webcam
    
    STM32_PORT = 'COM9'
    BAUDRATE = 38400
    
    REAL_WIDTH_CM = 16
    REAL_HEIGHT_CM = 16
    
    print(f"📱 Camera: {CAMERA_SOURCE}")
    print(f"🔌 STM32: {STM32_PORT} @ {BAUDRATE}")
    print(f"📏 Real world size: {REAL_WIDTH_CM}x{REAL_HEIGHT_CM} cm")
    print("-" * 50)
    
    tracker = STM32BallTracker(
        camera_source=CAMERA_SOURCE,
        stm32_port=STM32_PORT,
        baudrate=BAUDRATE,
        real_width_cm=REAL_WIDTH_CM,
        real_height_cm=REAL_HEIGHT_CM
    )
    
    success = tracker.run()
    if success:
        print("✅ Program completed successfully")
    else:
        print("❌ Program ended with error")


if __name__ == "__main__":
    main()