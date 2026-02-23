#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import time
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

class DrivableInference:
    def __init__(self):
        rospy.init_node('drivable_inference', anonymous=True)
        self.bridge = CvBridge()
        
        # 모델 경로
<<<<<<< HEAD
        # self.model_path = '/root/aim_ws/src/main/src/Camera/new_best.pt'
=======
>>>>>>> 8ec809943bb73b8cc7e6b9e8f6287a9ded555244
        self.model_path = '/home/autonav/aim_ws/src/main/src/Camera/new_best.pt'
        
        # 빈 틈 메꾸고, 잔디 부분 깎기 위한 파라미터 조절(홀수만 가능)
        self.USE_MORPH = True      # 기능 켜기
        # 1. Glue : 차선 사이 빈틈을 메꾸는 강도
        self.GLUE_SIZE = 11   # 클수록 멀리 떨어진 차선도 붙음.
        # 2. Cut : 바깥쪽 테두리를 깎아내는 강도
        self.CUT_SIZE = 39    # 무조건 GLUE_SIZE보다 커야 갓길이 제거됨.(이미 영역 자체를 넓게 잡은 다음 깎아내서 맞추도록 함.)
 

        # 모델 로드
        if not os.path.exists(self.model_path):
            print(f"오류: 모델 파일이 없습니다. 경로를 확인하세요: {self.model_path}")
            exit()
            
        print(f"모델 로딩 중... ({self.model_path})")
        try:
            self.model = YOLO(self.model_path, task='segment')
            print(f"모델 로드 완료!")
            print(f"전략: Glue({self.GLUE_SIZE}) -> Cut({self.CUT_SIZE})")
        except Exception as e:
            print(f"모델 로드 실패: {e}")
            exit()


        # 파라미터 설정(학습 때와 동일)
        self.CAM_FOV, self.CAM_HEIGHT = 90.0, 1.18
        self.IMG_W, self.IMG_H = 640, 480
        self.PITCH_OFFSET = 0.0
        self.ROI_MIN_X, self.ROI_MAX_X, self.ROI_WIDTH = 1.0, 20.0, 24.0
        
        self.BEV_W = 300
        self.pixels_per_meter = self.BEV_W / self.ROI_WIDTH
        self.BEV_H = int((self.ROI_MAX_X - self.ROI_MIN_X) * self.pixels_per_meter)
        self.DIST_COEFFS = np.array([-0.20, 0, 0, 0, 0])
        
        # 변환 행렬 계산
        self.calc_matrices()


        # ROS 통신
        self.input_topic = '/image_jpeg/compressed'
        self.sub = rospy.Subscriber(self.input_topic, CompressedImage, self.callback, queue_size=1)
        self.pub_vis = rospy.Publisher('/drivable/result/compressed', CompressedImage, queue_size=1)
        self.pub_mask = rospy.Publisher('/drivable/mask/compressed', CompressedImage, queue_size=1)

    def calc_matrices(self):
        f_x = (self.IMG_W / 2) / math.tan(math.radians(self.CAM_FOV) / 2)
        f_y = f_x
        c_x, c_y = self.IMG_W / 2, self.IMG_H / 2
        self.K = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])

        def project(x, y):
            u = c_x + (f_x * y) / x
            v = c_y + (f_y * self.CAM_HEIGHT) / x + (f_y * math.tan(math.radians(self.PITCH_OFFSET)))
            return [u, v]

        src_pts = np.float32([
            project(self.ROI_MAX_X, -self.ROI_WIDTH/2), project(self.ROI_MAX_X, self.ROI_WIDTH/2),
            project(self.ROI_MIN_X, -self.ROI_WIDTH/2), project(self.ROI_MIN_X, self.ROI_WIDTH/2)
        ])
        dst_pts = np.float32([[0, 0], [self.BEV_W, 0], [0, self.BEV_H], [self.BEV_W, self.BEV_H]])
        
        self.M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.source_mask = np.zeros((self.IMG_H, self.IMG_W), dtype=np.uint8)
        cv2.fillPoly(self.source_mask, [src_pts.astype(np.int32)], 255)

    def callback(self, msg):
        try:
            start_time = time.time()
            
            # 전처리
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            undist = cv2.undistort(cv_img, self.K, self.DIST_COEFFS)
            masked = cv2.bitwise_and(undist, undist, mask=self.source_mask)
            bev_input = cv2.warpPerspective(masked, self.M, (self.BEV_W, self.BEV_H))

            # YOLO 추론
            results = self.model(bev_input, imgsz=320, conf=0.5, verbose=False)

            # 마스크 생성
            drivable_mask = np.zeros((self.BEV_H, self.BEV_W), dtype=np.uint8)

            if results[0].masks is not None:
                masks = results[0].masks.data.cpu().numpy()
                if len(masks) > 0:
                    combined = np.max(masks, axis=0)
                    mask_resized = cv2.resize(combined, (self.BEV_W, self.BEV_H))
                    drivable_mask = (mask_resized > 0.5).astype(np.uint8) * 255

            # 빈 틈 메꾸고(Glue) -> 외곽 잔디 깎기(Cut) 로직
            if self.USE_MORPH and np.sum(drivable_mask) > 0:
                
                # Step 1: Dilation(팽창) - 차선 빈틈 메우기
                glue_kernel = np.ones((self.GLUE_SIZE, self.GLUE_SIZE), np.uint8)
                mask_glued = cv2.dilate(drivable_mask, glue_kernel, iterations=1)
                
                # Step 2: Erosion(침식) - 바깥 갓길 쳐내기
                # Glue보다 Cut이 커야 결과적으로 갓길이 사라짐
                cut_kernel = np.ones((self.CUT_SIZE, self.CUT_SIZE), np.uint8)
                drivable_mask = cv2.erode(mask_glued, cut_kernel, iterations=1)

            # 시각화
            vis_img = bev_input.copy()
            color_mask = np.zeros_like(vis_img)
            color_mask[drivable_mask == 255] = [0, 255, 0] # 초록색
            
            vis_img = cv2.addWeighted(vis_img, 1.0, color_mask, 0.5, 0)
            
            # 정보 출력(디버깅 확인용으로 넣은 것)
            fps = 1.0 / (time.time() - start_time)
            cv2.putText(vis_img, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if self.USE_MORPH:
                info_text = f"Glue:{self.GLUE_SIZE} / Cut:{self.CUT_SIZE}"
                cv2.putText(vis_img, info_text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # 발행
            self.pub_vis.publish(self.bridge.cv2_to_compressed_imgmsg(vis_img))
            self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(drivable_mask))

        except Exception as e:
            pass 

if __name__ == '__main__':
    try:
        DrivableInference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass