import cv2
import numpy as np
import sys
import os
from typing import Tuple

message = """
 ____    ___   ____    ____  
|  _ \  / _ \ / ___|  |___ \ 
| |_) || | | |\___ \    __) |
|  _ < | |_| | ___) |  / __/ 
|_| \_\ \___/ |____/  |_____|
                             
    _            _                 
   / \    _   _ | |_   ___         
  / _ \  | | | || __| / _ \  _____ 
 / ___ \ | |_| || |_ | (_) ||_____|
/_/   \_\ \__,_| \__| \___/        
                                   
__     __       _      _        _       
\ \   / /  ___ | |__  (_)  ___ | |  ___ 
 \ \ / /  / _ \| '_ \ | | / __|| | / _ \
  \ V /  |  __/| | | || || (__ | ||  __/
   \_/    \___||_| |_||_| \___||_| \___|  

"""
print(message)

print("ROS2 기반 자율주행 설계 및 구현")
print("Sungkyunkwan University Automation Lab.")

print("------------------Authors------------------")
print("Hyeong-Keun Hong <whaihong@g.skku.edu>")
print("Jinsun Lee <with23skku@g.skku.edu>")
print("Siwoo Lee <edenlee@g.skku.edu>")
print("Jae-Wook Jeon <jwjeon@skku.edu>")
print("------------------------------------------")




def calculate_slope_between_points(p1, p2):
    print(">>>> NEW VERSION OF SLOPE CALCULATION IS RUNNING! <<<<")
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    
    # 분모가 0이 되는 경우 (수직선) 방지
    if abs(p1_y - p2_y) < 1e-6: # 1e-6은 아주 작은 수를 의미
        return 0.0

    # ======================== 수정된 부분 시작 ========================
    # arctan과 각도 변환을 제거하고, 순수한 x변화량 / y변화량 비율을 반환합니다.
    # 이 값은 일반적으로 -1.0 ~ 1.0 사이의 작은 값을 가집니다.
    # (p2_y - p1_y)가 음수가 아니도록 y좌표가 큰 쪽에서 작은 쪽을 빼도록 수정
    if p2_y > p1_y:
        slope = (p2_x - p1_x) / (p2_y - p1_y)
    else:
        slope = (p1_x - p2_x) / (p1_y - p2_y)
    # ======================== 수정된 부분 종료 ========================
    
    return slope