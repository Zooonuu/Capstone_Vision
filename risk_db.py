# risk_db.py

# 모델 학습 시 설정될 클래스 이름들 (Roboflow 학습 시 지정한 이름과 동일해야 함)
# 예: 0: battery, 1: coin, 2: lego
RISK_DATABASE = {
    "battery": {
        "level": 3,
        "status": "위험(Emergency)",
        "robot_action_cmd": "REMOVE",
        "mouth_action_cmd": "EMERGENCY_STOP",
        "msg": "고위험 물체(건전지) 감지! 즉시 알람"
    },
    "coin": {
        "level": 2,
        "status": "주의(Caution)",
        "robot_action_cmd": "REMOVE",
        "mouth_action_cmd": "EMERGENCY_STOP",
        "msg": "주의 물체(동전) 감지. 수거 필요."
    },
    "lego": {
        "level": 2,
        "status": "주의(Caution)",
        "robot_action_cmd": "REMOVE",
        "mouth_action_cmd": "EMERGENCY_STOP",
        "msg": "주의 물체(레고) 감지. 수거 필요."
    },
    "unknown_small_object": {
        "level": 2,
        "status": "주의(Caution)",
        "robot_action_cmd": "REMOVE",
        "mouth_action_cmd": "EMERGENCY_STOP",
        "msg": "기존 소형 위험물 bbox 기준 이하의 작은 미확인 물체 감지. 삼킴 위험 가능성으로 수거 필요."
    },
    "unknown_large_object": {
        "level": 1,
        "status": "인식(Observe)",
        "robot_action_cmd": "NONE",
        "mouth_action_cmd": "NONE",
        "msg": "입 크기보다 큰 미확인 물체 감지. 별도 행동 없이 인식만 수행."
    }
}
