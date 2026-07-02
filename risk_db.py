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
        "mouth_action_cmd": "ALARM",
        "msg": "주의 물체(동전) 감지. 수거 필요."
    },
    "lego": {
        "level": 2,
        "status": "주의(Caution)",
        "robot_action_cmd": "REMOVE",
        "mouth_action_cmd": "ALARM",
        "msg": "주의 물체(레고) 감지. 수거 필요."
    }
}