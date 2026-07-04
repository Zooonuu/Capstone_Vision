# YOLO Validation Report

- 생성 시각: 2026-07-04T16:15:00
- 모델: `best.pt`
- 데이터셋: `data.yaml`
- 결과 폴더: `runs/detect/val`
- 클래스 수: 23

| Metric | Value |
|---|---:|
| Precision | 46.17% |
| Recall | 53.51% |
| mAP50 | 57.41% |
| mAP50-95 | 48.52% |

## Safety Mapping

| Risk Class | Model Classes |
|---|---|
| `lego` | `C1x1x1, C2x1x0, C2x1x1, C2x2x0, C2x2x1, C3x1x0, C3x1x1, C3x2x0, C4x1x0, C4x1x1, C4x2x0, C4x2x1, H2x2x1, P1x2x1, P2x2x1, P4x2x1, S1x1x1, S2x2x0, S2x2x1, W2x1x2, X1x2x1, X2x2x1, Z1x1x1` |

## Safety Evaluation Protocol

| 항목 | 확인 내용 |
|---|---|
| 위험 클래스 매핑 정확도 | 모델 세부 class가 `battery`/`coin`/`lego` 등 상위 위험 class로 올바르게 변환되는지 확인 |
| 안정 검출 후 수거 | 같은 bbox가 연속 프레임에서 유지될 때만 `REMOVE`로 확정되는지 확인 |
| 긴급정지 미탐률 | 입/손 근접 상황에서 `EMERGENCY_STOP`을 놓친 비율 확인 |
| 긴급정지 오탐률 | 입 근접이 아닌 상황에서 `EMERGENCY_STOP`이 잘못 발생한 비율 확인 |
| 1순위 목표 정확도 | 여러 객체 중 `risk_score`가 가장 높은 객체가 실제 최우선 위험물인지 확인 |

## Artifacts

- `confusion_matrix.png`: `runs/detect/val/confusion_matrix.png`
- `confusion_matrix_normalized.png`: `runs/detect/val/confusion_matrix_normalized.png`
- `BoxPR_curve.png`: `runs/detect/val/BoxPR_curve.png`
- `BoxF1_curve.png`: `runs/detect/val/BoxF1_curve.png`
- `BoxP_curve.png`: `runs/detect/val/BoxP_curve.png`
- `BoxR_curve.png`: `runs/detect/val/BoxR_curve.png`
