# val_model.py
# ROS2 노드가 아니라 학습된 모델(best.pt)을 mAP/Precision/Recall 등으로 검증하는 개발 스크립트입니다.
# data.yaml / best.pt가 있는 프로젝트 루트에서 실행하는 것을 전제로 합니다.
from datetime import datetime
import json
import multiprocessing
from pathlib import Path

from ultralytics import YOLO


MODEL_PATH = 'best.pt'
DATA_PATH = 'data.yaml'
DEVICE = 'cpu'
VAL_PROJECT = Path('runs/detect')
VAL_NAME = 'val'
REPORT_DIR = Path('reports/validation')


def _safe_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _percent(value):
    value = _safe_float(value)
    if value is None:
        return None
    return round(value * 100, 2)


def _format_percent(value):
    if value is None:
        return 'N/A'
    return f'{value:.2f}%'


def _class_names_to_list(class_names):
    if isinstance(class_names, dict):
        return [name for _, name in sorted(class_names.items())]
    if isinstance(class_names, (list, tuple)):
        return list(class_names)
    return []


def _repo_relative_path(path):
    path = Path(path)
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def _write_validation_report(metrics, class_names):
    save_dir = Path(getattr(metrics, 'save_dir', Path(VAL_PROJECT) / VAL_NAME))
    save_dir.mkdir(parents=True, exist_ok=True)

    box = metrics.box
    summary = {
        'generated_at': datetime.now().isoformat(timespec='seconds'),
        'model_path': MODEL_PATH,
        'data_path': DATA_PATH,
        'save_dir': _repo_relative_path(save_dir),
        'report_dir': str(REPORT_DIR),
        'class_count': len(_class_names_to_list(class_names)),
        'metrics': {
            'precision': _percent(getattr(box, 'mp', None)),
            'recall': _percent(getattr(box, 'mr', None)),
            'map50': _percent(getattr(box, 'map50', None)),
            'map50_95': _percent(getattr(box, 'map', None)),
        },
        'artifacts': {},
    }

    artifact_names = [
        'confusion_matrix.png',
        'confusion_matrix_normalized.png',
        'BoxPR_curve.png',
        'BoxF1_curve.png',
        'BoxP_curve.png',
        'BoxR_curve.png',
    ]
    for artifact_name in artifact_names:
        artifact_path = save_dir / artifact_name
        if artifact_path.exists():
            summary['artifacts'][artifact_name] = _repo_relative_path(artifact_path)

    lines = [
        '# YOLO Validation Report',
        '',
        f'- 생성 시각: {summary["generated_at"]}',
        f'- 모델: `{summary["model_path"]}`',
        f'- 데이터셋: `{summary["data_path"]}`',
        f'- 결과 폴더: `{summary["save_dir"]}`',
        f'- 클래스 수: {summary["class_count"]}',
        '',
        '| Metric | Value |',
        '|---|---:|',
        f'| Precision | {_format_percent(summary["metrics"]["precision"])} |',
        f'| Recall | {_format_percent(summary["metrics"]["recall"])} |',
        f'| mAP50 | {_format_percent(summary["metrics"]["map50"])} |',
        f'| mAP50-95 | {_format_percent(summary["metrics"]["map50_95"])} |',
        '',
        '## Artifacts',
        '',
    ]

    if summary['artifacts']:
        for artifact_name, artifact_path in summary['artifacts'].items():
            lines.append(f'- `{artifact_name}`: `{artifact_path}`')
    else:
        lines.append('- 저장된 그래프/혼동행렬 파일을 찾지 못했습니다.')

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = REPORT_DIR / 'validation_report.json'
    md_path = REPORT_DIR / 'validation_report.md'
    save_dir_json_path = save_dir / 'validation_report.json'
    save_dir_md_path = save_dir / 'validation_report.md'

    for path in (json_path, save_dir_json_path):
        with path.open('w', encoding='utf-8') as report_file:
            json.dump(summary, report_file, ensure_ascii=False, indent=2)

    for path in (md_path, save_dir_md_path):
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8')

    return md_path, json_path, summary


def main():
    print('🚀 학습된 최고 성능 모델(best.pt)의 검증을 시작합니다...')

    model = YOLO(MODEL_PATH)
    metrics = model.val(
        data=DATA_PATH,
        device=DEVICE,
        project=str(VAL_PROJECT.resolve()),
        name=VAL_NAME,
        exist_ok=True,
        plots=True,
    )

    report_md, report_json, summary = _write_validation_report(metrics, model.names)

    print('\n✅ 검증이 완료되었습니다!')
    print('\n[ 📊 핵심 점수 ]')
    print(f'▶ Precision (정밀도)       : {_format_percent(summary["metrics"]["precision"])}')
    print(f'▶ Recall (재현율)          : {_format_percent(summary["metrics"]["recall"])}')
    print(f'▶ mAP50 (기본 정확도)      : {_format_percent(summary["metrics"]["map50"])}')
    print(f'▶ mAP50-95 (엄격한 정확도) : {_format_percent(summary["metrics"]["map50_95"])}')
    print('\n[ 📁 검증 리포트 ]')
    print(f'▶ Markdown: {report_md}')
    print(f'▶ JSON    : {report_json}')


if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
