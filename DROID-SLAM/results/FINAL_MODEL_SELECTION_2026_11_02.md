# 최종 모델 선택 — 2026-11-02

평가 목적: V1_03 전체 궤적의 dense Sim(3) ATE를 original DROID보다 낮추기.
수치는 동일한 original EuRoC protocol, tracking stride 2, trajectory fill
stride 1, GT가 있는 전체 프레임을 각 궤적에 Sim(3) 정렬한 RMSE [m]이다.
Sim(3) ATE는 metric scale 정확도나 모든 시퀀스의 일반화 성능을 뜻하지 않는다.

## 선택

최종 연구 모델: `checkpoints/learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth`.
평가 시 `--use_learned_imu_cross_modal_calibration`을 반드시 활성화한다.
기존 direct R/v/p confidence head는 동결하고, 학습된 전역 R/v/p 정보 계수
3개만 추가한다. visual feature 추출기와 visual BA weight head는 변경하지 않는다.
체크포인트의 alpha는 [R, v, p] = [1.2150, 1.2126, 1.9887]이다.

발표 direct R/v/p 모델은 검증된 비교 대상 및 단순한 대조 모델로 보존한다:
`checkpoints/learned_attenuation_rvp_quality_pretrain_joint2000_001000.pth`.
새 reciprocal visual/IMU gate 500/1000 checkpoint는 최종 모델로 선택하지 않는다.

| V1_03 모델 | dense Sim(3) ATE [m] | original 대비 |
| --- | ---: | ---: |
| Original DROID | 0.018560 | 기준 |
| 발표 direct R/v/p | 0.018467 | 0.50% 개선 |
| reciprocal visual/IMU 500 | 0.018490 | 0.38% 개선 |
| reciprocal visual/IMU 1000 | 0.018591 | 0.17% 악화 |
| **선택: cross-modal alpha 750** | **0.018312** | **1.34% 개선** |

Reciprocal 500은 V1_03 metric SE(3) ATE에서 0.196750 m로 alpha의
0.197766 m보다 낮다. 그러나 original monocular DROID에는 관측 가능한
metric scale이 없으므로 이것을 original 대비 주 지표로 사용하지 않는다.
이 프로젝트에서 합의한 original 대비 선택 기준은 dense Sim(3) ATE다.

## 일반화 범위

| 시퀀스 | Original DROID | 선택 모델 | 변화 |
| --- | ---: | ---: | ---: |
| V1_03 | 0.018560 | 0.018312 | 1.34% 개선 |
| V2_03 | 0.013864 | 0.014157 | 2.11% 악화 |
| MH05 | 0.042880 | 0.042225 | 1.53% 개선 |

V1_03은 반복적인 모델 개발에 쓰였으므로 완전히 미사용인 test set이 아니다.
선택 모델이 모든 환경에서 original보다 정확하다고 주장하지 않는다.
단일 실행의 작은 차이를 통계적으로 안정적인 성능 향상이라고 주장하지 않는다.

근거 파일:

- `outputs/v1_03_cross_modal_alpha_localmatched750_global_dense/summary_full.md`
- `outputs/v2_03_cross_modal_alpha_localmatched750_global_dense/summary_full.md`
- `outputs/mh_05_cross_modal_alpha_localmatched750_global_dense/summary_full.md`
- `outputs/v1_03_reciprocal_visual_imu_500_global_dense/summary_full.md`
- `outputs/v1_03_reciprocal_visual_imu_mh03_mh04_1k_global_dense_retry/summary_full.md`

최종 모델 평가 설정은
`outputs/v1_03_cross_modal_alpha_localmatched750_global_dense/run_manifest.txt`에
기록되어 있다. local visual:3/full15DoF:2, global visual:7/full15DoF:12.
