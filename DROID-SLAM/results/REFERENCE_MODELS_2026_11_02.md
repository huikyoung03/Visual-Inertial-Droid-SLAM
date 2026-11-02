# 비교 기준 보존 — 2026-11-02

이 문서는 사용자가 확정한 과거 비교 수치와 checkpoint를 보존한다.
지표는 dense full-trajectory Sim(3) ATE [m], 낮을수록 좋다.
개선율은 `(original - learned) / original * 100`. 아래 표는 소수점 6자리로
반올림한 기존 summary 기준이며 반복 실행의 통계적 유의성을 뜻하지 않는다.
Sim(3)는 전체 scale을 정렬하므로 metric scale 정확도와 구분한다.

## 두 기준 모델

발표 모델: `checkpoints/learned_attenuation_rvp_quality_pretrain_joint2000_001000.pth`
— frozen visual DROID + learned direct R/V/P IMU attenuation.

1.34% 모델: `checkpoints/learned_cross_modal_alpha_localmatched_mh03_mh04_1k_000750.pth`
— 위 temporal attenuation에 학습된 전역 R/V/P information calibration을 추가.
alpha는 수동 지정값은 아니지만 구간마다 변하는 신뢰도가 아닌 학습된 전역 3개 계수다.

| 시퀀스 (tracking stride 2) | Original | 발표 direct RVP | 개선율 | cross-modal alpha | 개선율 |
| --- | ---: | ---: | ---: | ---: | ---: |
| V1_03 | 0.018560 | 0.018467 | +0.50% | 0.018312 | +1.34% |
| V2_03 | 0.013864 | 0.014151 | -2.07% | 0.014157 | -2.11% |
| MH05 | 0.042880 | 0.042159 | +1.68% | 0.042225 | +1.53% |

Sources (repository-relative):

- `results/V1_03_FINAL_RESULT.md`
- `outputs/v2_03_visualpre7_learned_precision_1k_global_dense/summary_full.csv`
- `outputs/mh_05_final_direct_rvp_global_dense/summary_full.csv`
- `outputs/v1_03_cross_modal_alpha_localmatched750_global_dense/summary_full.csv`
- `outputs/v2_03_cross_modal_alpha_localmatched750_global_dense/summary_full.csv`
- `outputs/mh_05_cross_modal_alpha_localmatched750_global_dense/summary_full.csv`

두 모델 모두 V2_03에서 original보다 나쁘다. ATE 표만으로 그 원인이
IMU 과신인지 초기화인지 covariance/Jacobian인지 확정할 수 없다.
V1의 최고값만으로 alpha가 모든 데이터에서 우수하다고 결론 내리지 않는다.

## 발표 모델의 V1_03 입력 빈도 실험

| tracking stride | 명목 입력 Hz (원본 20 Hz) | Original | 발표 direct RVP | 개선율 |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 20 | 0.018956 | 0.019080 | -0.65% |
| 2 | 10 | 0.018560 | 0.018467 | +0.50% |
| 4 | 5 | 0.018908 | 0.018868 | +0.21% |

- 진짜 stride1: `outputs/v1_03_final_direct_rvp_actual20hz_stride1`.
- `outputs/v1_03_final_direct_rvp_20hz_stride1`은 이름과 달리 runtime log가
  tracking stride2이므로 독립적인 stride1 실험으로 세지 않는다.
- stride4: `outputs/v1_03_final_direct_rvp_5hz_stride4/summary_full.csv`에 위 결과가
  있다. 다만 같은 이름의 wrapper log는 original 실행 후 shell quote 오류로
  끝난다. 따라서 해당 log 하나만으로 learned 실행 전 과정을 증명할 수 없다.
  이 수치는 기존 결과 기록으로 보존하며, 공식 제출 시 recon의 입력/weight
  provenance를 추가 확인해야 한다.
- tracking stride와 trajectory filling은 별개다. fill stride1이면 tracking
  입력 수가 달라도 최종 dense pose 수는 V1_03에서 2149개로 같을 수 있다.

## 이미 수행한 joint visual/IMU 실험

| 기존 출력 폴더 | V1_03 dense Sim(3) ATE |
| --- | ---: |
| `v1_03_joint_visual_imu_1k_global_dense` | 0.018726 |
| `v1_03_paired_visual_gate_1k_global_dense` | 0.019290 |
| `v1_03_joint_visual_gate_fullstage_1k_global_dense` | 0.018505 |
| `v1_03_joint_factor_allocation_mh03_mh04_1k_global_dense` | 0.018767 |

각 폴더의 `summary_full.md`가 출처다. 과거 visual gate 실험이 없었던 것이
아니다. 발표 모델을 넘는 개선이 검증되지 않았다. 현재 softmax gate와 과거
scalar gate는 checkpoint 형태/학습 조건이 다르므로 같은 모델로 합쳐 말하지 않는다.

새 reciprocal 실험은 아래 설계 문서로 추적하며, 기존 결과를 덮어쓰지 않는다:
`results/RECIPROCAL_VISUAL_IMU_EXPERIMENT.md`.
