/**
 * @file hall_driver.c
 * @brief 홀 센서 드라이버 소스 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#include "hall_driver.h"

/**
 * @brief 홀 센서의 3상 상태에 따른 기본 전기적 각도 테이블
 * @note 모터 내의 홀센서 배치에 따라 달라질 수 있으므로, 실제 하드웨어에 맞게 조정 필요
 *     - STOP은 회전 상태를 모를 때 사용. 각 섹터의 중간각
 */
static const float ANGLE_TABLE_STOP[8] = {
    0.0f, // 000 (잘못된 상태, 기본값으로 0도)
    -PI / 2.0f, // 001
    PI / 6.0f, // 010
    -PI / 6.0f, // 011
    PI * 5.0f / 6.0f, // 100
    -PI * 5.0f / 6.0f, // 101
    PI / 2.0f, // 110
    0 // 111 (잘못된 상태, 기본값으로 0도)
};
static const float ANGLE_TABLE_POS[8] = {
    0, // 000 (잘못된 상태, 기본값으로 0도)
    -PI / 6.0f, // 001
    PI / 2.0f, // 010
    PI / 6.0f, // 011
    -PI * 5.0f / 6.0f, // 100
    -PI / 2.0f, // 101
    PI * 5.0f / 6.0f, // 110
    0 // 111 (잘못된 상태, 기본값으로 0도)
};
static const float ANGLE_TABLE_NEG[8] = {
    0, // 000 (잘못된 상태, 기본값으로 0도)
    PI / 6.0f, // 001
    PI * 5.0f / 6.0f, // 010
    PI / 2.0f, // 011
    -PI / 2.0f, // 100
    -PI / 6.0f, // 101
    -PI * 5.0f / 6.0f, // 110
    0 // 111 (잘못된 상태, 기본값으로 0도)
};

/**
 * @brief 홀 센서의 3상 상태를 읽어 0~7 사이의 값으로 반환하는 내부 함수
 * @param pHandle 홀 센서 핸들러 구조체 포인터
 * @return 0~7 사이의 홀 센서 상태 값
 * @note 제작 당시 사용했던 슈미트 트리거가 반전식이라서 출력 뒤집음
 */
static uint8_t HALL_ReadRaw(sHallHandle *pHandle) {
    uint8_t state = 0;
    state |= (HAL_GPIO_ReadPin(pHandle->HallAPort, pHandle->HallAPin) == GPIO_PIN_SET) ? 0 : 4;
    state |= (HAL_GPIO_ReadPin(pHandle->HallBPort, pHandle->HallBPin) == GPIO_PIN_SET) ? 0 : 2;
    state |= (HAL_GPIO_ReadPin(pHandle->HallCPort, pHandle->HallCPin) == GPIO_PIN_SET) ? 0 : 1;
    return state;
}

/**
 * @brief 이전 홀 센서 상태와 현재 홀 센서 상태를 비교하여 회전 방향을 판단하는 내부 함수
 * @param prevState 이전 홀 센서 상태 (0~7)
 * @param currState 현재 홀 센서 상태 (0~7)
 * @return 회전 방향 (0: UNKNOWN, 1: abc순, -1: cba순)
 * @note 마찬가지로 홀센서 배치에 따라 달라질 수 있으므로, 실제 하드웨어에 맞게 조정 필요
 */
static int8_t HALL_GetDirection(uint8_t prevState, uint8_t currState) {
    if ((prevState == 2 && currState == 3) ||
        (prevState == 3 && currState == 1) ||
        (prevState == 1 && currState == 5) ||
        (prevState == 5 && currState == 4) ||
        (prevState == 4 && currState == 6) ||
        (prevState == 6 && currState == 2)) {
        return -1; // cba순
    } else if ((prevState == 3 && currState == 2) ||
               (prevState == 1 && currState == 3) ||
               (prevState == 5 && currState == 1) ||
               (prevState == 4 && currState == 5) ||
               (prevState == 6 && currState == 4) ||
               (prevState == 2 && currState == 6)) {
        return 1; // abc순
    } else {
        return 0; // 방향 판단 불가
    }
}



static float HALL_GetSectorBaseAngle(uint8_t state, int8_t direction) {
    if (state > 7) {
        return 0.0f; // 잘못된 상태, 기본값으로 0도 반환
    }

//    switch (direction) {
//        case 1: // abc순
//            return ANGLE_TABLE_POS[state];
//        case -1: // cba순
//            return ANGLE_TABLE_NEG[state];
//            break;
//        default: // 방향 판단 불가
//            return ANGLE_TABLE_STOP[state];
//            break;
//    }
//    return 0.0f; // 안전장치, 실제로는 도달하지 않아야 함
    return ANGLE_TABLE_STOP[state];
}

void HALL_Init(sHallHandle *pHandle) {
    // 내부 변수 초기화
    pHandle->Omega = 0.0f;
    pHandle->RotationDirection = 0;

    // 속도 계산용 타이머 주기 계산
    pHandle->TimerPeriodSec = (float) (pHandle->htim->Init.Prescaler + 1) / (float) (HAL_RCC_GetPCLK1Freq());

    // 최초 상태 강제 읽기 (이하 설명할 내부 함수 활용)
    pHandle->CurrentHallState = HALL_ReadRaw(pHandle);
    pHandle->PreviousHallState = pHandle->CurrentHallState;

    // 현재 상태에 맞는 기본 각도 세팅
    pHandle->ElectricalTheta = HALL_GetSectorBaseAngle(pHandle->CurrentHallState, 0);
    HAL_TIM_Base_Start(pHandle->htim); // 타이머 인터럽트 시작 (각속도 계산용)
}

volatile float debug_offset = 0.0f;
void HALL_Update(sHallHandle *pHandle) {
    uint8_t newState = HALL_ReadRaw(pHandle);

    // 디바운스 처리
    if (newState == pHandle->DebounceState) {
        pHandle->DebounceCounter++;

        // 디바운스 통과
        if (pHandle->DebounceCounter >= pHandle->DebounceRepeat) {

            // 상태 변화 감지
            if (newState != pHandle->CurrentHallState) {
                // 홀비트 최신화
                pHandle->PreviousHallState = pHandle->CurrentHallState;
                pHandle->CurrentHallState = newState;


                // 회전방향 판단
                pHandle->RotationDirection = HALL_GetDirection(pHandle->PreviousHallState, pHandle->CurrentHallState);


                // 각속도 계산
                uint32_t currentTick = __HAL_TIM_GET_COUNTER(pHandle->htim); // 현재 시간 (tick 단위)
                uint32_t lastTick = pHandle->LastUpdateTick;

                pHandle->LastUpdateTick = currentTick;

                // 타이머가 업카운트 모드라고 가정할 때, 시간 간격 계산
                uint32_t tickDiff = (currentTick - lastTick);
                float dT = tickDiff * pHandle->TimerPeriodSec; // 시간 간격 (초 단위)

                pHandle->Omega = (PI_3 * pHandle->RotationDirection) / (float) dT; // rad/s


                // 섹터별 전기각 업데이트
                pHandle->ElectricalTheta = HALL_GetSectorBaseAngle(pHandle->CurrentHallState, pHandle->RotationDirection) + debug_offset;
                if (pHandle->ElectricalTheta >= PI) {
                    pHandle->ElectricalTheta -= TWO_PI;
                }
                else if (pHandle->ElectricalTheta < -PI) {
                    pHandle->ElectricalTheta += TWO_PI;
                }
            }
        pHandle->DebounceCounter = 0; // 카운터 리셋
        }
    }
    else {
        pHandle->DebounceState = newState;
        pHandle->DebounceCounter = 0; // 카운터 리셋
    }
}

void HALL_ExtrapolateAngle(sHallHandle *pHandle, float dt_loop) {
    float deltaTheta = pHandle->Omega * dt_loop; // 각속도 * 시간 = 각도 변화량

    float newTheta = pHandle->ElectricalTheta + deltaTheta;
    while (newTheta >= PI) {
        newTheta -= TWO_PI;
    }
    while (newTheta < -PI) {
        newTheta += TWO_PI;
}

    // 전기각 보간 (Q1.31 형식으로 덧셈)
    pHandle->ElectricalTheta = newTheta;
}
