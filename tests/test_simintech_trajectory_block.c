#include "passive_flight_simintech/SimInTechBlockApi.h"
#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum TestTrajectoryVariableIndex {
    TEST_INPUT_ALTITUDE = 0,
    TEST_INPUT_SPEED = 1,
    TEST_INPUT_OBJECT_INDEX = 2,

    TEST_OUTPUT_TIME = 3,
    TEST_OUTPUT_DOWNRANGE = 4,
    TEST_OUTPUT_ALTITUDE = 5,
    TEST_OUTPUT_SPEED = 6,
    TEST_OUTPUT_PATH_ANGLE = 7,
    TEST_OUTPUT_PITCH_ANGLE = 8,
    TEST_OUTPUT_PITCH_RATE = 9,
    TEST_OUTPUT_ATTACK_ANGLE = 10,
    TEST_OUTPUT_FALL_TIME = 11,
    TEST_OUTPUT_FINISHED = 12,
    TEST_OUTPUT_RESULT_CODE = 13,

    TEST_EXTERNAL_VARIABLE_COUNT = 14
};

static void require(
    int condition,
    const char* message
) {
    if (!condition) {
        fprintf(
            stderr,
            "Test failure: %s\n",
            message
        );

        exit(EXIT_FAILURE);
    }
}

static void requireNear(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (fabs(actual - expected) > tolerance) {
        fprintf(
            stderr,
            "Test failure: %s: "
            "actual=%.15f, expected=%.15f\n",
            message,
            actual,
            expected
        );

        exit(EXIT_FAILURE);
    }
}

int main(void) {
    int nExtVars = 0;
    int nDinVars = 0;
    int nAlgVars = 0;
    int nStateVars = 0;
    int nConsts = 0;

    int sizeOfStateVars = 0;
    int sizeOfConsts = 0;
    int sizeOfLocalVars = 0;
    int dinVarsDimension = 0;
    int algVarsDimension = 0;

    void* extVarsInfo = NULL;
    void* dinVarsInfo = NULL;
    void* algVarsInfo = NULL;
    void* stateVarsInfo = NULL;
    void* constInfo = NULL;

    solver_struct solverData;
    unsigned int schemeHash = 0;
    char algorithmName[256] = {0};
    void* algorithmObjectId = NULL;

    double values[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];
    void* addresses[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    void* locals;
    double fallTimeS;
    double expectedDownrangeM;
    double expectedFallTimeS;
    double expectedImpactSpeedMps;
    double expectedImpactPathAngleRad;
    double expectedImpactPitchAngleRad;
    double expectedImpactAttackAngleRad;
    int32_t expectedTerminationReason;
    int index;
    int result;

    memset(&solverData, 0, sizeof(solverData));

    result = INFO_FUNC(
        &nExtVars,
        &nDinVars,
        &nAlgVars,
        &nStateVars,
        &nConsts,
        &sizeOfStateVars,
        &sizeOfConsts,
        &sizeOfLocalVars,
        &dinVarsDimension,
        &algVarsDimension,
        &extVarsInfo,
        &dinVarsInfo,
        &algVarsInfo,
        &stateVarsInfo,
        &constInfo,
        &solverData,
        &schemeHash,
        algorithmName,
        &algorithmObjectId
    );

    require(
        result == r_Success,
        "INFO_FUNC failed"
    );
    require(
        nExtVars ==
            TEST_EXTERNAL_VARIABLE_COUNT,
        "Incorrect external variable count"
    );
    require(
        nDinVars == 0 &&
        nAlgVars == 0 &&
        nStateVars == 0 &&
        nConsts == 0,
        "Unexpected internal variable count"
    );
    require(
        sizeOfLocalVars > 0,
        "Local state size must be positive"
    );
    require(
        extVarsInfo != NULL,
        "External variable information is null"
    );
    require(
        solverData.TaskInfo != NULL,
        "Task information is null"
    );
    require(
        schemeHash != 0,
        "Scheme hash is zero"
    );

    locals = calloc(
        1,
        (size_t) sizeOfLocalVars
    );

    require(
        locals != NULL,
        "Cannot allocate local state"
    );

    for (index = 0;
         index < TEST_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        values[index] = 0.0;
        addresses[index] = &values[index];
    }

    values[TEST_INPUT_ALTITUDE] = 100.0;
    values[TEST_INPUT_SPEED] = 200.0;
    values[TEST_INPUT_OBJECT_INDEX] = 0.0;

    require(
        pfSimInTechCalculate(
            "FAB_1500T_POSTNIKOV_1979",
            values[TEST_INPUT_ALTITUDE],
            values[TEST_INPUT_SPEED],
            &expectedDownrangeM,
            &expectedFallTimeS,
            &expectedImpactSpeedMps,
            &expectedImpactPathAngleRad,
            &expectedImpactPitchAngleRad,
            &expectedImpactAttackAngleRad,
            &expectedTerminationReason
        ) == PF_RESULT_OK,
        "Reference C API calculation failed"
    );

    require(
        expectedTerminationReason ==
            PF_TERMINATION_GROUND_REACHED,
        "Reference calculation did not reach ground"
    );

    result = INIT_FUNC(
        0.001,
        0.0,
        addresses,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        locals,
        &solverData,
        algorithmObjectId
    );

    require(
        result == r_Success,
        "INIT_FUNC failed"
    );

    result = RUN_FUNC(
        f_InitState,
        0.001,
        0.0,
        addresses,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        locals,
        &solverData,
        algorithmObjectId
    );

    require(
        result == r_Success,
        "Initial RUN_FUNC failed"
    );

    requireNear(
        values[TEST_OUTPUT_TIME],
        0.0,
        1.0e-12,
        "Initial trajectory time is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_DOWNRANGE],
        0.0,
        1.0e-9,
        "Initial downrange is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_ALTITUDE],
        100.0,
        1.0e-6,
        "Initial altitude is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_SPEED],
        200.0,
        1.0e-6,
        "Initial speed is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_ATTACK_ANGLE],
        0.0,
        1.0e-9,
        "Initial angle of attack is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_FINISHED],
        0.0,
        0.0,
        "Trajectory must not initially be finished"
    );
    requireNear(
        values[TEST_OUTPUT_RESULT_CODE],
        (double) PF_RESULT_OK,
        0.0,
        "Initial calculation returned an error"
    );

    fallTimeS = values[TEST_OUTPUT_FALL_TIME];

    requireNear(
        fallTimeS,
        expectedFallTimeS,
        1.0e-12,
        "Fall time is incorrect"
    );

    result = RUN_FUNC(
        f_GoodStep,
        0.001,
        1.0,
        addresses,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        locals,
        &solverData,
        algorithmObjectId
    );

    require(
        result == r_Success,
        "Intermediate RUN_FUNC failed"
    );

    requireNear(
        values[TEST_OUTPUT_TIME],
        1.0,
        1.0e-9,
        "Interpolated trajectory time is incorrect"
    );
    require(
        values[TEST_OUTPUT_DOWNRANGE] > 0.0,
        "Intermediate downrange must be positive"
    );
    require(
        isfinite(
            values[TEST_OUTPUT_ALTITUDE]
        ) &&
        values[TEST_OUTPUT_ALTITUDE] > 0.0,
        "Intermediate altitude must be finite and positive"
    );
    require(
        values[TEST_OUTPUT_SPEED] > 0.0,
        "Intermediate speed must be positive"
    );
    requireNear(
        values[TEST_OUTPUT_FINISHED],
        0.0,
        0.0,
        "Trajectory finished too early"
    );

    result = RUN_FUNC(
        f_GoodStep,
        0.001,
        fallTimeS + 1.0,
        addresses,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        locals,
        &solverData,
        algorithmObjectId
    );

    require(
        result == r_Success,
        "Final RUN_FUNC failed"
    );

    requireNear(
        values[TEST_OUTPUT_TIME],
        fallTimeS,
        1.0e-9,
        "Final trajectory time must be clamped"
    );
    requireNear(
        values[TEST_OUTPUT_DOWNRANGE],
        expectedDownrangeM,
        1.0e-9,
        "Final downrange is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_ALTITUDE],
        0.0,
        1.0e-9,
        "Final altitude is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_SPEED],
        expectedImpactSpeedMps,
        1.0e-9,
        "Final speed is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_PATH_ANGLE],
        expectedImpactPathAngleRad,
        1.0e-12,
        "Final flight path angle is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_PITCH_ANGLE],
        expectedImpactPitchAngleRad,
        1.0e-12,
        "Final pitch angle is incorrect"
    );

    requireNear(
        values[TEST_OUTPUT_ATTACK_ANGLE],
        expectedImpactAttackAngleRad,
        1.0e-12,
        "Final angle of attack is incorrect"
    );

    requireNear(
        values[TEST_OUTPUT_FINISHED],
        1.0,
        0.0,
        "Trajectory finish flag is incorrect"
    );
    requireNear(
        values[TEST_OUTPUT_RESULT_CODE],
        (double) PF_RESULT_OK,
        0.0,
        "Final calculation returned an error"
    );

    result = RUN_FUNC(
        f_Stop,
        0.001,
        fallTimeS + 1.0,
        addresses,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        locals,
        &solverData,
        algorithmObjectId
    );

    require(
        result == r_Success,
        "Stop RUN_FUNC failed"
    );

    free(locals);

    printf(
        "All native SimInTech trajectory block tests passed.\n"
    );

    return EXIT_SUCCESS;
}
