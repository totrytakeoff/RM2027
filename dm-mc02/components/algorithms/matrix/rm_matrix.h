#ifndef RM_MATRIX_H
#define RM_MATRIX_H
/* Minimal CMSIS-DSP f32 matrix declarations. Only scalar source implementations
   are compiled; no MCU header, binary DSP archive, NEON or MVE is exposed. */
#include <stdint.h>
typedef float float32_t;
typedef enum {
    ARM_MATH_SUCCESS=0, ARM_MATH_ARGUMENT_ERROR=-1, ARM_MATH_LENGTH_ERROR=-2,
    ARM_MATH_SIZE_MISMATCH=-3, ARM_MATH_NANINF=-4, ARM_MATH_SINGULAR=-5,
    ARM_MATH_TEST_FAILURE=-6, ARM_MATH_DECOMPOSITION_FAILURE=-7
} arm_status;
typedef struct { uint16_t numRows,numCols; float32_t *pData; } arm_matrix_instance_f32;
void arm_mat_init_f32(arm_matrix_instance_f32 *,uint16_t,uint16_t,float32_t *);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *,const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *,const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *,const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *);
#endif
