# Embedded regression and bring-up images.  Keep this list explicit: adding a
# directory must not silently add a new firmware image to every build.

set(RM_EMBEDDED_TEST_DIRS
    firmware/basic/et08_sbus_demo
    firmware/basic/gimbal_demo
    firmware/basic/mecanum_chassis_demo
    firmware/basic/motor_test
    firmware/basic/omni_chassis_demo
    firmware/basic/pitch_comp_scan_demo
    firmware/basic/relay_test
    firmware/basic/remote_control_demo
    firmware/basic/shoot_test
    firmware/basic/test_dm_imu
    firmware/basic/test_imu
    firmware/basic/vt_link_monitor_demo
    firmware/hero/hero_shoot_test
    firmware/hero/integrated_demo
    firmware/hero/integrated_vt_km_demo
    firmware/infantry/infantry_chassis_et08_demo
    firmware/infantry/infantry_chassis_gimbal_et08_demo
    firmware/infantry/infantry_chassis_gimbal_et08_alt_wiring_demo
    firmware/infantry/infantry_gimbal_et08_demo
    firmware/infantry/infantry_minimal
    firmware/infantry/infantry_omni_demo
    firmware/infantry/infantry_shoot_et08_demo
    firmware/infantry/infantry_shoot_test
    firmware/leg/wheelleg_all_test
    firmware/leg/wheelleg_balance_test
    firmware/leg/wheelleg_basic_test
    firmware/leg/wheelleg_imu_test
    firmware/leg/wheelleg_joint_pos_test
    firmware/leg/wheelleg_joint_test
    firmware/leg/wheelleg_shoot_test
)

foreach(test_dir IN LISTS RM_EMBEDDED_TEST_DIRS)
    add_subdirectory(${test_dir})
endforeach()

# tests/host is intentionally a standalone host project and is never loaded by
# the ARM cross-compilation tree.
