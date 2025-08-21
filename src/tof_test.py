import time
import qwiic_vl53l1x

tof = qwiic_vl53l1x.QwiicVL53L1X(0x29, True)
tof.init_sensor(0x29)

while True:
    mode = int(input("mode (1: short, 2: long): "))
    imp = int(input("Inter measurement period (ms): "))
    [roi_x, roi_y, opt_center] = input("ROI, (x y optical_center): ").split()

    tof.set_distance_mode(mode)
    tof.set_inter_measurement_in_ms(imp)
    tof.set_roi(int(roi_x), int(roi_y), int(opt_center))
    tof.start_ranging()
    time.sleep(0.05)
    try:
        while True:
            print(tof.get_distance(), tof.get_roi_xy(), tof.get_timing_budget_in_ms())
            time.sleep(0.06)
    except:
        tof.stop_ranging()
