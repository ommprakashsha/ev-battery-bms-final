// SPDX-License-Identifier: GPL-2.0
/*
 * bms_driver.h - EV Battery Management System Kernel Driver Header
 *
 * Shared between kernel module (bms_driver.c) and userspace daemon.
 * Defines cell data structures and all IOCTL commands.
 */
#ifndef BMS_DRIVER_H
#define BMS_DRIVER_H

#include <linux/ioctl.h>

#define BMS_DEVICE_NAME   "bms_drv"
#define BMS_CLASS_NAME    "bms"
#define MAX_CELLS          6
#define BMS_MAGIC         'B'

/* Cell thermal/electrical state */
typedef enum {
    CELL_NORMAL   = 0,
    CELL_WARM     = 1,
    CELL_HOT      = 2,
    CELL_CRITICAL = 3,
    CELL_DEAD     = 4,
} CellState;

/* Per-cell data (kernel <-> userspace) */
struct bms_cell_data {
    unsigned char cell_id;
    float voltage_V;        /* 2.5 - 4.2 V */
    float temperature_C;    /* -20 to 80 C */
    float current_A;
    float soc_percent;
    int   state;            /* CellState enum */
    unsigned char fault_flags; /* bits: OV=0,UV=1,OT=2,UT=3,OC=4 */
};

/* Full pack summary */
struct bms_pack_data {
    struct bms_cell_data cells[MAX_CELLS];
    float pack_voltage_V;
    float pack_current_A;
    float avg_temperature_C;
    float pack_soc_percent;
    unsigned char active_fault_count;
    unsigned int sample_count;
};

/* Fault injection payload */
struct bms_fault_inject {
    unsigned char cell_id;
    unsigned char fault_type;  /* 0=OV,1=UV,2=OT,3=UT,4=OC */
    float value;
};

/* Fault flag bit positions */
#define FAULT_OVERVOLTAGE   (1 << 0)
#define FAULT_UNDERVOLTAGE  (1 << 1)
#define FAULT_OVERTEMP      (1 << 2)
#define FAULT_UNDERTEMP     (1 << 3)
#define FAULT_OVERCURRENT   (1 << 4)

/* IOCTL command definitions */
#define BMS_IOCTL_GET_PACK_DATA     _IOR(BMS_MAGIC, 1, struct bms_pack_data)
#define BMS_IOCTL_GET_CELL_DATA     _IOWR(BMS_MAGIC, 2, struct bms_cell_data)
#define BMS_IOCTL_INJECT_FAULT      _IOW(BMS_MAGIC, 3, struct bms_fault_inject)
#define BMS_IOCTL_CLEAR_FAULTS      _IO(BMS_MAGIC, 4)
#define BMS_IOCTL_SET_BALANCE_MODE  _IOW(BMS_MAGIC, 5, unsigned char)
#define BMS_IOCTL_GET_BALANCE_MODE  _IOR(BMS_MAGIC, 6, unsigned char)

#endif /* BMS_DRIVER_H */
