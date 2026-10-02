/*
 * bms_ctl.c - BMS Kernel Driver Control and Fault Injection Tool
 *
 * Command-line utility to interact directly with /dev/bms_drv via ioctl:
 *   - Read single cell telemetry or full pack data
 *   - Inject faults (overvoltage, undervoltage, overtemperature, etc.)
 *   - Reset active faults
 *   - Configure balancing mode
 *
 * Usage:
 *   bms_ctl status
 *   bms_ctl inject <cell_id> <fault_type> <value>
 *   bms_ctl clear
 *   bms_ctl balance <mode>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "../kernel/bms_driver.h"

#define DEVICE_PATH "/dev/bms_drv"

static void print_usage(const char *prog) {
    printf("Usage:\n");
    printf("  %s status                      - Read full pack telemetry\n", prog);
    printf("  %s cell <id>                   - Read specific cell (0-%d)\n", prog, MAX_CELLS - 1);
    printf("  %s inject <cell> <type> <val>  - Inject fault on cell\n", prog);
    printf("        fault types: 0=OV, 1=UV, 2=OT, 3=UT, 4=OC\n");
    printf("  %s clear                       - Clear all active faults\n", prog);
    printf("  %s balance <0|1|2>             - Set balance mode (0=off, 1=passive, 2=active)\n", prog);
}

static int open_device(void) {
    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Error opening " DEVICE_PATH);
        fprintf(stderr, "Ensure kernel module bms_driver.ko is loaded and permissions are set.\n");
    }
    return fd;
}

static void print_pack_status(int fd) {
    struct bms_pack_data pack;
    if (ioctl(fd, BMS_IOCTL_GET_PACK_DATA, &pack) < 0) {
        perror("ioctl BMS_IOCTL_GET_PACK_DATA failed");
        return;
    }

    printf("===============================================================\n");
    printf(" Pack Summary | Voltage: %.2f V | Current: %.2f A | Avg T: %.1f C\n",
           pack.pack_voltage_V, pack.pack_current_A, pack.avg_temperature_C);
    printf(" Active Faults: %u | Samples: %u\n",
           pack.active_fault_count, pack.sample_count);
    printf("---------------------------------------------------------------\n");
    printf(" ID | Voltage (V) | Temp (C) | Current (A) | SoC (%%) | State | Faults\n");
    printf("----+-------------+----------+-------------+---------+-------+-------\n");

    for (int i = 0; i < MAX_CELLS; ++i) {
        const char *st_str = "NORMAL";
        if (pack.cells[i].state == CELL_WARM)     st_str = "WARM";
        if (pack.cells[i].state == CELL_HOT)      st_str = "HOT";
        if (pack.cells[i].state == CELL_CRITICAL) st_str = "CRIT";
        if (pack.cells[i].state == CELL_DEAD)     st_str = "DEAD";

        char faults[32] = "";
        unsigned char f = pack.cells[i].fault_flags;
        if (f & FAULT_OVERVOLTAGE)  strcat(faults, "OV ");
        if (f & FAULT_UNDERVOLTAGE) strcat(faults, "UV ");
        if (f & FAULT_OVERTEMP)     strcat(faults, "OT ");
        if (f & FAULT_UNDERTEMP)    strcat(faults, "UT ");
        if (f & FAULT_OVERCURRENT)  strcat(faults, "OC ");
        if (strlen(faults) == 0)    strcpy(faults, "None");

        printf(" %2d |    %7.3f  |   %6.1f |    %7.2f  |  %5.1f  | %-5s | %s\n",
               pack.cells[i].cell_id,
               pack.cells[i].voltage_V,
               pack.cells[i].temperature_C,
               pack.cells[i].current_A,
               pack.cells[i].soc_percent,
               st_str,
               faults);
    }
    printf("===============================================================\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    int fd = open_device();
    if (fd < 0) return 1;

    if (strcmp(argv[1], "status") == 0) {
        print_pack_status(fd);
    } else if (strcmp(argv[1], "cell") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Missing cell ID\n");
            close(fd);
            return 1;
        }
        struct bms_cell_data cell;
        memset(&cell, 0, sizeof(cell));
        cell.cell_id = (unsigned char)atoi(argv[2]);
        if (ioctl(fd, BMS_IOCTL_GET_CELL_DATA, &cell) < 0) {
            perror("ioctl BMS_IOCTL_GET_CELL_DATA failed");
        } else {
            printf("Cell %u: Voltage=%.3f V, Temp=%.1f C, Current=%.2f A, SoC=%.1f%%, Flags=0x%02X\n",
                   cell.cell_id, cell.voltage_V, cell.temperature_C,
                   cell.current_A, cell.soc_percent, cell.fault_flags);
        }
    } else if (strcmp(argv[1], "inject") == 0) {
        if (argc < 5) {
            fprintf(stderr, "Usage: %s inject <cell_id> <fault_type> <value>\n", argv[0]);
            close(fd);
            return 1;
        }
        struct bms_fault_inject fi;
        fi.cell_id    = (unsigned char)atoi(argv[2]);
        fi.fault_type = (unsigned char)atoi(argv[3]);
        fi.value      = (float)atof(argv[4]);

        if (ioctl(fd, BMS_IOCTL_INJECT_FAULT, &fi) < 0) {
            perror("ioctl BMS_IOCTL_INJECT_FAULT failed");
            close(fd);
            return 1;
        }
        printf("Successfully injected fault (type %u, value %.2f) into cell %u\n",
               fi.fault_type, fi.value, fi.cell_id);
    } else if (strcmp(argv[1], "clear") == 0) {
        if (ioctl(fd, BMS_IOCTL_CLEAR_FAULTS) < 0) {
            perror("ioctl BMS_IOCTL_CLEAR_FAULTS failed");
            close(fd);
            return 1;
        }
        printf("Cleared all faults on driver.\n");
    } else if (strcmp(argv[1], "balance") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: %s balance <0|1|2>\n", argv[0]);
            close(fd);
            return 1;
        }
        unsigned char mode = (unsigned char)atoi(argv[2]);
        if (ioctl(fd, BMS_IOCTL_SET_BALANCE_MODE, &mode) < 0) {
            perror("ioctl BMS_IOCTL_SET_BALANCE_MODE failed");
            close(fd);
            return 1;
        }
        printf("Balancing mode set to %u\n", mode);
    } else {
        print_usage(argv[0]);
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}
