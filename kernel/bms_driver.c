// SPDX-License-Identifier: GPL-2.0
/*
 * bms_driver.c - EV Battery Management System Kernel Character Device Driver
 *
 * Simulates a 6-cell lithium-ion battery pack with:
 *   - Periodic simulation tick via kernel timer (200ms interval)
 *   - Voltage/temperature/SoC drift and clamping
 *   - Fault detection (OV, UV, OT, UT, OC) with flag bits
 *   - Full IOCTL interface for userspace interaction
 *   - /proc/bms_status for human-readable pack state
 *   - Fault injection support for testing
 *
 * Usage:
 *   insmod bms_driver.ko
 *   cat /proc/bms_status
 *   (use ioctl from userspace daemon or test program)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/string.h>
#include "bms_driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Omm Prakash Sha");
MODULE_DESCRIPTION("EV Battery Management System Kernel Driver");
MODULE_VERSION("1.0");

/* =========================================================================
 * Global driver state
 * ========================================================================= */

/* Full battery pack state — all access protected by bms_spinlock */
static struct bms_pack_data g_pack;

/* Spinlock protecting g_pack and g_balance_mode from concurrent access */
static spinlock_t bms_spinlock;

/* Character device infrastructure */
static int            g_major;
static struct class  *g_class;
static struct device *g_device;
static struct cdev    g_cdev;

/* Periodic simulation timer (fires every 200 ms) */
static struct timer_list g_sim_timer;

/*
 * Cell balancing mode:
 *   0 = off
 *   1 = passive (resistive dissipation)
 *   2 = active (energy redistribution)
 */
static unsigned char g_balance_mode;

/* /proc entry handle */
static struct proc_dir_entry *g_proc_entry;

/* =========================================================================
 * Fixed-point float helpers
 *
 * The kernel does not use FPU by default. We emulate floating-point
 * arithmetic using scaled integers (millivolts, milli-degrees, etc.).
 * All bms_cell_data float fields are stored as their bit patterns in
 * an int-sized IEEE 754 union — safe because we only read/write them
 * from kernel context with kernel_fpu_begin/end OR we use a soft-float
 * integer representation below.
 *
 * Strategy: store values as scaled integers internally, convert to the
 * float bit-pattern expected by the struct only when writing out.
 * Voltages : stored as millivolts (mV),  1 V = 1000 units
 * Temps    : stored as centi-degrees (cdeg), 1 °C = 100 units
 * SoC      : stored as centi-percent (cpct), 1 % = 100 units
 * Current  : stored as centi-amps (cA),  1 A = 100 units
 * ========================================================================= */

/*
 * Encode a scaled integer as an IEEE 754 single-precision float.
 * scale is the divisor that converts the integer to the real value.
 * e.g.  int_to_float_bits(3700, 1000) => bits for 3.700f
 *
 * We perform the conversion using the kernel's soft-float support
 * by keeping fp operations inside kernel_fpu_begin()/kernel_fpu_end()
 * on x86, or by simply doing the bit-pattern math manually.
 *
 * For portability across arches we use a union trick and __kernel_fpu
 * only where available.  Since this is a simulation driver we accept
 * the limitation that the float values in the struct are approximations
 * computed via integer arithmetic without hardware FPU.
 */

/* Simple Q16.16 fixed-point: value = raw / 65536 */

/* Convert milli-units to a float bit-pattern stored as u32.
 * We use the identity:  f = integer_part + fraction_part/1000
 * then encode as IEEE 754 manually. */

/**
 * mk_float_bits - Build IEEE 754 float from integer and fractional parts.
 * @integer:    Integer part (may be negative).
 * @frac_thou:  Fractional part in thousandths (0-999), always positive.
 *
 * Returns the 32-bit IEEE 754 bit pattern for (integer + frac_thou/1000).
 * Supports the range [-999, 9999] which covers all BMS quantities.
 *
 * NOTE: This produces an approximation; for production use kernel_fpu_begin.
 */
static u32 mk_float_bits(int integer, unsigned int frac_thou)
{
	u32 bits;
	int sign = 0;
	unsigned int abs_int;
	unsigned int mantissa;
	int exponent;
	unsigned int frac_bits;

	/* Handle sign */
	if (integer < 0) {
		sign = 1;
		abs_int = (unsigned int)(-integer);
	} else {
		abs_int = (unsigned int)integer;
	}

	/* Clamp fractional part */
	if (frac_thou >= 1000)
		frac_thou = 999;

	/*
	 * Represent value as abs_int + frac_thou/1000.
	 * Scale to a 24-bit mantissa.
	 *
	 * Approach: compute in units of 1/1024 (shifts instead of divides)
	 * frac_thou/1000 ≈ frac_thou * 1024 / 1000 / 1024
	 */

	if (abs_int == 0 && frac_thou == 0) {
		/* Encode ±0 */
		return (u32)sign << 31;
	}

	/* Find exponent: the position of the highest set bit of the integer part */
	if (abs_int >= 1) {
		unsigned int tmp = abs_int;
		exponent = 0;
		while (tmp > 1) { tmp >>= 1; exponent++; }
		/* Value = 1.xxx * 2^exponent  (abs_int in range [2^exp, 2^(exp+1)-1]) */

		/*
		 * Mantissa: 23 explicit bits after the implicit leading 1.
		 * Lower bits come from the fractional part.
		 */
		mantissa = abs_int - (1u << exponent); /* remove implicit 1 */
		mantissa <<= (23 - exponent);          /* shift to fill 23 bits */

		/* Fill remaining bits from fraction */
		frac_bits = (frac_thou << (23 - exponent)) / 1000;
		/* Protect against exponent == 23 */
		if (exponent < 23)
			mantissa |= frac_bits & ((1u << (23 - exponent)) - 1u);

	} else {
		/*
		 * abs_int == 0, value is purely fractional: 0.frac_thou
		 * Find leading bit of frac_thou/1000 expressed in binary.
		 * 0.5 = 0x3F000000, 0.25 = 0x3E800000, etc.
		 */
		unsigned int f = frac_thou; /* range 1-999 */
		/*
		 * Multiply by 2 until >= 1000 (i.e. >= 1.0), counting shifts.
		 * Each shift is a multiply by 2 (add an exponent of -1).
		 */
		exponent = -1;
		while (f < 1000 && exponent > -127) {
			f <<= 1;
			exponent--;
		}
		/* f is now in range [1000, 2000), represents 1.xxx */
		f -= 1000; /* remove implicit 1 */
		mantissa = (f * (1u << 23)) / 1000;
	}

	/* IEEE 754 single: [sign:1][exponent+127:8][mantissa:23] */
	bits = ((u32)sign << 31) |
	       ((u32)((exponent + 127) & 0xFF) << 23) |
	       (mantissa & 0x7FFFFFu);

	return bits;
}

/**
 * encode_float - Write a scaled-integer value into a float field.
 * @dest:   Pointer to the float field in the struct.
 * @milli:  Value in milli-units (divide by 1000 to get real value).
 *
 * Handles negative values (e.g. negative temperatures).
 */
static void encode_float(float *dest, int milli)
{
	int integer  = milli / 1000;
	int frac     = milli % 1000;
	u32 bits;

	if (frac < 0)
		frac = -frac;

	bits = mk_float_bits(integer, (unsigned int)frac);
	/* Write bit-pattern into the float field */
	memcpy(dest, &bits, sizeof(u32));
}

/* =========================================================================
 * Simulation state (internal, integer-scaled)
 * ========================================================================= */

/*
 * Per-cell internal simulation state.
 * All values use integer scaled representations to avoid FPU.
 *   voltage_mv   : millivolts        (2500 - 4200)
 *   temp_cdeg    : centi-degrees C   (-2000 to 8000)
 *   soc_cpct     : centi-percent SoC (0 - 10000)
 *   current_ca   : centi-amps        (negative = discharge)
 */
struct cell_sim {
	int voltage_mv;
	int temp_cdeg;
	int soc_cpct;
	int current_ca;
};

static struct cell_sim g_cells_sim[MAX_CELLS];

/* =========================================================================
 * Simulation core
 * ========================================================================= */

/**
 * clamp_int - Clamp an integer value between min and max (inclusive).
 */
static inline int clamp_int(int val, int min, int max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

/**
 * bms_update_pack_aggregates - Recompute pack-level summary from cell data.
 *
 * Must be called with bms_spinlock held.
 */
static void bms_update_pack_aggregates(void)
{
	int i;
	int sum_voltage_mv  = 0;
	int sum_temp_cdeg   = 0;
	int sum_soc_cpct    = 0;
	int sum_current_ca  = 0;
	unsigned char fault_count = 0;

	for (i = 0; i < MAX_CELLS; i++) {
		sum_voltage_mv += g_cells_sim[i].voltage_mv;
		sum_temp_cdeg  += g_cells_sim[i].temp_cdeg;
		sum_soc_cpct   += g_cells_sim[i].soc_cpct;
		sum_current_ca += g_cells_sim[i].current_ca;
		if (g_pack.cells[i].fault_flags)
			fault_count++;
	}

	/* Pack voltage = sum of series cells */
	encode_float(&g_pack.pack_voltage_V,  sum_voltage_mv);
	/* Pack current = average (parallel sensing) */
	encode_float(&g_pack.pack_current_A,  sum_current_ca / MAX_CELLS);
	/* Average temperature */
	encode_float(&g_pack.avg_temperature_C, sum_temp_cdeg / MAX_CELLS);
	/* Average SoC */
	encode_float(&g_pack.pack_soc_percent,  sum_soc_cpct / MAX_CELLS);

	g_pack.active_fault_count = fault_count;
}

/**
 * bms_simulate_tick - Advance simulation by one time step (200 ms).
 *
 * Updates each cell's voltage, temperature, SoC, and current using
 * random noise and simple physics models.  Detects faults and updates
 * the public bms_pack_data struct.
 *
 * Called from timer callback — must not sleep.
 */
static void bms_simulate_tick(void)
{
	unsigned long flags;
	int i;
	u8 rand_byte;

	spin_lock_irqsave(&bms_spinlock, flags);

	for (i = 0; i < MAX_CELLS; i++) {
		struct cell_sim      *sim  = &g_cells_sim[i];
		struct bms_cell_data *cell = &g_pack.cells[i];

		/* ---- Voltage drift ±3 mV (±0.003 V) ---- */
		get_random_bytes(&rand_byte, 1);
		/* rand_byte in [0,255]; centre at 128 => delta in [-128,127] */
		/* Scale: 128 maps to 3 mV => delta_mv = (rand_byte - 128) * 3 / 128 */
		{
			int delta_mv = ((int)rand_byte - 128) * 3 / 128;
			sim->voltage_mv += delta_mv;
		}

		/* ---- Temperature model ---- */
		/* Ambient = 2500 cdeg (25.00 °C) */
		/* Heating: I^2 * R effect — use |current_ca| / 100 as proxy */
		{
			int abs_curr = sim->current_ca < 0
			             ? -sim->current_ca
			             : sim->current_ca;
			/* Heating: +0.01 °C per 1 A per tick = 1 cdeg per 100 cA */
			int heat_cdeg = abs_curr / 100;
			/* Cooling toward ambient (25°C = 2500 cdeg):
			 *   delta = (ambient - temp) / 500 per tick  */
			int cool_cdeg = (2500 - sim->temp_cdeg) / 500;

			sim->temp_cdeg += heat_cdeg + cool_cdeg;
		}

		/* ---- SoC discharge: -0.001 % per tick = -0.1 cpct per tick ---- */
		sim->soc_cpct -= 10; /* 0.10 cpct = 0.001 % */

		/* ---- Simulate small random current variation ---- */
		get_random_bytes(&rand_byte, 1);
		{
			/* Current wanders around -5 A (discharge) ±2 A */
			int delta_ca = ((int)rand_byte - 128) * 200 / 128;
			sim->current_ca += delta_ca;
			/* Keep near discharge baseline of -500 cA (-5 A) */
			sim->current_ca = (sim->current_ca * 9 + (-500)) / 10;
		}

		/* ---- Clamp all values ---- */
		sim->voltage_mv = clamp_int(sim->voltage_mv, 2500, 4200);
		sim->temp_cdeg  = clamp_int(sim->temp_cdeg, -2000, 8000);
		sim->soc_cpct   = clamp_int(sim->soc_cpct, 0, 10000);
		sim->current_ca = clamp_int(sim->current_ca, -10000, 10000);

		/* ---- Encode float fields into the public struct ---- */
		encode_float(&cell->voltage_V,      sim->voltage_mv);
		encode_float(&cell->temperature_C,  sim->temp_cdeg);
		encode_float(&cell->soc_percent,    sim->soc_cpct);
		encode_float(&cell->current_A,      sim->current_ca);

		/* ---- Fault detection ---- */
		cell->fault_flags = 0;

		/* OV: voltage > 4.15 V = 4150 mV */
		if (sim->voltage_mv > 4150)
			cell->fault_flags |= FAULT_OVERVOLTAGE;

		/* UV: voltage < 2.90 V = 2900 mV */
		if (sim->voltage_mv < 2900)
			cell->fault_flags |= FAULT_UNDERVOLTAGE;

		/* OT: temperature > 65.00 °C = 6500 cdeg */
		if (sim->temp_cdeg > 6500)
			cell->fault_flags |= FAULT_OVERTEMP;

		/* UT: temperature < -10.00 °C = -1000 cdeg */
		if (sim->temp_cdeg < -1000)
			cell->fault_flags |= FAULT_UNDERTEMP;

		/* OC: |current| > 30 A = 3000 cA */
		{
			int abs_curr = sim->current_ca < 0
			             ? -sim->current_ca
			             : sim->current_ca;
			if (abs_curr > 3000)
				cell->fault_flags |= FAULT_OVERCURRENT;
		}

		/* ---- Determine CellState from fault flags and temperature ---- */
		if (cell->fault_flags & (FAULT_OVERVOLTAGE | FAULT_UNDERVOLTAGE |
		                         FAULT_OVERTEMP    | FAULT_OVERCURRENT)) {
			/* Multiple critical faults → DEAD */
			int nfaults = 0;
			unsigned char ff = cell->fault_flags;
			while (ff) { nfaults += (ff & 1); ff >>= 1; }
			if (nfaults >= 2)
				cell->state = CELL_DEAD;
			else
				cell->state = CELL_CRITICAL;
		} else if (sim->temp_cdeg > 5000) { /* > 50 °C */
			cell->state = CELL_HOT;
		} else if (sim->temp_cdeg > 3500) { /* > 35 °C */
			cell->state = CELL_WARM;
		} else {
			cell->state = CELL_NORMAL;
		}
	}

	/* ---- Update pack-level aggregates ---- */
	bms_update_pack_aggregates();
	g_pack.sample_count++;

	spin_unlock_irqrestore(&bms_spinlock, flags);
}

/* =========================================================================
 * Timer callback
 * ========================================================================= */

/**
 * bms_timer_callback - Kernel timer handler: tick simulation and reschedule.
 * @t: Pointer to the timer_list (unused directly; g_sim_timer is global).
 */
static void bms_timer_callback(struct timer_list *t)
{
	bms_simulate_tick();

	/* Reschedule for the next 200 ms tick */
	mod_timer(&g_sim_timer, jiffies + msecs_to_jiffies(200));
}

/* =========================================================================
 * Character device file operations
 * ========================================================================= */

/**
 * bms_open - Called when userspace opens /dev/bms_drv.
 */
static int bms_open(struct inode *inode, struct file *filp)
{
	pr_info("bms_driver: device opened by PID %d\n", current->pid);
	return 0;
}

/**
 * bms_release - Called when userspace closes /dev/bms_drv.
 */
static int bms_release(struct inode *inode, struct file *filp)
{
	pr_info("bms_driver: device closed by PID %d\n", current->pid);
	return 0;
}

/**
 * bms_read - Copy raw pack data bytes to userspace.
 *
 * Allows a simple `read()` syscall to obtain a snapshot of the full
 * bms_pack_data struct without using ioctl.
 */
static ssize_t bms_read(struct file *filp, char __user *buf,
                         size_t count, loff_t *f_pos)
{
	unsigned long flags;
	struct bms_pack_data snapshot;
	size_t to_copy;

	/* Only serve reads from the beginning */
	if (*f_pos >= (loff_t)sizeof(snapshot))
		return 0;

	/* Snapshot under lock */
	spin_lock_irqsave(&bms_spinlock, flags);
	memcpy(&snapshot, &g_pack, sizeof(snapshot));
	spin_unlock_irqrestore(&bms_spinlock, flags);

	to_copy = sizeof(snapshot) - (size_t)*f_pos;
	if (to_copy > count)
		to_copy = count;

	if (copy_to_user(buf, (char *)&snapshot + *f_pos, to_copy))
		return -EFAULT;

	*f_pos += to_copy;
	return (ssize_t)to_copy;
}

/**
 * bms_ioctl - Handle all IOCTL commands from userspace.
 */
static long bms_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	int ret = 0;

	switch (cmd) {

	/* ---- GET_PACK_DATA: return full pack snapshot ---- */
	case BMS_IOCTL_GET_PACK_DATA: {
		struct bms_pack_data snapshot;

		spin_lock_irqsave(&bms_spinlock, flags);
		memcpy(&snapshot, &g_pack, sizeof(snapshot));
		spin_unlock_irqrestore(&bms_spinlock, flags);

		if (copy_to_user((void __user *)arg, &snapshot, sizeof(snapshot)))
			return -EFAULT;
		break;
	}

	/* ---- GET_CELL_DATA: userspace provides cell_id, we fill the rest ---- */
	case BMS_IOCTL_GET_CELL_DATA: {
		struct bms_cell_data cell_out;
		unsigned char cid;

		/* Read just the cell_id field (first byte of struct) */
		if (get_user(cid, (unsigned char __user *)arg))
			return -EFAULT;

		if (cid >= MAX_CELLS)
			return -EINVAL;

		spin_lock_irqsave(&bms_spinlock, flags);
		memcpy(&cell_out, &g_pack.cells[cid], sizeof(cell_out));
		spin_unlock_irqrestore(&bms_spinlock, flags);

		if (copy_to_user((void __user *)arg, &cell_out, sizeof(cell_out)))
			return -EFAULT;
		break;
	}

	/* ---- INJECT_FAULT: override a cell's measurement to trigger fault ---- */
	case BMS_IOCTL_INJECT_FAULT: {
		struct bms_fault_inject inj;
		u32 val_bits;
		int val_milli; /* value converted to milli-units */

		if (copy_from_user(&inj, (void __user *)arg, sizeof(inj)))
			return -EFAULT;

		if (inj.cell_id >= MAX_CELLS)
			return -EINVAL;

		/*
		 * Extract the float value as its raw IEEE 754 bits,
		 * then approximate milli-units from the exponent/mantissa.
		 * For injection we perform a rough conversion:
		 *   We read the float bit-pattern and use sign+exponent+mantissa
		 *   to produce an integer in milli-units.
		 *
		 * Simple approach: treat inj.value bits as u32 and decode.
		 */
		memcpy(&val_bits, &inj.value, sizeof(u32));
		{
			/* IEEE 754 decode: val ≈ sign * 2^(exp-127) * (1 + mant/2^23) */
			int sign_f     = (val_bits >> 31) ? -1 : 1;
			int exp_f      = (int)((val_bits >> 23) & 0xFF) - 127;
			unsigned int mant_f = val_bits & 0x7FFFFFu;
			int whole;

			if (exp_f < 0) {
				whole = 0; /* < 1.0, treat as 0 for milli */
			} else if (exp_f > 13) {
				whole = 9999; /* too large, clamp */
			} else {
				whole = (int)(1u << exp_f) +
				        (int)((mant_f >> (23 - exp_f)));
			}
			/* Fractional milli part: (mant_f % (1<<(23-exp_f))) * 1000 / (1<<(23-exp_f)) */
			{
				unsigned int frac_bits_rem;
				unsigned int frac_milli = 0;
				if (exp_f >= 0 && exp_f < 23) {
					frac_bits_rem = mant_f & ((1u << (23 - exp_f)) - 1u);
					frac_milli = frac_bits_rem * 1000u >> (23 - exp_f);
				}
				val_milli = sign_f * (whole * 1000 + (int)frac_milli);
			}
		}

		spin_lock_irqsave(&bms_spinlock, flags);
		switch (inj.fault_type) {
		case 0: /* OV — set voltage high */
			g_cells_sim[inj.cell_id].voltage_mv = val_milli;
			encode_float(&g_pack.cells[inj.cell_id].voltage_V, val_milli);
			pr_info("bms_driver: inject OV fault on cell %d: %d mV\n",
			        inj.cell_id, val_milli);
			break;
		case 1: /* UV — set voltage low */
			g_cells_sim[inj.cell_id].voltage_mv = val_milli;
			encode_float(&g_pack.cells[inj.cell_id].voltage_V, val_milli);
			pr_info("bms_driver: inject UV fault on cell %d: %d mV\n",
			        inj.cell_id, val_milli);
			break;
		case 2: /* OT — set temperature high */
			g_cells_sim[inj.cell_id].temp_cdeg = val_milli;
			encode_float(&g_pack.cells[inj.cell_id].temperature_C, val_milli);
			pr_info("bms_driver: inject OT fault on cell %d: %d cdeg\n",
			        inj.cell_id, val_milli);
			break;
		case 3: /* UT — set temperature low */
			g_cells_sim[inj.cell_id].temp_cdeg = val_milli;
			encode_float(&g_pack.cells[inj.cell_id].temperature_C, val_milli);
			pr_info("bms_driver: inject UT fault on cell %d: %d cdeg\n",
			        inj.cell_id, val_milli);
			break;
		case 4: /* OC — set current high */
			g_cells_sim[inj.cell_id].current_ca = val_milli;
			encode_float(&g_pack.cells[inj.cell_id].current_A, val_milli);
			pr_info("bms_driver: inject OC fault on cell %d: %d cA\n",
			        inj.cell_id, val_milli);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		spin_unlock_irqrestore(&bms_spinlock, flags);
		break;
	}

	/* ---- CLEAR_FAULTS: zero all fault flags, reset states ---- */
	case BMS_IOCTL_CLEAR_FAULTS: {
		int i;
		spin_lock_irqsave(&bms_spinlock, flags);
		for (i = 0; i < MAX_CELLS; i++) {
			g_pack.cells[i].fault_flags = 0;
			g_pack.cells[i].state       = CELL_NORMAL;
		}
		g_pack.active_fault_count = 0;
		spin_unlock_irqrestore(&bms_spinlock, flags);
		pr_info("bms_driver: all faults cleared\n");
		break;
	}

	/* ---- SET_BALANCE_MODE: store balance mode byte ---- */
	case BMS_IOCTL_SET_BALANCE_MODE: {
		unsigned char mode;
		if (get_user(mode, (unsigned char __user *)arg))
			return -EFAULT;
		if (mode > 2)
			return -EINVAL;
		spin_lock_irqsave(&bms_spinlock, flags);
		g_balance_mode = mode;
		spin_unlock_irqrestore(&bms_spinlock, flags);
		pr_info("bms_driver: balance mode set to %u\n", (unsigned)mode);
		break;
	}

	/* ---- GET_BALANCE_MODE: return current balance mode byte ---- */
	case BMS_IOCTL_GET_BALANCE_MODE: {
		unsigned char mode;
		spin_lock_irqsave(&bms_spinlock, flags);
		mode = g_balance_mode;
		spin_unlock_irqrestore(&bms_spinlock, flags);
		if (put_user(mode, (unsigned char __user *)arg))
			return -EFAULT;
		break;
	}

	default:
		return -ENOTTY;
	}

	return ret;
}

/* File operations table for the character device */
static const struct file_operations bms_fops = {
	.owner          = THIS_MODULE,
	.open           = bms_open,
	.release        = bms_release,
	.read           = bms_read,
	.unlocked_ioctl = bms_ioctl,
};

/* =========================================================================
 * /proc/bms_status — seq_file interface
 * ========================================================================= */

/**
 * state_name - Return a human-readable string for a CellState value.
 */
static const char *state_name(int state)
{
	switch (state) {
	case CELL_NORMAL:   return "NORMAL  ";
	case CELL_WARM:     return "WARM    ";
	case CELL_HOT:      return "HOT     ";
	case CELL_CRITICAL: return "CRITICAL";
	case CELL_DEAD:     return "DEAD    ";
	default:            return "UNKNOWN ";
	}
}

/**
 * bms_proc_show - seq_file show callback for /proc/bms_status.
 *
 * Prints a formatted table of all cell states plus a pack summary.
 * Float values are displayed using the integer simulation state to
 * avoid FPU in proc context.
 */
static int bms_proc_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	struct cell_sim cells_snap[MAX_CELLS];
	struct bms_pack_data pack_snap;
	int i;

	/* Snapshot everything under lock */
	spin_lock_irqsave(&bms_spinlock, flags);
	memcpy(cells_snap, g_cells_sim, sizeof(cells_snap));
	memcpy(&pack_snap,  &g_pack,     sizeof(pack_snap));
	spin_unlock_irqrestore(&bms_spinlock, flags);

	/* Header */
	seq_printf(m, "=== EV Battery Management System — Pack Status ===\n");
	seq_printf(m, "Balance mode : %u  (%s)\n",
	           (unsigned)g_balance_mode,
	           g_balance_mode == 0 ? "OFF" :
	           g_balance_mode == 1 ? "PASSIVE" : "ACTIVE");
	seq_printf(m, "Sample count : %u\n\n", pack_snap.sample_count);

	/* Column headers */
	seq_printf(m, "%-6s %-12s %-12s %-12s %-12s %-10s %-8s\n",
	           "CellID", "Voltage(V)", "Temp(C)", "Current(A)",
	           "SoC(%)", "State", "Faults");
	seq_printf(m, "%-6s %-12s %-12s %-12s %-12s %-10s %-8s\n",
	           "------", "----------", "-------", "----------",
	           "------", "--------", "------");

	/* One row per cell — display integer-scaled values as decimals */
	for (i = 0; i < MAX_CELLS; i++) {
		struct cell_sim      *s = &cells_snap[i];
		struct bms_cell_data *c = &pack_snap.cells[i];
		int v_int  =  s->voltage_mv  / 1000;
		int v_frac = (s->voltage_mv  < 0 ? -s->voltage_mv  : s->voltage_mv)  % 1000;
		int t_int  =  s->temp_cdeg   / 100;
		int t_frac = (s->temp_cdeg   < 0 ? -s->temp_cdeg   : s->temp_cdeg)   % 100;
		int i_int  =  s->current_ca  / 100;
		int i_frac = (s->current_ca  < 0 ? -s->current_ca  : s->current_ca)  % 100;
		int sc_int =  s->soc_cpct    / 100;
		int sc_frac= (s->soc_cpct    < 0 ? -s->soc_cpct    : s->soc_cpct)    % 100;

		/* Print sign for negatives */
		seq_printf(m,
		           "%-6u %c%d.%03d      %c%d.%02d       %c%d.%02d       %d.%02d        %-10s 0x%02X\n",
		           (unsigned)c->cell_id,
		           (s->voltage_mv  < 0 ? '-' : ' '), v_int, v_frac,
		           (s->temp_cdeg   < 0 ? '-' : ' '), t_int, t_frac,
		           (s->current_ca  < 0 ? '-' : ' '), i_int, i_frac,
		           sc_int, sc_frac,
		           state_name(c->state),
		           (unsigned)c->fault_flags);
	}

	/* Pack summary */
	{
		/* Pack voltage = sum of cell voltages (series) */
		int pv_sum_mv = 0;
		int pt_sum_cdeg = 0;
		int ps_sum_cpct = 0;
		for (i = 0; i < MAX_CELLS; i++) {
			pv_sum_mv   += cells_snap[i].voltage_mv;
			pt_sum_cdeg += cells_snap[i].temp_cdeg;
			ps_sum_cpct += cells_snap[i].soc_cpct;
		}
		seq_printf(m, "\n--- Pack Summary ---\n");
		seq_printf(m, "  Pack Voltage   : %d.%03d V\n",
		           pv_sum_mv / 1000, pv_sum_mv % 1000);
		seq_printf(m, "  Avg Temp       : %d.%02d C\n",
		           (pt_sum_cdeg / MAX_CELLS) / 100,
		           (pt_sum_cdeg / MAX_CELLS) % 100);
		seq_printf(m, "  Avg SoC        : %d.%02d %%\n",
		           (ps_sum_cpct / MAX_CELLS) / 100,
		           (ps_sum_cpct / MAX_CELLS) % 100);
		seq_printf(m, "  Active Faults  : %u\n",
		           (unsigned)pack_snap.active_fault_count);
	}

	return 0;
}

/**
 * bms_proc_open - Open handler for /proc/bms_status.
 */
static int bms_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bms_proc_show, NULL);
}

/* proc_ops for kernels >= 5.6; fall back to file_operations for older */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops bms_proc_ops = {
	.proc_open    = bms_proc_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations bms_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = bms_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};
#endif

/* =========================================================================
 * Module init / exit
 * ========================================================================= */

/**
 * bms_init - Module initialisation.
 *
 * Sets up:
 *   1. Spinlock
 *   2. Initial cell simulation state (6 cells, realistic starting values)
 *   3. Character device (major number, cdev, class, device node)
 *   4. /proc/bms_status
 *   5. Simulation timer
 */
static int __init bms_init(void)
{
	int i;
	int ret;
	dev_t dev_num;

	pr_info("bms_driver: initialising EV BMS kernel driver v1.0\n");

	/* ---- 1. Initialise spinlock ---- */
	spin_lock_init(&bms_spinlock);

	/* ---- 2. Set up initial cell state ---- */
	memset(&g_pack, 0, sizeof(g_pack));

	/*
	 * Realistic starting values for a 6S Li-ion pack:
	 *   Voltage : ~3.70 V  (mid-SoC resting)
	 *   Temp    : ~25.0 °C
	 *   SoC     : ~85.0 %
	 *   Current : -5.0 A   (gentle discharge)
	 *
	 * Slight per-cell offsets create natural imbalance.
	 */
	for (i = 0; i < MAX_CELLS; i++) {
		/* voltage_mv: 3700 + i*10  (3700, 3710, ..., 3750) */
		g_cells_sim[i].voltage_mv = 3700 + i * 10;
		/* temp_cdeg: 2500 - i*50  (25.00, 24.50, ..., 22.50 °C) */
		g_cells_sim[i].temp_cdeg  = 2500 - i * 50;
		/* soc_cpct: 8500 - i*20   (85.00, 84.80, ..., 84.00 %) */
		g_cells_sim[i].soc_cpct   = 8500 - i * 20;
		/* current_ca: -500 cA = -5.0 A (discharge) */
		g_cells_sim[i].current_ca = -500;

		/* Populate the public struct */
		g_pack.cells[i].cell_id = (unsigned char)i;
		encode_float(&g_pack.cells[i].voltage_V,     g_cells_sim[i].voltage_mv);
		encode_float(&g_pack.cells[i].temperature_C, g_cells_sim[i].temp_cdeg);
		encode_float(&g_pack.cells[i].soc_percent,   g_cells_sim[i].soc_cpct);
		encode_float(&g_pack.cells[i].current_A,     g_cells_sim[i].current_ca);
		g_pack.cells[i].state       = CELL_NORMAL;
		g_pack.cells[i].fault_flags = 0;

		pr_info("bms_driver:   cell[%d] init: %d mV, %d cdeg, %d cpct\n",
		        i,
		        g_cells_sim[i].voltage_mv,
		        g_cells_sim[i].temp_cdeg,
		        g_cells_sim[i].soc_cpct);
	}

	bms_update_pack_aggregates();
	g_pack.sample_count = 0;

	/* ---- 3. Register character device ---- */

	/* Allocate major number dynamically */
	ret = alloc_chrdev_region(&dev_num, 0, 1, BMS_DEVICE_NAME);
	if (ret < 0) {
		pr_err("bms_driver: alloc_chrdev_region failed (%d)\n", ret);
		return ret;
	}
	g_major = MAJOR(dev_num);
	pr_info("bms_driver: registered chrdev major=%d\n", g_major);

	/* Initialise and add the cdev */
	cdev_init(&g_cdev, &bms_fops);
	g_cdev.owner = THIS_MODULE;
	ret = cdev_add(&g_cdev, dev_num, 1);
	if (ret < 0) {
		pr_err("bms_driver: cdev_add failed (%d)\n", ret);
		goto err_unregister_chrdev;
	}
	pr_info("bms_driver: cdev added\n");

	/* Create device class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	g_class = class_create(BMS_CLASS_NAME);
#else
	g_class = class_create(THIS_MODULE, BMS_CLASS_NAME);
#endif
	if (IS_ERR(g_class)) {
		ret = PTR_ERR(g_class);
		pr_err("bms_driver: class_create failed (%d)\n", ret);
		goto err_cdev_del;
	}
	pr_info("bms_driver: device class '%s' created\n", BMS_CLASS_NAME);

	/* Create the device node /dev/bms_drv */
	g_device = device_create(g_class, NULL,
	                          MKDEV(g_major, 0),
	                          NULL, BMS_DEVICE_NAME);
	if (IS_ERR(g_device)) {
		ret = PTR_ERR(g_device);
		pr_err("bms_driver: device_create failed (%d)\n", ret);
		goto err_class_destroy;
	}
	pr_info("bms_driver: device /dev/%s created\n", BMS_DEVICE_NAME);

	/* ---- 4. Create /proc/bms_status ---- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	g_proc_entry = proc_create("bms_status", 0444, NULL, &bms_proc_ops);
#else
	g_proc_entry = proc_create("bms_status", 0444, NULL, &bms_proc_fops);
#endif
	if (!g_proc_entry) {
		pr_err("bms_driver: proc_create failed\n");
		ret = -ENOMEM;
		goto err_device_destroy;
	}
	pr_info("bms_driver: /proc/bms_status created\n");

	/* ---- 5. Start simulation timer (200 ms periodic) ---- */
	timer_setup(&g_sim_timer, bms_timer_callback, 0);
	ret = mod_timer(&g_sim_timer, jiffies + msecs_to_jiffies(200));
	if (ret) {
		/* mod_timer returns 1 if timer was already pending — not an error */
		pr_info("bms_driver: timer rescheduled (was pending)\n");
	}
	pr_info("bms_driver: simulation timer started (200 ms interval)\n");

	pr_info("bms_driver: initialisation complete\n");
	return 0;

	/* Error unwind path */
err_device_destroy:
	device_destroy(g_class, MKDEV(g_major, 0));
err_class_destroy:
	class_destroy(g_class);
err_cdev_del:
	cdev_del(&g_cdev);
err_unregister_chrdev:
	unregister_chrdev_region(MKDEV(g_major, 0), 1);
	return ret;
}

/**
 * bms_exit - Module cleanup.
 *
 * Reverses bms_init() in the correct order to avoid use-after-free.
 */
static void __exit bms_exit(void)
{
	pr_info("bms_driver: unloading EV BMS kernel driver\n");

	/* Stop timer — del_timer_sync waits for any running callback */
	del_timer_sync(&g_sim_timer);
	pr_info("bms_driver: simulation timer stopped\n");

	/* Remove /proc entry */
	remove_proc_entry("bms_status", NULL);
	pr_info("bms_driver: /proc/bms_status removed\n");

	/* Destroy device node and class */
	device_destroy(g_class, MKDEV(g_major, 0));
	class_destroy(g_class);
	pr_info("bms_driver: device /dev/%s and class removed\n", BMS_DEVICE_NAME);

	/* Remove cdev and release major number */
	cdev_del(&g_cdev);
	unregister_chrdev_region(MKDEV(g_major, 0), 1);
	pr_info("bms_driver: chrdev major=%d released\n", g_major);

	pr_info("bms_driver: unload complete\n");
}

module_init(bms_init);
module_exit(bms_exit);
