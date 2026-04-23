
#include "DAP.h"
#include "DAP_config.h"
#include "class/vendor/vendor_device.h"
#include "common/tusb_types.h"

#include "device/dcd.h"
#include "device/usbd.h"
#include "projdefs.h"
#include <cmsis_gcc.h>
#include <stdio.h>
// Assumptions: abits=7, data_len=32
#define DMI_OP_NOP   0x0
#define DMI_OP_READ  0x1
#define DMI_OP_WRITE 0x2


#define DMI_IR_ADDRESS 0x11 

#define PIN_TCK_SET PIN_SWCLK_TCK_SET
#define PIN_TCK_CLR PIN_SWCLK_TCK_CLR
#define PIN_TMS_SET PIN_SWDIO_TMS_SET
#define PIN_TMS_CLR PIN_SWDIO_TMS_CLR

#define JTAG_CYCLE_TCK()                \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDI(tdi)             \
  PIN_TDI_OUT(tdi);                     \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDO(tdo)             \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  tdo = PIN_TDO_IN();                   \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDIO(tdi,tdo)        \
  PIN_TDI_OUT(tdi);                     \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  tdo = PIN_TDO_IN();                   \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define PIN_DELAY() __NOP();



void JTAG_DR_Write32(uint32_t data) {
    uint8_t tdi_buf[4];
    // RISC-V JTAG is LSB first
    tdi_buf[0] = (uint8_t)(data & 0xFF);
    tdi_buf[1] = (uint8_t)((data >> 8) & 0xFF);
    tdi_buf[2] = (uint8_t)((data >> 16) & 0xFF);
    tdi_buf[3] = (uint8_t)((data >> 24) & 0xFF);

    // 1. Enter Shift-DR
    PIN_TMS_SET();
    JTAG_CYCLE_TCK(); // Select-DR-Scan
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK(); // Capture-DR
    JTAG_CYCLE_TCK(); // Shift-DR

    // 2. Shift 32 bits
    // Note: If JTAG_Sequence handles the Exit1-DR transition on the last bit,
    // you don't need manual TMS toggling for the exit. 
    // Usually, JTAG_Sequence just shifts.
    JTAG_Sequence(32, tdi_buf, NULL);

    // 3. Exit-DR and Update-DR
    PIN_TMS_SET();
    JTAG_CYCLE_TCK(); // Exit1-DR
    JTAG_CYCLE_TCK(); // Update-DR
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK(); // Idle
}
// Shift a single DMI transfer (issue a request or read result).
// address: 7-bit DMI address
// data:    pointer to 32-bit data (for write pass input, for read receives output)
// op:      DMI op (00 NOP, 01 READ, 10 WRITE)
// Returns: 2-bit status (bits [1:0]) from the DMI (0=OK, 1=..., 2=FAIL, 3=BUSY)
static inline uint8_t JTAG_Transfer_RISCV(uint8_t address, uint32_t *data, uint8_t op)
{
    uint32_t bit;
    uint32_t n;
    uint32_t write_val = (data) ? *data : 0U;
    uint32_t read_val = 0U;
    uint8_t status = 0U;
    const uint8_t abits = 7U;

    /* 1) Go to Shift-DR */
    PIN_TMS_SET(); JTAG_CYCLE_TCK();   /* Select-DR-Scan */
    PIN_TMS_CLR(); JTAG_CYCLE_TCK();   /* Capture-DR */
    JTAG_CYCLE_TCK();                  /* Shift-DR */

    /* 2) Bypass bits before this device */
    for (n = DAP_Data.jtag_dev.index; n; n--) {
        JTAG_CYCLE_TCK();
    }

    /* 3) Shift OP (2 bits), LSB-first. Capture status bits from TDO */
    JTAG_CYCLE_TDIO(op >> 0, bit); status |= (uint8_t)(bit << 0);
    JTAG_CYCLE_TDIO(op >> 1, bit); status |= (uint8_t)(bit << 1);

    /* 4) Shift DATA (32 bits), LSB-first. Use TDIO so TDO gives the previous-data result */
    {
        uint32_t w = write_val;
        read_val = 0U;
        for (n = 0U; n < 32U; n++) {
            JTAG_CYCLE_TDIO(w, bit);          /* shift LSB of w, capture TDO bit */
            read_val |= ((uint32_t)bit << n); /* assemble LSB-first */
            w >>= 1;
        }
    }

    /* 5) Shift ADDRESS (abits), LSB-first.
       Must handle the last address bit specially: after it we either have bypass devices to shift through
       or we will transition to Exit1-DR with the last bit clock. */
    {
        uint32_t after = (DAP_Data.jtag_dev.count > DAP_Data.jtag_dev.index + 1U)
                          ? (DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U) : 0U;
        uint32_t a = address;

        for (n = abits; n; n--) {
            if (n == 1U) {
                /* last address bit => set TMS so the next TCK moves to Exit1-DR */
                if (after) {
                    /* shift last address bit (goes to Exit1-DR on that TCK), then clock bypass bits of later devices */
                    JTAG_CYCLE_TDIO(a, bit);   /* last addr bit, captured into 'bit' */
                    /* now there are 'after' bypass devices: clock them in bypass (TMS stays 0 during these clocks) */
                    for (; after > 0U; --after) {
                        JTAG_CYCLE_TCK();
                    }
                    PIN_TMS_SET();
                    JTAG_CYCLE_TCK(); /* move from Bypass... to Exit1-DR */
                } else {
                    /* no devices after: set TMS and shift last address bit (this TCK becomes Exit1-DR) */
                    PIN_TMS_SET();
                    JTAG_CYCLE_TDIO(a, bit); /* last addr bit and Exit1-DR */
                }
            } else {
                JTAG_CYCLE_TDIO(a, bit); /* normal address bit */
            }
            a >>= 1;
        }
    }

    /* 6) Update-DR (move from Exit1-DR -> Update-DR with an extra TCK), then Idle */
    JTAG_CYCLE_TCK();            /* Update-DR */
    PIN_TMS_CLR();
    JTAG_CYCLE_TCK();            /* Idle */
    PIN_TDI_OUT(1U);             /* set TDI idling */

    /* 7) optional idle clock required by some DTM implementations (dtmcs.idle) */
    for (int i = 0; i < 8; i++) { // 8 to be safe (minimum 7)
        JTAG_CYCLE_TCK();
    }

    if (data) {
        *data = read_val;
    }

    return status & 0x3U;
}
volatile uint64_t dmi_total_cycles = 0;
volatile uint64_t dmi_busy_cycles  = 0;
volatile uint32_t dmi_busy_loops   = 0;
volatile uint32_t dmi_ops          = 0;


// Simple poll-read helper. Returns the 32-bit data read from 'addr'.
// Blocks until device responds or until max_retries exceeded.
static uint32_t riscv_dmi_read_poll(uint8_t addr, unsigned max_retries)
{
    uint32_t data = 0U;
    uint8_t status;

    /* 1) Issue read request (this starts the read on the target) */
    JTAG_Transfer_RISCV(addr, NULL, DMI_OP_READ);

    /* 2) Poll: send NOP transfers until status != BUSY (3) */
    do {
        status = JTAG_Transfer_RISCV(0 /* addr ignored for NOP */, &data, DMI_OP_NOP);

        if (status == 3)
            dmi_busy_loops++;
        if (max_retries && --max_retries == 0) break;
    } while (status == 3);

    return data;
}


 static uint8_t riscv_dmi_write_poll(uint8_t addr, uint32_t data, unsigned max_retries)
{
    uint8_t status;
    uint32_t dummy;

    status = JTAG_Transfer_RISCV(addr, &data, DMI_OP_WRITE);

    /* 2) If Busy, we must poll with NOPs until it's not busy, 
          THEN re-issue the write because the busy one was ignored. */
    if (status == 3) {
        do {
            status = JTAG_Transfer_RISCV(0x00, &dummy, DMI_OP_NOP);

            if (max_retries && --max_retries == 0) return 3; // Timeout
        } while (status == 3);

        /* Now that it's finally NOT busy, re-issue the original write */
        status = JTAG_Transfer_RISCV(addr, &data, DMI_OP_WRITE);
    }

    /* 3) Optional: Final check to ensure the write was accepted 
          and didn't trigger an immediate error. */
    status = JTAG_Transfer_RISCV(0x00, &dummy, DMI_OP_NOP);
    
    return status; // Should be 0 (Success)
}


void ddmi_init()
{
    DAP_Setup();
    PORT_JTAG_SETUP();

    // 1. Clear the structure to zero out ir_before and ir_after
    memset(&DAP_Data.jtag_dev, 0, sizeof(DAP_Data.jtag_dev));

    // 2. Configure for a single RISC-V device
    DAP_Data.jtag_dev.count = 1;   // One device on chain
    DAP_Data.jtag_dev.index = 0;   // We are talking to the first one
    DAP_Data.jtag_dev.ir_length[0] = 5; // Standard RISC-V DTM IR length is 5 bits

    // 1. Clear DTM errors
    JTAG_IR(0x10);
    JTAG_DR_Write32(0x00010071); // dmireset
        //printf("read value:%08X\n", probe_dmi_read(0x11, 7));
    JTAG_IR(0x01);
    printf("IDCODE:%08X\n", JTAG_ReadIDCode());

            // Step 1: Clear sticky errors in DTMCS
    JTAG_IR(0x10);             // Select DTMCS
    //printf("DTMCS %08X\n", JTAG_ReadIDCode());
    JTAG_IR(0x11);

    vTaskDelay(pdMS_TO_TICKS(100));
}

#define MAX_BATCH 6
#define MAX_PKT 10
#define CMD_SIZE      9
#define RESP_SIZE     4

// Protocol constants
#define DMI_REQ_LEN  9
#define DMI_RESP_LEN 4
#define DDMI_ITF 1
void ddmi_process_with_chain();

void ddmi_worker_task(void *pvParameters) 
{
    while (1) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ddmi_process_with_chain();
    }
}

static uint32_t response_pool[100];
static uint8_t  total_responses_queued = 0;
#define USB_FS_MPS 64
void ddmi_process_with_chain() {
    uint8_t pkt[64];
    uint32_t len = tud_vendor_n_read(DDMI_ITF, pkt, 64);
    if (len < 2) return;

    uint8_t countdown = pkt[0];
    uint8_t op_count  = pkt[1];
    uint8_t *payload  = pkt + 2;

    if(op_count == 255) //management frame
    {
        if(payload[0] == 'R')
        {
            ddmi_init();
            printf("RESET!\n");
            return;
        }
    }

    // --- PHASE 1: EXECUTE ON THE FLY ---
    for (uint8_t i = 0; i < op_count; i++) {
        uint8_t *cmd = payload + (i * 9);
        
        // Parse: Type, Addr, Data (Manual shifts for speed/alignment)
        uint32_t addr = cmd[1] | (cmd[2] << 8) | (cmd[3] << 16) | (cmd[4] << 24);
        uint32_t data = cmd[5] | (cmd[6] << 8) | (cmd[7] << 16) | (cmd[8] << 24);

        if (cmd[0] == 'r') {
            response_pool[total_responses_queued++] = riscv_dmi_read_poll(addr, 1000);
        } else {
            riscv_dmi_write_poll(addr, data, 1000);
            response_pool[total_responses_queued++] = 0x00000000;
        }
    }

    // --- PHASE 2: BURST RESPONSES ---
    // Only when the chain is complete do we saturate the USB IN pipe
    if (countdown == 0) {
        uint8_t *resp_ptr = (uint8_t *)response_pool;
        uint32_t bytes_remaining = total_responses_queued * RESP_SIZE;

        while (bytes_remaining > 0) {
            // Check how much space is in the TinyUSB TX FIFO
            uint32_t fifo_space = tud_vendor_n_write_available(DDMI_ITF);
            
            if (fifo_space >= USB_FS_MPS || fifo_space >= bytes_remaining) {
                uint32_t chunk = (bytes_remaining >= USB_FS_MPS) ? USB_FS_MPS : bytes_remaining;
                
                tud_vendor_n_write(DDMI_ITF,resp_ptr, chunk);
                
                // If we just sent a full 64-byte block, flush it immediately 
                // to ensure the Host sees a "Full Packet"
                if (chunk == USB_FS_MPS) {
                    tud_vendor_n_write_flush(DDMI_ITF);
                }
                
                resp_ptr += chunk;
                bytes_remaining -= chunk;
            } else {
                // FIFO full, give the USB stack a moment to breathe
                tud_vendor_n_write_flush(DDMI_ITF);
                break; 
            }
        }
        
        // Final flush for the "tail" fragment (if total length % 64 != 0)
        tud_vendor_n_write_flush(DDMI_ITF);
        total_responses_queued = 0;
    }
}
