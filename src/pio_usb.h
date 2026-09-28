
#pragma once

#include "pio_usb_configuration.h"
#include "usb_definitions.h"

#ifdef __cplusplus
 extern "C" {
#endif

// Host functions
usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c);
int pio_usb_host_add_port(uint8_t pin_dp, PIO_USB_PINOUT pinout);
void pio_usb_host_task(void);
void pio_usb_host_stop(void);
void pio_usb_host_restart(void);
uint32_t pio_usb_host_get_frame_number(void);

// Call this every 1ms when skip_alarm_pool is true.
void pio_usb_host_frame(void);

// [LOCAL PATCH] Isochronous streaming inside the SOF interrupt (USB-Audio-Toolkit's passthrough
// audio relay). The two hooks are weak no-ops; an application defines them to move one packet
// per frame without waiting for the host stack's task loop.
//   sof:   just before this frame's SOF goes out (to time the SOF finer than the 1 us timer)
//   begin: after this frame's SOF went out, before any transaction of the frame
//   end:   after all transactions of the frame, before the host stack is told about completions
void pio_usb_host_frame_sof_cb(uint32_t frame);
void pio_usb_host_frame_begin_cb(uint32_t frame);
void pio_usb_host_frame_end_cb(uint32_t frame);
// [LOCAL PATCH] At the start of every endpoint 0 transaction (token = USB_PID_SETUP / IN / OUT),
// before the token goes out. Weak no-op; an application may start a capture of the lines here
void pio_usb_host_ep0_transaction_cb(uint8_t token, uint8_t dev_addr);
// Result of the endpoint's last transfer, taken exactly once: 1 = complete (*actual_len set),
// -1 = error, -2 = stalled, 0 = nothing finished since the last call. Taking it clears the
// completion bit so the host stack never sees the transfer (for endpoints the app opened itself).
int pio_usb_host_endpoint_take_result(uint8_t root_idx, uint8_t device_address,
                                      uint8_t ep_address, uint16_t *actual_len);
// true while a transfer is queued on the endpoint (false also when the endpoint is not open)
bool pio_usb_host_endpoint_busy(uint8_t root_idx, uint8_t device_address, uint8_t ep_address);
// Length of the next frame(s) in microseconds (default 1000). Used to phase-lock the SOF to
// another USB bus; takes effect from the next frame. Only with the built-in alarm pool timer.
void pio_usb_host_set_frame_period_us(uint32_t period_us);

// [LOCAL PATCH] Trace of the host's transactions on endpoint 0 (USB-Audio-Toolkit): what each
// device answered, to tell a DATA0/DATA1 mix-up from no answer or a broken packet. Written from
// the SOF interrupt into a ring; entry i is pio_usb_ep0_trace[i % PIO_USB_EP0_TRACE_LEN] for
// i < pio_usb_ep0_trace_count. A reader copies entries and checks the count did not move by a ring.
#define PIO_USB_EP0_TRACE_LEN 32
typedef struct {
  uint32_t time_us;    // get_time_us_32() after the transaction
  uint32_t frame;      // pio_usb_host_get_frame_number()
  uint8_t dev_addr;
  uint8_t token;       // USB_PID_SETUP / USB_PID_IN / USB_PID_OUT
  uint8_t expect_pid;  // IN: the DATA PID the host expected (the data toggle), else 0
  uint8_t got_pid;     // what came back: DATA0/1 for IN data, ACK/NAK/STALL, 0 = nothing
  int16_t len;         // IN: data bytes when the packet was sound (CRC ok), else -1
  int8_t res;          // 0 sound answer, -1 broken or unexpected answer, -2 no answer at all
  uint8_t failed;      // the endpoint's consecutive-failure count after this transaction
  uint8_t rx_irq;      // receiver flags afterwards: bit 3 a packet started, bit 2 EOP seen, bit 1 bit-stuff error
  uint8_t rx[2];       // the first two bytes received (SYNC 0x80 and the PID), as they were
  uint8_t irq0;        // the receive PIO IRQ flags when the transaction began (bit 2 EOP wait, bit 4 decoder trigger)
  uint8_t eop_pc;      // the edge detector's pc (from its program start) when the transaction began:
                       // 0 = parked at the EOP wait (normal), else it was free to decode the host's own packets
  uint8_t setup[8];    // SETUP: the request itself
} pio_usb_ep0_trace_t;
extern pio_usb_ep0_trace_t pio_usb_ep0_trace[PIO_USB_EP0_TRACE_LEN];
extern volatile uint32_t pio_usb_ep0_trace_count;
// Recording stops once the count reaches this (default: never). Set it to count + LEN to keep
// exactly the next LEN transactions, e.g. one whole enumeration
extern volatile uint32_t pio_usb_ep0_trace_stop_at;

// Device functions
usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers);
void pio_usb_device_task(void);

// Common functions
endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx);
int pio_usb_get_in_data(endpoint_t *ep, uint8_t *buffer, uint8_t len);
int pio_usb_set_out_data(endpoint_t *ep, const uint8_t *buffer, uint8_t len);

#ifdef __cplusplus
 }
#endif
