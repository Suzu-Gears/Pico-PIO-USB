
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
