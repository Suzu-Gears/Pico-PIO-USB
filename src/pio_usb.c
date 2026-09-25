/**
 * Copyright (c) 2021 sekigon-gonnoc
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdio.h>
#include <stdint.h>
#include <string.h> // memcpy

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/platform.h"

#include "pio_usb.h"
#include "usb_definitions.h"
#include "pio_usb_configuration.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"

#define UNUSED_PARAMETER(x) (void)x

usb_device_t pio_usb_device[PIO_USB_DEVICE_CNT];
pio_port_t pio_port[1];
root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
endpoint_t pio_usb_ep_pool[PIO_USB_EP_POOL_CNT];

static uint8_t ack_encoded[5];
static uint8_t nak_encoded[5];
static uint8_t stall_encoded[5];
static uint8_t pre_encoded[5];

// [LOCAL PATCH] 送信完了待ちの時間切れ（USB-Audio-Toolkit, 2026-09-24）。
// 送信ステートマシンがパケットの途中（EOP の後、完了記号の前）でデータ待ちのまま止まることがあり
// （原因は未特定）、下の待ちループは SOF の割り込みの中で永久に回って loop() を止め、watchdog 再起動に
// なっていた（治具のソフト watchdog で pio_usb_bus_usb_transfer の *pc < COMP 待ちと特定）。
// 最長のパケット（ISO 1023 B）でも 1 ms 足らずなので 3 ms で打ち切り、送信 DMA とステートマシンを
// 立て直して抜ける。そのパケットは応答待ちが時間切れになり、上位（TinyUSB）が再送する。
// 回数と、打ち切った瞬間の状態は下の変数で読める（アプリ側で表示する）
#define PIO_USB_TX_TIMEOUT_US 3000
#define PIO_USB_RX_PACKET_TIMEOUT_US 2000
volatile uint32_t pio_usb_tx_timeout_count = 0;
volatile uint32_t pio_usb_rx_timeout_count = 0;  // 受信側: EOP の来ない受信を打ち切った回数
// [0] 打ち切った待ち（1 = EOP 待ち, 2 = 完了記号待ち, 3/4 = PRE 送信）
// [1] SM の PC  [2] DMA の残り転送数  [3] DMA の CTRL（bit26 = BUSY）  [4] TX FIFO の段数
volatile uint32_t pio_usb_tx_timeout_info[5] = {0};

static void __no_inline_not_in_flash_func(tx_timeout_recover)(pio_port_t *pp, uint32_t where) {
  pio_usb_tx_timeout_info[0] = where;
  pio_usb_tx_timeout_info[1] = pp->pio_usb_tx->sm[pp->sm_tx].addr;
  pio_usb_tx_timeout_info[2] = dma_hw->ch[pp->tx_ch].transfer_count;
  pio_usb_tx_timeout_info[3] = dma_hw->ch[pp->tx_ch].ctrl_trig;
  pio_usb_tx_timeout_info[4] = pio_sm_get_tx_fifo_level(pp->pio_usb_tx, pp->sm_tx);
  pio_usb_tx_timeout_count++;
  dma_channel_abort(pp->tx_ch);
  pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  // 完了記号の位置（set pindirs, 0 = 線を離す）へ飛ばす。その次の out で止まり、通常の待機状態になる
  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_reset_instr);
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK;
}

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(send_pre)(pio_port_t *pp) {
  // send PRE token in full-speed
  pp->low_speed = false;
  uint16_t instr = pp->fs_tx_pre_program->instructions[0];
  pp->pio_usb_tx->instr_mem[pp->offset_tx] = instr;

  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK;       // clear complete flag
  dma_channel_transfer_from_buffer_now(pp->tx_ch, pre_encoded,
                                       sizeof(pre_encoded));

  uint32_t const t0 = get_time_us_32();  // [LOCAL PATCH] 時間切れ（ファイル先頭の説明）
  while ((pp->pio_usb_tx->irq & IRQ_TX_EOP_MASK) == 0) {
    if (get_time_us_32() - t0 > PIO_USB_TX_TIMEOUT_US) {
      tx_timeout_recover(pp, 3);
      break;
    }
  }
  // Wait for complete transmission of the PRE packet. We don't want to
  // accidentally send trailing Ks in low speed mode due to an early start
  // instruction that re-enables the outputs.
  uint32_t stall_mask = 1 << (PIO_FDEBUG_TXSTALL_LSB + pp->sm_tx);
  pp->pio_usb_tx->fdebug = stall_mask; // clear sticky stall mask bit
  while (!(pp->pio_usb_tx->fdebug & stall_mask)) {
    if (get_time_us_32() - t0 > PIO_USB_TX_TIMEOUT_US) {
      tx_timeout_recover(pp, 4);
      break;
    }
  }

  // change bus speed to low-speed
  pp->low_speed = true;
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  instr = pp->fs_tx_program->instructions[0];
  pp->pio_usb_tx->instr_mem[pp->offset_tx] = instr;
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);

  // pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  // pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  // SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

void __not_in_flash_func(pio_usb_bus_usb_transfer)(pio_port_t *pp,
                                              uint8_t *data, uint16_t len) {
  if (pp->need_pre) {
    send_pre(pp);
  }

  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_start_instr);
  dma_channel_transfer_from_buffer_now(pp->tx_ch, data, len);
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK; // clear complete flag

  io_ro_32 *pc = &pp->pio_usb_tx->sm[pp->sm_tx].addr;
  uint32_t const t0 = get_time_us_32();  // [LOCAL PATCH] 時間切れ（ファイル先頭の説明）
  while ((pp->pio_usb_tx->irq & IRQ_TX_ALL_MASK) == 0) {
    if (get_time_us_32() - t0 > PIO_USB_TX_TIMEOUT_US) {
      tx_timeout_recover(pp, 1);
      return;
    }
  }
  pp->pio_usb_tx->irq = IRQ_TX_ALL_MASK; // clear complete flag

  // [LOCAL PATCH] 完了記号の待ちは「pc を見る」だけだと取りこぼす（USB-Audio-Toolkit, 2026-09-25）。
  // usb_tx.pio では SE0（EOP）の後、SM は完了記号（アドレス 2）→ out（3）→ 詰め物の K（1）と進み、
  // TX FIFO が空になると pc=1 の out で止まる。これが正常な終わり方で、pc が 2〜3 にいるのは 8 サイクル
  // （フルスピードで約 170 ns）しかない。下のループの 1 周がそれより長くなる（割り込み・バスの取り合い・
  // 時間切れのための時刻読み）とその窓を見逃し、正常に送り終えた SM を pc=1 のまま待ち続けていた
  // （2026-09-24 に入れた 3 ms の時間切れの正体。打ち切り時の記録 wait=2 sm_pc=1 dma_left=0 fifo=0 は正常終了の状態）。
  // そこで、EOP の後に TX FIFO が尽きて止まった（FDEBUG の TXSTALL）ことでも完了とみなす。詰め物は完了記号の後に
  // しか無いので、EOP の後の停止は完了記号を通り過ぎた印になる（send_pre の PRE の待ちと同じ方法）
  uint32_t const stall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + pp->sm_tx);
  pp->pio_usb_tx->fdebug = stall_mask;  // EOP より前（送り始め）の停止は数えない
  if (pp->low_speed) {
    // For Low speed host, wait until EOP is fully sent. Otherwise, we can send another packet
    // before inter-packet delay timeout, which is 2-bit time by USB specs.
    // For Full speed, our overhead is probably enough without this additional wait.
    while (*pc <= PIO_USB_TX_ENCODED_DATA_COMP && !(pp->pio_usb_tx->fdebug & stall_mask)) {
      if (get_time_us_32() - t0 > PIO_USB_TX_TIMEOUT_US) {
        tx_timeout_recover(pp, 2);
        return;
      }
    }
  } else {
    while (*pc < PIO_USB_TX_ENCODED_DATA_COMP && !(pp->pio_usb_tx->fdebug & stall_mask)) {
      if (get_time_us_32() - t0 > PIO_USB_TX_TIMEOUT_US) {
        tx_timeout_recover(pp, 2);
        return;
      }
    }
  }
}

void __no_inline_not_in_flash_func(pio_usb_bus_send_token)(pio_port_t *pp,
                                                           uint8_t token,
                                                           uint8_t addr,
                                                           uint8_t ep_num) {

  uint8_t packet[4] = {USB_SYNC, token, 0, 0};
  uint16_t dat = ((uint16_t)(ep_num & 0xf) << 7) | (addr & 0x7f);
  uint8_t crc = calc_usb_crc5(dat);
  packet[2] = dat & 0xff;
  packet[3] = (crc << 3) | ((dat >> 8) & 0x1f);

  uint8_t packet_encoded[sizeof(packet) * 2 * 7 / 6 + 2];
  uint8_t encoded_len = pio_usb_ll_encode_tx_data(packet, sizeof(packet), packet_encoded);

  pio_usb_bus_usb_transfer(pp, packet_encoded, encoded_len);
}

void __no_inline_not_in_flash_func(pio_usb_bus_prepare_receive)(const pio_port_t *pp) {
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr);
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr2);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);
}

static inline __force_inline bool pio_usb_bus_wait_for_rx_start(const pio_port_t* pp) {
  // USB 2.0 specs: 7.1.19.1: handshake timeout
  // Full-Speed (12 Mbps): 1 bit time = 1 / 12 MHz = 83.3 ns --> 16 bit times = 1.33 µs
  // Low-Speed (1.5 Mbps): 1 bit time = 1 / 1.5 MHz = 666.7 ns --> 16 bit times = 10.67 µs

  // We're starting the timing somewhere in the current microsecond so always assume the first one
  // is less than a full microsecond. For example, a wait of 2 could actually be 1.1 microseconds.
  // We will use 3 us (24 bit time) for Full speed and 12us (18 bit time) for Low speed.
  uint32_t start = get_time_us_32();
  uint32_t timeout = pp->low_speed ? 12 : 3;
  while (get_time_us_32() - start <= timeout) {
    if ((pp->pio_usb_rx->irq & IRQ_RX_START_MASK) != 0) {
      return true;
    }
  }
  return false;
};

uint8_t __no_inline_not_in_flash_func(pio_usb_bus_wait_handshake)(pio_port_t* pp) {
  if (!pio_usb_bus_wait_for_rx_start(pp)) {
    return 0;
  }

  int16_t idx = 0;
  // Timeout in seven microseconds. That is enough time to receive one byte at low speed.
  // This is to detect packets without an EOP because the device was unplugged.
  uint32_t start = get_time_us_32();
  while (get_time_us_32() - start <= 7) {
    if (idx < 2 && pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
      uint8_t data = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
      pp->usb_rx_buffer[idx++] = data;

      start = get_time_us_32(); // reset timeout when a byte is received
    } else if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) != 0) {
      break; // exit if we've gotten an EOP
    }
  }

  if (idx != 2 || pp->usb_rx_buffer[0] != USB_SYNC) {
    return 0; // invalid handshake
  }

  return pp->usb_rx_buffer[1];
}

int __no_inline_not_in_flash_func(pio_usb_bus_receive_packet_and_handshake)(
    pio_port_t *pp, uint8_t handshake) {
  uint16_t crc = 0xffff;
  uint16_t crc_prev = 0xffff;
  uint16_t crc_prev2 = 0xffff;
  uint16_t crc_receive = 0xffff;
  bool crc_match = false;
  const uint16_t rx_buf_len = sizeof(pp->usb_rx_buffer) / sizeof(pp->usb_rx_buffer[0]);
  int16_t idx = 0;

  // Per USB Specs 7.1.18 for turnaround: We must wait at least 2 bit times for inter-packet delay.
  // This is essential for working with LS device specially when we overlocked the mcu.
  // Pre-calculate number of cycle per bit time
  // - Lowspeed: 1 bit time = (cpufreq / 1.5 Mhz) = clk_div_ls_tx.div_int*6Mhz / 1.5 Mhz = 4 * clk_div_ls_tx.div_int
  // - Fullspeed 1 bit time = (cpufreq / 12 Mhz) = clk_div_fs_tx.div_int*48Mhz / 12 Mhz = 4 * clk_div_fs_tx.div_int
  // Since there is also overhead, we only wait 1.5 bit for LS and no wait for FS
  uint32_t turnaround_in_cycle = 0;
  if (pp->low_speed) {
    turnaround_in_cycle = 6 * pp->clk_div_ls_tx.div_int; // 1.5 bit time
  }

  if (!pio_usb_bus_wait_for_rx_start(pp)) {
    return -1;
  }

  // Timing Critical: use local variable to reduce de-reference
  PIO pio_usb_rx = pp->pio_usb_rx;
  uint sm_rx =  pp->sm_rx;
  uint8_t *usb_rx_buffer = pp->usb_rx_buffer;

  // Timeout in seven microseconds. That is enough time to receive one byte at low speed.
  // This is to detect packets without an EOP because the device was unplugged.
  uint32_t start = get_time_us_32();
  uint32_t const packet_start = start;  // [LOCAL PATCH] パケット全体の上限（下の説明）
  while (1) {
    if (pio_sm_get_rx_fifo_level(pio_usb_rx, sm_rx)) {
      uint8_t data = pio_sm_get(pio_usb_rx, sm_rx) >> 24;
      if (idx >= 0 && idx < rx_buf_len) {  // [LOCAL PATCH] idx が桁あふれで負になっても手前を書かない
        usb_rx_buffer[idx] = data;
      }
      start = get_time_us_32(); // reset timeout when a byte is received
      // [LOCAL PATCH] 上の 7 us の時間切れはバイトが届くたびに延長されるので、EOP の来ないままバイトが
      // 届き続けると（線の雑音、電源投入直後の DUT）永久に回る。治具ではこれが SOF 割り込みの中で起き、
      // loop() が止まって watchdog 再起動になった（2026-09-24、ソフト watchdog で特定）。
      // FS の最長パケット（ISO 1023 B）でも 1 ms 足らずなので、パケット全体に 2 ms の上限を設ける
      if (start - packet_start > PIO_USB_RX_PACKET_TIMEOUT_US) {
        pio_usb_rx_timeout_count++;
        return -1;
      }

      if (idx >= 2) {
        crc_prev2 = crc_prev;
        crc_prev = crc;
        crc = update_usb_crc16(crc, data);
        crc_receive = (crc_receive >> 8) | (data << 8);
        crc_match = ((crc_receive ^ 0xffff) == crc_prev2);
      }
      idx++;
    } else if ((pio_usb_rx->irq & IRQ_RX_COMP_MASK) != 0) {
      // Exit since we've gotten an EOP.
      // Timing critical: per USB specs, handshake must be sent within 2-7 bit-time strictly
      if (turnaround_in_cycle) {
        busy_wait_at_least_cycles(turnaround_in_cycle); // wait for turnaround for LS only
      }

      if (handshake == USB_PID_ACK || handshake == 0) {
        // Only accept if crc matches.
        // [LOCAL PATCH] handshake 0 = isochronous: accept but send no handshake
        if (idx >= 4 && crc_match) {
          if (handshake == USB_PID_ACK) {
            pio_usb_bus_usb_transfer(pp, ack_encoded, 5);
          }
          return idx - 4;
        }
      } else if (handshake == USB_PID_NAK) {
        pio_usb_bus_usb_transfer(pp, nak_encoded, 5);
      } else {
        pio_usb_bus_usb_transfer(pp, stall_encoded, 5);
      }
      break;
    } else if (get_time_us_32() - start > 7) {
      return -1; // device is probably unplugged
    }
  }

  return -1;
}

static __always_inline void add_pio_host_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint *offset, int debug_pin) {
  if (debug_pin < 0) {
    *offset = pio_add_program(pio, program);
  } else {
    *offset = pio_add_program(pio, debug_program);
  }
}

static void __no_inline_not_in_flash_func(initialize_host_programs)(
    pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t *port) {
  // TX program should be placed at address 0
  pio_add_program_at_offset(pp->pio_usb_tx, pp->fs_tx_program, 0);
  pp->offset_tx = 0;
  usb_tx_fs_program_init(pp->pio_usb_tx, pp->sm_tx, pp->offset_tx, port->pin_dp,
                         port->pin_dm);
  uint32_t sideset_fj_lk;
  if (c->pinout == PIO_USB_PINOUT_DPDM) {
    sideset_fj_lk = pio_encode_sideset(2, usb_tx_dpdm_FJ_LK);
  } else {
    sideset_fj_lk = pio_encode_sideset(2, usb_tx_dmdp_FJ_LK);
  }

  pp->tx_start_instr = pio_encode_jmp(pp->offset_tx + 4) | sideset_fj_lk;
  pp->tx_reset_instr = pio_encode_jmp(pp->offset_tx + 2) | sideset_fj_lk;

  add_pio_host_rx_program(pp->pio_usb_rx, &usb_nrzi_decoder_program,
                          &usb_nrzi_decoder_debug_program, &pp->offset_rx,
                          c->debug_pin_rx);
  usb_rx_fs_program_init(pp->pio_usb_rx, pp->sm_rx, pp->offset_rx, port->pin_dp,
                         port->pin_dm, c->debug_pin_rx);
  pp->rx_reset_instr = pio_encode_jmp(pp->offset_rx);
  pp->rx_reset_instr2 = pio_encode_set(pio_x, 0);

  add_pio_host_rx_program(pp->pio_usb_rx, &usb_edge_detector_program,
                          &usb_edge_detector_debug_program, &pp->offset_eop,
                          c->debug_pin_eop);
  eop_detect_fs_program_init(pp->pio_usb_rx, c->sm_eop, pp->offset_eop,
                             port->pin_dp, port->pin_dm, true,
                             c->debug_pin_eop);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
}

static void configure_tx_channel(uint8_t ch, PIO pio, uint sm) {
  dma_channel_config conf = dma_channel_get_default_config(ch);

  channel_config_set_read_increment(&conf, true);
  channel_config_set_write_increment(&conf, false);
  channel_config_set_transfer_data_size(&conf, DMA_SIZE_8);
  channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));

  dma_channel_set_config(ch, &conf, false);
  dma_channel_set_write_addr(ch, &pio->txf[sm], false);
}

static void apply_config(pio_port_t *pp, const pio_usb_configuration_t *c,
                         root_port_t *port) {
  pp->pio_usb_tx = pio_get_instance(c->pio_tx_num);
  pp->sm_tx = c->sm_tx;
  pp->tx_ch = c->tx_ch;
  pp->pio_usb_rx = pio_get_instance(c->pio_rx_num);
  pp->sm_rx = c->sm_rx;
  pp->sm_eop = c->sm_eop;
  port->pin_dp = c->pin_dp;

  uint highest_pin;
  if (c->pinout == PIO_USB_PINOUT_DPDM) {
    port->pin_dm = c->pin_dp + 1;
    highest_pin = port->pin_dm;
    pp->fs_tx_program = &usb_tx_dpdm_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dpdm_program;
    pp->ls_tx_program = &usb_tx_dmdp_program;
  } else {
    port->pin_dm = c->pin_dp - 1;
    highest_pin = port->pin_dp;
    pp->fs_tx_program = &usb_tx_dmdp_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dmdp_program;
    pp->ls_tx_program = &usb_tx_dpdm_program;
  }

#if defined(PICO_PIO_USE_GPIO_BASE) && PICO_PIO_USE_GPIO_BASE+0
  if (highest_pin > 32) {
    pio_set_gpio_base(pp->pio_usb_tx, 16);
    pio_set_gpio_base(pp->pio_usb_rx, 16);
  }
#else
  (void)highest_pin;
#endif

  port->pinout = c->pinout;

  pp->debug_pin_rx = c->debug_pin_rx;
  pp->debug_pin_eop = c->debug_pin_eop;

  pio_sm_claim(pp->pio_usb_tx, pp->sm_tx);
  pio_sm_claim(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_claim(pp->pio_usb_rx, pp->sm_eop);
}

static void port_pin_drive_setting(const root_port_t *port) {
  gpio_set_slew_rate(port->pin_dp, GPIO_SLEW_RATE_FAST);
  gpio_set_slew_rate(port->pin_dm, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(port->pin_dp, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(port->pin_dm, GPIO_DRIVE_STRENGTH_12MA);
}

void pio_usb_bus_init(pio_port_t *pp, const pio_usb_configuration_t *c,
                      root_port_t *root) {
  memset(root, 0, sizeof(root_port_t));

  pp->pio_usb_tx = pio_get_instance(c->pio_tx_num);
  dma_claim_mask(1<<c->tx_ch);
  configure_tx_channel(c->tx_ch, pp->pio_usb_tx, c->sm_tx);

  apply_config(pp, c, root);
  initialize_host_programs(pp, c, root);
  port_pin_drive_setting(root);
  root->initialized = true;
  root->dev_addr = 0;

  // pre-encode handshake packets
  uint8_t raw_packet[] = {USB_SYNC, USB_PID_ACK};
  pio_usb_ll_encode_tx_data(raw_packet, 2, ack_encoded);
  raw_packet[1] = USB_PID_NAK;
  pio_usb_ll_encode_tx_data(raw_packet, 2, nak_encoded);
  raw_packet[1] = USB_PID_STALL;
  pio_usb_ll_encode_tx_data(raw_packet, 2, stall_encoded);
  raw_packet[1] = USB_PID_PRE;
  pio_usb_ll_encode_tx_data(raw_packet, 2, pre_encoded);
}

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx) {
  uint8_t ep_id = device->endpoint_id[idx];
  if (ep_id == 0) {
    return NULL;
  } else if (ep_id >= 1) {
    return &pio_usb_ep_pool[ep_id - 1];
  }
  return NULL;
}

int __no_inline_not_in_flash_func(pio_usb_get_in_data)(endpoint_t *ep,
                                                       uint8_t *buffer,
                                                       uint8_t len) {
  if (ep->has_transfer || ep->is_tx) {
    return -1;
  }

  if (ep->new_data_flag) {
    len = len < ep->actual_len ? len : ep->actual_len;
    memcpy(buffer, (void *)ep->buffer, len);

    ep->new_data_flag = false;

    return pio_usb_ll_transfer_start(ep, ep->buffer, ep->size) ? len : -1;
  }

  return -1;
}

int __no_inline_not_in_flash_func(pio_usb_set_out_data)(endpoint_t *ep,
                                                        const uint8_t *buffer,
                                                        uint8_t len) {
  if (ep->has_transfer || !ep->is_tx) {
    return -1;
  }

  return pio_usb_ll_transfer_start(ep, (uint8_t *)buffer, len) ? 0 : -1;
}

//--------------------------------------------------------------------+
// Low Level Function
//--------------------------------------------------------------------+

void __no_inline_not_in_flash_func(pio_usb_ll_configure_endpoint)(
    endpoint_t *ep, uint8_t const *desc_endpoint) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  ep->size = d->max_size[0] | (d->max_size[1] << 8);
  // [LOCAL PATCH] the TX staging buffer (prepare_tx_data) and ep->buffer hold PIO_USB_EP_SIZE bytes.
  // A larger wMaxPacketSize used to overflow them (a 196-byte isochronous packet with the default
  // 192); clamp so a big endpoint gets truncated packets instead of a corrupted stack
  if (ep->size > PIO_USB_EP_SIZE) {
    ep->size = PIO_USB_EP_SIZE;
  }
  ep->ep_num = d->epaddr;
  ep->attr = d->attr;
  ep->interval = d->interval;
  ep->interval_counter = 0;
  ep->data_id = 0;
}

// Encode transfer data to 2bit sequence represents TX PIO instruction address
// [LOCAL PATCH] widened to 16 bit so >64-byte isochronous packets encode correctly
uint16_t __no_inline_not_in_flash_func(pio_usb_ll_encode_tx_data)(
    uint8_t const *buffer, uint16_t buffer_len, uint8_t *encoded_data) {
  uint16_t bit_idx = 0;
  int current_state = 1;
  int bit_stuffing = 6;
  for (int idx = 0; idx < buffer_len; idx++) {
    uint8_t data_byte = buffer[idx];
    for (int b = 0; b < 8; b++) {
      uint16_t byte_idx = bit_idx >> 2;
      encoded_data[byte_idx] <<= 2;
      if (data_byte & (1 << b)) {
        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
        }
        bit_stuffing--;
      } else {
        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
          current_state = 0;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
          current_state = 1;
        }
        bit_stuffing = 6;
      }

      bit_idx++;

      if (bit_stuffing == 0) {
        byte_idx = bit_idx >> 2;
        encoded_data[byte_idx] <<= 2;

        if (current_state) {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_J;
          current_state = 0;
        } else {
          encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
          current_state = 1;
        }
        bit_stuffing = 6;
        bit_idx++;
      }
    }
  }

  uint16_t byte_idx = bit_idx >> 2;
  encoded_data[byte_idx] <<= 2;
  encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_SE0;
  bit_idx++;

  byte_idx = bit_idx >> 2;
  encoded_data[byte_idx] <<= 2;
  encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_COMP;
  bit_idx++;

  // terminate buffers with K
  do {
    byte_idx = bit_idx >> 2;
    encoded_data[byte_idx] <<= 2;
    encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
    bit_idx++;
  } while (bit_idx & 0x03);

  byte_idx = bit_idx >> 2;
  return byte_idx;
}

static inline __force_inline void prepare_tx_data(endpoint_t *ep) {
  uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);
  uint8_t buffer[PIO_USB_EP_SIZE + 4];
  buffer[0] = USB_SYNC;
  buffer[1] = (ep->data_id == 1) ? USB_PID_DATA1
                                 : USB_PID_DATA0; // USB_PID_SETUP also DATA0
  memcpy(buffer + 2, ep->app_buf, xact_len);

  uint16_t const crc16 = calc_usb_crc16(ep->app_buf, xact_len);
  buffer[2 + xact_len] = crc16 & 0xff;
  buffer[2 + xact_len + 1] = crc16 >> 8;

  ep->encoded_data_len =
      pio_usb_ll_encode_tx_data(buffer, xact_len + 4, ep->buffer);
}

bool __no_inline_not_in_flash_func(pio_usb_ll_transfer_start)(endpoint_t *ep,
                                                              uint8_t *buffer,
                                                              uint16_t buflen) {
  if (ep->has_transfer) {
    return false;
  }

  ep->app_buf = buffer;
  ep->total_len = buflen;
  ep->actual_len = 0;
  ep->failed_count = 0;

  // [LOCAL PATCH] isochronous endpoints always use DATA0 (no handshake ever
  // toggles it), so pin it here in case a previous transfer flipped it
  if ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS) {
    ep->data_id = 0;
  }

  if (ep->is_tx) {
    prepare_tx_data(ep);
  } else {
    ep->new_data_flag = false;
  }

  ep->transfer_started = false;
  ep->transfer_aborted = false;
  ep->has_transfer = true;

  return true;
}

bool __no_inline_not_in_flash_func(pio_usb_ll_transfer_continue)(
    endpoint_t *ep, uint16_t xferred_bytes) {
  ep->app_buf += xferred_bytes;
  ep->actual_len += xferred_bytes;
  ep->data_id ^= 1;

  if ((xferred_bytes < ep->size) || (ep->actual_len >= ep->total_len)) {
    // complete if all bytes transferred or short packet
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
    return false;
  } else {
    if (ep->is_tx) {
      prepare_tx_data(ep);
    }

    return true;
  }
}

void __no_inline_not_in_flash_func(pio_usb_ll_transfer_complete)(
    endpoint_t *ep, uint32_t flag) {
  root_port_t *rport = PIO_USB_ROOT_PORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep - pio_usb_ep_pool));

  rport->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    rport->ep_complete |= ep_mask;
    if (!ep->is_tx) {
      ep->new_data_flag = true;
    }
  } else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    rport->ep_error |= ep_mask;
  } else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    rport->ep_stalled |= ep_mask;
  } else {
    // something wrong
  }

  ep->has_transfer = false;
}

int pio_usb_host_add_port(uint8_t pin_dp, PIO_USB_PINOUT pinout) {
  for (int idx = 0; idx < PIO_USB_ROOT_PORT_CNT; idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(idx);
    if (!root->initialized) {
      root->pin_dp = pin_dp;

      if (pinout == PIO_USB_PINOUT_DPDM) {
        root->pin_dm = pin_dp + 1;
      } else {
        root->pin_dm = pin_dp - 1;
      }
      root->pinout = pinout;

      gpio_pull_down(pin_dp);
      gpio_pull_down(root->pin_dm);
      pio_gpio_init(pio_port[0].pio_usb_tx, pin_dp);
      pio_gpio_init(pio_port[0].pio_usb_tx, root->pin_dm);
      gpio_set_inover(pin_dp, GPIO_OVERRIDE_INVERT);
      gpio_set_inover(root->pin_dm, GPIO_OVERRIDE_INVERT);
      pio_sm_set_pindirs_with_mask64(pio_port[0].pio_usb_tx, pio_port[0].sm_tx, 0,
                                   (1ull << pin_dp) | (1ull << root->pin_dm));
      port_pin_drive_setting(root);
      root->initialized = true;

      return 0;
    }
  }

  return -1;
}

#pragma GCC pop_options
