#include "main.h"

#define CLAMP_0_255(value) ((value) < 0 ? 0 : ((value) > 255 ? 255 : (value)))

uint32_t _usb_interval = 7000;

uint _adapter_output_irq;
uint _adapter_input_irq;
uint _gamecube_offset;
pio_sm_config _gamecube_c[4];

const uint8_t _led_defs[4] = {0, 1, 2, 3};

volatile bool _gc_tx_done = false;
bool _gc_running = false;

#define ALIGNED_JOYBUS_8(val) ((val) << 24)

uint8_t _port_phases[4] = {0};
uint32_t _port_probes[4] = {0};
uint32_t _port_inputs[4][4] = {{0}};
joybus_input_s _port_joybus[4] = {0, 0, 0, 0};

#define ORIGIN_DELAY_CYCLES 8
uint8_t delay_cycles = 0;

bool _port_rumble[4] = {false, false, false, false};

// Raw joybus command passthrough (for WebUSB)
volatile bool _port_raw_cmd_pending[4]          = {false};
volatile uint8_t _port_raw_cmd_buf[4][128]       = {0};
volatile uint8_t _port_raw_cmd_len[4]            = {0};
volatile bool _port_raw_expect_response[4]       = {false};
volatile uint8_t _port_raw_expected_len[4]       = {0};
volatile uint8_t _port_raw_response[4][128]      = {0};
volatile uint8_t _port_raw_response_len[4]       = {0};
volatile bool _port_raw_response_ready[4]        = {false};
static bool _port_raw_drained[4]                 = {false};

void joybus_itf_queue_raw_cmd(uint8_t port, uint8_t *cmd, uint8_t cmd_len, uint8_t resp_len) {
    if (port >= 4) return;
    uint8_t n = cmd_len < 128 ? cmd_len : 128;
    uint8_t r = resp_len < 128 ? resp_len : 128;
    memcpy((void*)_port_raw_cmd_buf[port], cmd, n);
    _port_raw_cmd_len[port] = n;
    _port_raw_expected_len[port] = r;
    _port_raw_response_ready[port] = false;
    _port_raw_cmd_pending[port] = true;
}

bool joybus_itf_consume_raw_response(uint8_t port, uint8_t *buf, uint8_t *out_len) {
    if (port >= 4 || !_port_raw_response_ready[port]) return false;
    uint8_t len = _port_raw_response_len[port];
    memcpy(buf, (void*)_port_raw_response[port], len);
    *out_len = len;
    _port_raw_response_ready[port] = false;
    return true;
}

typedef struct
{
    int lx_offset;
    int ly_offset;
    int rx_offset;
    int ry_offset;
    int lt_offset;
    int rt_offset;
} analog_offset_s;

analog_offset_s _port_offsets[4] = {0};

uint read_count = 0;

void _gc_port_reset(uint port)
{
    _port_joybus[port].port_itf = -1;
    _port_phases[port] = 0;
}

void _gc_port_data(uint port)
{
    if (!_port_phases[port])
    {
        // For this specific circumstance, we must push
        // manually since our data is set to push auto 32 bits
        pio_sm_exec(JOYBUS_PIO, port, pio_encode_push(false, false));
        _port_probes[port] = pio_sm_get(JOYBUS_PIO, port) >> 17;

        if (_port_probes[port] & 0x09 )
        {
            // Successfully obtained GC info
            // Set our delay cycles so controllers have
            // a moment to adjust their voltage and settle
            // before getting calibration data
            delay_cycles = ORIGIN_DELAY_CYCLES;
            _port_phases[port] = 1;
        }

        _port_probes[port] = 0;
    }
    else if (_port_phases[port] == 1)
    {

        // Collect data for analog offset creation
        for (uint i = 0; i < 2; i++)
        {
            if (!pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port))
            {
                _port_inputs[port][i] = pio_sm_get(JOYBUS_PIO, port);
            }
            else
            {
                _gc_port_reset(port);
                return;
            }
        }

        _port_joybus[port].byte_1 = _port_inputs[port][0];
        _port_joybus[port].byte_2 = _port_inputs[port][1];

        _port_offsets[port].lx_offset = 128 - (int)_port_joybus[port].stick_left_x;
        _port_offsets[port].rx_offset = 128 - (int)_port_joybus[port].stick_right_x;
        _port_offsets[port].ly_offset = 128 - (int)_port_joybus[port].stick_left_y;
        _port_offsets[port].ry_offset = 128 - (int)_port_joybus[port].stick_right_y;

        _port_offsets[port].lt_offset = -(int)_port_joybus[port].analog_trigger_l;
        _port_offsets[port].rt_offset = -(int)_port_joybus[port].analog_trigger_r;

        // Set the port phase
        _port_phases[port] = 2;

        // Set the port USB Interface
        int tmp_itf = 0;

        for (uint8_t i = 0; i < 4; i++)
        {
            bool itfInUse = false;

            for (uint8_t j = 0; j < 4; j++)
            {
                if (_port_joybus[j].port_itf == i)
                {
                    itfInUse = true;
                    break;
                }
            }

            if (!itfInUse)
            {
                tmp_itf = i;
                break; // Exit the loop once an available port number is found
            }
        }

        _port_joybus[port].port_itf = tmp_itf;
    }
    else if (_port_phases[port] == 2)
    {
        // If a raw response was already drained in the tight-loop path,
        // skip normal FIFO reading for this port.
        if (_port_raw_drained[port]) {
            _port_raw_drained[port] = false;
            return;
        }

        static uint8_t port_reset_timer[4] = {0};

        for (uint i = 0; i < 2; i++)
        {
            if (!pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port))
            {
                _port_inputs[port][i] = pio_sm_get(JOYBUS_PIO, port);
            }
            else
            {   
                port_reset_timer[port] += 1;
                if(port_reset_timer[port]>=10)
                {
                    _gc_port_reset(port);
                    port_reset_timer[port] = 0;
                }
                
                return;
            }
        }
        _port_joybus[port].byte_1 = _port_inputs[port][0];
        _port_joybus[port].byte_2 = _port_inputs[port][1];

        int lx = CLAMP_0_255(_port_joybus[port].stick_left_x + _port_offsets[port].lx_offset);
        int ly = CLAMP_0_255(_port_joybus[port].stick_left_y + _port_offsets[port].ly_offset);
        int rx = CLAMP_0_255(_port_joybus[port].stick_right_x + _port_offsets[port].rx_offset);
        int ry = CLAMP_0_255(_port_joybus[port].stick_right_y + _port_offsets[port].ry_offset);

        int lt = CLAMP_0_255(_port_joybus[port].analog_trigger_l + _port_offsets[port].lt_offset);
        int rt = CLAMP_0_255(_port_joybus[port].analog_trigger_r + _port_offsets[port].rt_offset);

        // Apply offsets
        _port_joybus[port].stick_left_x = (uint8_t)lx;
        _port_joybus[port].stick_left_y = (uint8_t)ly;
        _port_joybus[port].stick_right_x = (uint8_t)rx;
        _port_joybus[port].stick_right_y = (uint8_t)ry;

        _port_joybus[port].analog_trigger_l = (uint8_t)lt;
        _port_joybus[port].analog_trigger_r = (uint8_t)rt;
    }
}

void _gamecube_get_data()
{
    _gc_port_data(0);
    _gc_port_data(1);
    _gc_port_data(2);
    _gc_port_data(3);
}

// Drain a raw joybus response from the PIO RX FIFO in a tight loop.
// PIO auto-pushes every 32 bits (4 bytes). We read words as they arrive
// so the 4-deep FIFO never overflows, even for responses up to 128 bytes.
// Trailing bytes (when response isn't a multiple of 4) are recovered via
// a forced ISR push.
static void _drain_raw_response(uint port) {
    uint8_t expected = _port_raw_expected_len[port];
    if (expected == 0) expected = 8;  // fallback

    // Number of full 32-bit words the PIO will auto-push.
    // response_bits = expected*8 + 1 (stop bit).
    // full_words = floor(response_bits / 32)
    uint16_t response_bits = (uint16_t)expected * 8 + 1;
    uint16_t full_words = response_bits / 32;
    uint16_t remaining_bits = response_bits % 32;

    uint8_t *buf = (uint8_t *)_port_raw_response[port];
    uint8_t byte_pos = 0;

    // First-word timeout is long (50ms) because the controller may need to
    // perform slow operations (e.g. flash erase/write) before responding.
    // Subsequent words use a short timeout since joybus data flows
    // continuously once started (~128µs per 32-bit word).
    uint32_t word_timeout_us = 50000;

    // Read full 32-bit words from FIFO
    for (uint16_t w = 0; w < full_words && byte_pos + 4 <= 128; w++) {
        uint32_t t = time_us_32();
        while (pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port)) {
            if (time_us_32() - t > word_timeout_us) goto cleanup;
        }
        word_timeout_us = 200;  // short timeout for subsequent words
        uint32_t word = pio_sm_get(JOYBUS_PIO, port);
        buf[byte_pos++] = (word >> 24) & 0xFF;
        buf[byte_pos++] = (word >> 16) & 0xFF;
        buf[byte_pos++] = (word >>  8) & 0xFF;
        buf[byte_pos++] =  word        & 0xFF;
    }

    // Handle remaining bits (partial word in ISR).
    // The PIO is stalled at "wait 0 pin, 0" after the stop bit.
    // The ISR contains remaining_bits, left-shifted (MSB-aligned within
    // those bits). Force-push to recover them.
    if (remaining_bits > 1 && byte_pos < expected) {
        if (byte_pos == 0) {
            // Entire response is < 32 bits (e.g. 1-byte ack).  No auto-push
            // will fire, so we can't poll the FIFO.  Wait for the controller
            // to finish processing and send the response.
            uint32_t t = time_us_32();
            while (time_us_32() - t < 50000) {
                // spin — can't detect partial ISR fill
            }
        }
        // At ~4µs/bit, remaining_bits takes at most ~128µs after data starts.
        sleep_us(200);
        pio_sm_exec(JOYBUS_PIO, port, pio_encode_push(false, false));
        if (!pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port)) {
            uint32_t word = pio_sm_get(JOYBUS_PIO, port);
            // ISR was filled with left-shifts, so data is at positions
            // [remaining_bits-1 : 0].  Shift left to align to MSB.
            word <<= (32 - remaining_bits);
            uint8_t data_bytes = (remaining_bits - 1) / 8;  // exclude stop bit
            for (uint8_t i = 0; i < data_bytes && byte_pos < 128; i++) {
                buf[byte_pos++] = (word >> 24) & 0xFF;
                word <<= 8;
            }
        }
    }

cleanup:
    // Always drain FIFO and force-push ISR to clear any stale bits.
    // Without this, leftover bits in the ISR corrupt all future joybus
    // communication on this port (bits get prepended to the next response,
    // shifting every byte by a few bits).
    while (!pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port)) {
        pio_sm_get(JOYBUS_PIO, port);
    }
    pio_sm_exec(JOYBUS_PIO, port, pio_encode_push(false, false));
    while (!pio_sm_is_rx_fifo_empty(JOYBUS_PIO, port)) {
        pio_sm_get(JOYBUS_PIO, port);
    }

    _port_raw_response_len[port] = byte_pos < expected ? byte_pos : expected;
    _port_raw_response_ready[port] = true;
    _port_raw_expect_response[port] = false;
}

void _gamecube_send_probe()
{
    pio_sm_clear_fifos(JOYBUS_PIO, 0);
    pio_set_sm_mask_enabled(JOYBUS_PIO, 0b1111, false);
    uint8_t raw_preloaded[4] = {0};
    for (uint i = 0; i < 4; i++)
    {
        switch (_port_phases[i])
        {
        default:
        case 0:
        {
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_set(pio_y, 0));
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_jmp(_gamecube_offset));
            pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(0x00));
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_set(pio_y, 7));
        }
        break;

        case 1:
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_set(pio_y, 0));
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_jmp(_gamecube_offset));
            pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(0x41));
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_set(pio_y, 7));
            break;

        case 2:
        {
            pio_sm_exec_wait_blocking(JOYBUS_PIO, i, pio_encode_jmp(_gamecube_offset + joybus_offset_joybusout));
            if (_port_raw_cmd_pending[i]) {
                // Clear RX FIFO for this port before receiving response
                pio_sm_clear_fifos(JOYBUS_PIO, i);
                // Pre-load up to 4 bytes (TX FIFO depth) while SM is disabled.
                // Remaining bytes are fed after SMs are enabled.
                uint8_t preload = _port_raw_cmd_len[i] < 4 ? _port_raw_cmd_len[i] : 4;
                for (uint8_t b = 0; b < preload; b++) {
                    pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(_port_raw_cmd_buf[i][b]));
                }
                raw_preloaded[i] = preload;
                _port_raw_cmd_pending[i] = false;
                _port_raw_expect_response[i] = true;
            } else {
                pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(0x40));
                pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(0x03));
                pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(_port_rumble[i]));
            }
        }
        break;
        }
    }
    pio_set_sm_mask_enabled(JOYBUS_PIO, 0b1111, true);

    // Feed remaining raw command bytes now that SMs are running.
    // pio_sm_put_blocking will stall if FIFO is full, which is fine
    // since the SM is consuming bytes as it transmits.
    for (uint i = 0; i < 4; i++) {
        if (raw_preloaded[i] > 0) {
            for (uint8_t b = raw_preloaded[i]; b < _port_raw_cmd_len[i]; b++) {
                pio_sm_put_blocking(JOYBUS_PIO, i, ALIGNED_JOYBUS_8(_port_raw_cmd_buf[i][b]));
            }
        }
    }
}

void joybus_itf_enable_rumble(uint8_t interface, bool enable)
{
    for (uint i = 0; i < 4; i++)
    {
        if (_port_joybus[i].port_itf == interface)
        {
            _port_rumble[i] = enable;
            break;
        }
    }
}

void joybus_itf_poll(joybus_input_s **out)
{

    *out = _port_joybus;

    if(delay_cycles>0)
    {
        delay_cycles--;
        return;
    }

    _gamecube_send_probe();

    // Drain raw responses in a tight loop before the normal sleep+read.
    // This handles responses up to 128 bytes by reading the PIO FIFO
    // continuously so it never overflows (4-word depth).
    bool had_raw = false;
    for (uint i = 0; i < 4; i++) {
        if (_port_raw_expect_response[i]) {
            _drain_raw_response(i);
            _port_raw_drained[i] = true;
            had_raw = true;
        }
    }

    // For normal ports, the 8-byte response (~260µs) fits in the FIFO.
    // If we already spent time draining a raw port, skip the extra sleep.
    if (!had_raw) {
        sleep_us(500);
    }

    _gamecube_get_data();
}

void joybus_itf_init()
{
    _gamecube_offset = pio_add_program(JOYBUS_PIO, &joybus_program);
    for (uint i = 0; i < 4; i++)
    {
        memset(&_port_joybus[i], 0, sizeof(joybus_input_s));
        _port_joybus[i].port_itf = -1;
    }

    joybus_program_init(JOYBUS_PIO, _gamecube_offset + joybus_offset_joybusout, JOYBUS_PORT_1, _gamecube_c);
    sleep_ms(100);
}