#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "pulse.pio.h"

const uint PIN_ENA = 11;
const uint PIN_DIR = 12;
const uint PIN_PUL = 13;
const uint PIN_LIM = 14; // リミットスイッチまたは代わりのセンサー

class StepperMotor {
    public:
    StepperMotor(uint pin_ena, uint pin_dir, uint pin_pul, uint pin_lim, uint8_t pio, uint8_t irq, uint32_t *pulses, uint8_t num_pulse, bool direction) {
        ena_ = pin_ena;
        dir_ = pin_dir;
        pul_ = pin_pul;
        lim_ = pin_lim;
        pio_ = pio == 0 ? pio0 : pio1;
        irq_ = irq;
        pulses_ = pulses;
        num_pulse_ = num_pulse;
        direction_ = direction;
        idx_pulse_ = 0;
        if (irq_ == 0) {
            irq_num_ = pio == 0 ? PIO0_IRQ_0 : PIO1_IRQ_0;
        } else {
            irq_num_ = pio == 0 ? PIO0_IRQ_1 : PIO1_IRQ_1;
        }
        sm_ = pio_claim_unused_sm(pio_, true);
        offset_ = pio_add_program(pio_, &pulse_program);
        pulse_program_init(pio_, sm_, offset_, pul_, lim_);
        gpio_init(ena_);
        gpio_init(dir_);
        gpio_set_dir(ena_, GPIO_OUT);
        gpio_set_dir(dir_, GPIO_OUT);
    }

    void start() {
        irq_set_enabled(irq_num_, true);
        pio_set_irq0_source_enabled(pio_, (enum pio_interrupt_source) ((uint) pis_interrupt0 + sm_), true);
        gpio_put(ena_, true);
        gpio_put(dir_, direction_);
        pio_sm_set_enabled(pio_, sm_, true);
        pio_sm_put_blocking(pio_, sm_, pulses_[idx_pulse_++]);
    }

    void irq_handler() {
        pio_interrupt_clear(pio_, irq_);
        if (idx_pulse_ < num_pulse_) {
            pio_sm_put_blocking(pio_, sm_, pulses_[idx_pulse_++]);
        } else {
            pio_sm_set_enabled(pio_, sm_, false);
            pio_set_irq0_source_enabled(pio_, (enum pio_interrupt_source) ((uint) pis_interrupt0 + sm_), false);
            irq_set_enabled(irq_num_, false);
            gpio_put(ena_, false);
        }
    }

    void set_pulses(uint32_t *pulses, uint8_t num_pulse, bool direction) {
        pulses_ = pulses;
        num_pulse_ = num_pulse;
        direction_ = direction;
        idx_pulse_ = 0;
    }

    private:
    uint ena_;
    uint dir_;
    uint pul_;
    uint lim_;
    PIO pio_;
    uint8_t irq_;
    uint irq_num_;
    uint sm_;
    uint offset_;
    uint32_t *pulses_;
    uint8_t num_pulse_;
    uint8_t idx_pulse_;
    bool direction_ = true;
};

StepperMotor* smtr = nullptr;

void irq_handler() {
    if (smtr != nullptr) {
        smtr->irq_handler();
    }
}

uint32_t create_pulse_req(uint16_t count, uint16_t interval) {
    uint32_t req = 0;
    req = interval;
    req <<= 16;
    req |= count;
    return req;
}

int main() {
    uint32_t t_last, t_delta;
    bool direction = true;
    uint32_t pulses[7] = {
        create_pulse_req(100, 4),
        create_pulse_req(200, 3),
        create_pulse_req(100, 2),
        create_pulse_req(800, 1),
        create_pulse_req(100, 2),
        create_pulse_req(200, 3),
        create_pulse_req(100, 4)
    };

    stdio_init_all();
    smtr = new StepperMotor(PIN_ENA, PIN_DIR, PIN_PUL, PIN_LIM, 0, 0, pulses, 7, direction);
    irq_set_exclusive_handler(PIO0_IRQ_0, irq_handler);

    t_last = time_us_32();
    smtr->start();

    while (true) {
        t_delta = time_us_32() - t_last;
        if (t_delta >= 2500000) {
            direction = !direction;
            pulses[0] = create_pulse_req(100, 4);
            pulses[1] = create_pulse_req(200, 3);
            pulses[2] = create_pulse_req(100, 2);
            pulses[3] = create_pulse_req(800, 1);
            pulses[4] = create_pulse_req(100, 2);
            pulses[5] = create_pulse_req(200, 3);
            pulses[6] = create_pulse_req(100, 4);
            smtr->set_pulses(pulses, 7, direction);
            smtr->start();
            t_last = time_us_32();
            printf("reverse!\n");
        }
    }
}
