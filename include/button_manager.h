#pragma once

#include <vector>
#include "TCA9555.h"

const double TRANS_TIME = 0.5;
const bool PRESSED = 0;

class ButtonState
{
public:
    ButtonState(TCA9535 * tca, uint8_t pin, double t) : _tca(tca), _pin(pin), _pressed(false)
    {
        _t_last_unpressed = t;
    }

    bool is_pressed() const { return _pressed; }

    void update(double t)
    {
        bool new_state = _tca->read1(_pin);
        if (new_state != PRESSED){
            _t_last_unpressed = t;
        }
        _pressed = (t - _t_last_unpressed) > TRANS_TIME;
    }

private:
    TCA9535 * _tca;
    uint8_t _pin;
    double _t_last_unpressed;
    bool _curr_state;
    bool _pressed;
};

class ButtonManager
{
public:
    ButtonManager(const std::vector<uint8_t>& pins, double t) : _TCA(0x27)
    {
        _TCA.begin();
        for (const auto pin : pins)
        {
            _button_states.insert(std::make_pair(pin, new ButtonState(&_TCA, pin, t)));
        }
    }
    void update(double t)
    {
        for (auto &pair : _button_states)
        {
            pair.second->update(t);
        }
    }

    const ButtonState& get_button_state(uint8_t pin) const
    {
        return *_button_states.at(pin);
    }

private:
    std::unordered_map<uint8_t, ButtonState *> _button_states;
    TCA9535 _TCA;
};