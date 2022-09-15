/* -*- c++ -*- */
/*
 * Copyright 2015-2016,2019 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include "usrp_block_impl.h"
#include <chrono>
#include <thread>

using namespace gr::uhd;
using namespace std::chrono_literals;

namespace {
constexpr auto LOCK_TIMEOUT = 1.5s;
}

/**********************************************************************
 * Structors
 *********************************************************************/
usrp_block::usrp_block(const std::string& name) : sync_block(name, "uhd")
{
    // nop
}

const std::string& gr::uhd::cmd_chan_key()
{
    static const std::string val("chan");
    return val;
}
const std::string& gr::uhd::cmd_gain_key()
{
    static const std::string val("gain");
    return val;
}
const std::string& gr::uhd::cmd_power_key()
{
    static const std::string val("power_dbm");
    return val;
}
const std::string& gr::uhd::cmd_freq_key()
{
    static const std::string val("freq");
    return val;
}
const std::string& gr::uhd::cmd_lo_offset_key()
{
    static const std::string val("lo_offset");
    return val;
}
const std::string& gr::uhd::cmd_tune_key()
{
    static const std::string val("tune");
    return val;
}
const std::string& gr::uhd::cmd_mtune_key()
{
    static const std::string val("mtune");
    return val;
}
const std::string& gr::uhd::cmd_lo_freq_key()
{
    static const std::string val("lo_freq");
    return val;
}
const std::string& gr::uhd::cmd_dsp_freq_key()
{
    static const std::string val("dsp_freq");
    return val;
}
const std::string& gr::uhd::cmd_rate_key()
{
    static const std::string val("rate");
    return val;
}
const std::string& gr::uhd::cmd_bandwidth_key()
{
    static const std::string val("bandwidth");
    return val;
}
const std::string& gr::uhd::cmd_time_key()
{
    static const std::string val("time");
    return val;
}
const std::string& gr::uhd::cmd_mboard_key()
{
    static const std::string val("mboard");
    return val;
}
const std::string& gr::uhd::cmd_antenna_key()
{
    static const std::string val("antenna");
    return val;
}
const std::string& gr::uhd::cmd_direction_key()
{
    static const std::string val("direction");
    return val;
}
const std::string& gr::uhd::cmd_tag_key()
{
    static const std::string val("tag");
    return val;
}
const std::string& gr::uhd::cmd_pc_clock_resync_key()
{
    static const std::string val("pc_clock_resync");
    return val;
}

const std::string& gr::uhd::cmd_gpio_key()
{
    static const std::string val("gpio");
    return val;
}

const std::string& gr::uhd::direction_rx()
{
    static const std::string val("RX");
    return val;
}
const std::string& gr::uhd::direction_tx()
{
    static const std::string val("TX");
    return val;
}

usrp_block_impl::usrp_block_impl(const ::uhd::device_addr_t& device_addr,
                                 const ::uhd::stream_args_t& stream_args,
                                 const std::string& ts_tag_name)
    : _stream_args(stream_args),
      _nchan(stream_args.channels.size()),
      _stream_now(_nchan == 1 and ts_tag_name.empty()),
      _start_time_set(false),
      _force_tune(false),
      _curr_tx_tune_req(stream_args.channels.size(), ::uhd::tune_request_t()),
      _curr_rx_tune_req(stream_args.channels.size(), ::uhd::tune_request_t()),
      _tx_chans_to_tune(stream_args.channels.size()),
      _rx_chans_to_tune(stream_args.channels.size())
{
    _dev = ::uhd::usrp::multi_usrp::make(device_addr);

    _check_mboard_sensors_locked();

    auto msg_in = message_port::make("cmd", port_direction_t::INPUT, true);
    msg_in->register_callback([this](pmtf::pmt msg) { this->msg_handler_command(msg); });

    add_port(std::move(msg_in));

// cuz we lazy:
#define REGISTER_CMD_HANDLER(key, _handler)                                 \
    register_msg_cmd_handler(                                               \
        key, [this](const pmtf::pmt& var, int chan, const pmtf::pmt& msg) { \
            this->_handler(var, chan, msg);                                 \
        })
    // Register default command handlers:
    REGISTER_CMD_HANDLER(cmd_freq_key(), _cmd_handler_freq);
    REGISTER_CMD_HANDLER(cmd_gain_key(), _cmd_handler_gain);
    REGISTER_CMD_HANDLER(cmd_power_key(), _cmd_handler_power);
    REGISTER_CMD_HANDLER(cmd_lo_offset_key(), _cmd_handler_looffset);
    REGISTER_CMD_HANDLER(cmd_tune_key(), _cmd_handler_tune);
    REGISTER_CMD_HANDLER(cmd_mtune_key(), _cmd_handler_mtune);
    REGISTER_CMD_HANDLER(cmd_lo_freq_key(), _cmd_handler_lofreq);
    REGISTER_CMD_HANDLER(cmd_dsp_freq_key(), _cmd_handler_dspfreq);
    REGISTER_CMD_HANDLER(cmd_rate_key(), _cmd_handler_rate);
    REGISTER_CMD_HANDLER(cmd_bandwidth_key(), _cmd_handler_bw);
    REGISTER_CMD_HANDLER(cmd_antenna_key(), _cmd_handler_antenna);
    REGISTER_CMD_HANDLER(cmd_gpio_key(), _cmd_handler_gpio);
    REGISTER_CMD_HANDLER(cmd_pc_clock_resync_key(), _cmd_handler_pc_clock_resync);
}

usrp_block_impl::~usrp_block_impl()
{
    // nop
}

/**********************************************************************
 * Helpers
 *********************************************************************/
void usrp_block_impl::_update_stream_args(const ::uhd::stream_args_t& stream_args_)
{
    ::uhd::stream_args_t stream_args(stream_args_);
    if (stream_args.channels.empty()) {
        stream_args.channels = _stream_args.channels;
    }
    if (stream_args.cpu_format != _stream_args.cpu_format ||
        stream_args.channels.size() != _stream_args.channels.size()) {
        throw std::runtime_error(
            "Cannot change I/O signatures while updating stream args!");
    }
    _stream_args = stream_args;
}

bool usrp_block_impl::_wait_for_locked_sensor(std::vector<std::string> sensor_names,
                                              const std::string& sensor_name,
                                              get_sensor_fn_t get_sensor_fn)
{
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) ==
        sensor_names.end()) {
        return true;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto timeout = start + LOCK_TIMEOUT;

    while (std::chrono::steady_clock::now() < timeout) {
        if (get_sensor_fn(sensor_name).to_bool()) {
            return true;
        }

        std::this_thread::sleep_for(100ms);
    }

    // One last try:
    return get_sensor_fn(sensor_name).to_bool();
}

bool usrp_block_impl::_check_mboard_sensors_locked()
{
    bool clocks_locked = true;

    // Check ref lock for all mboards
    for (size_t mboard_index = 0; mboard_index < _dev->get_num_mboards();
         mboard_index++) {
        std::string sensor_name = "ref_locked";
        if (_dev->get_clock_source(mboard_index) == "internal") {
            continue;
        } else if (_dev->get_clock_source(mboard_index) == "mimo") {
            sensor_name = "mimo_locked";
        }
        if (not _wait_for_locked_sensor(get_mboard_sensor_names(mboard_index),
                                        sensor_name,
                                        [this, mboard_index](const std::string& name) {
                                            return static_cast<::uhd::sensor_value_t>(
                                                this->get_mboard_sensor(name,
                                                                        mboard_index));
                                        })) {
            d_logger->warn(
                "Sensor '{:s}' failed to lock within timeout on motherboard {:d}.",
                sensor_name,
                mboard_index);
            clocks_locked = false;
        }
    }

    return clocks_locked;
}

void usrp_block_impl::_set_center_freq_from_internals_allchans()
{
    for (size_t chan = 0; chan < _rx_chans_to_tune.size(); chan++) {
        if (_rx_chans_to_tune[chan]) {
            _set_center_freq_from_internals(chan, direction_rx());
            _rx_chans_to_tune[chan] = false;
        }
    }

    for (size_t chan = 0; chan < _tx_chans_to_tune.size(); chan++) {
        if (_tx_chans_to_tune[chan]) {
            _set_center_freq_from_internals(chan, direction_tx());
            _tx_chans_to_tune[chan] = false;
        }
    }
}


/**********************************************************************
 * Public API calls
 *********************************************************************/
::uhd::sensor_value_t usrp_block_impl::get_mboard_sensor(const std::string& name,
                                                         size_t mboard)
{
    return _dev->get_mboard_sensor(name, mboard);
}

std::vector<std::string> usrp_block_impl::get_mboard_sensor_names(size_t mboard)
{
    return _dev->get_mboard_sensor_names(mboard);
}

void usrp_block_impl::set_time_source(const std::string& source, const size_t mboard)
{
    return _dev->set_time_source(source, mboard);
}

std::string usrp_block_impl::get_time_source(const size_t mboard)
{
    return _dev->get_time_source(mboard);
}

std::vector<std::string> usrp_block_impl::get_time_sources(const size_t mboard)
{
    return _dev->get_time_sources(mboard);
}

void usrp_block_impl::set_clock_source(const std::string& source, const size_t mboard)
{
    return _dev->set_clock_source(source, mboard);
}

std::string usrp_block_impl::get_clock_source(const size_t mboard)
{
    return _dev->get_clock_source(mboard);
}

std::vector<std::string> usrp_block_impl::get_clock_sources(const size_t mboard)
{
    return _dev->get_clock_sources(mboard);
}

double usrp_block_impl::get_clock_rate(size_t mboard)
{
    return _dev->get_master_clock_rate(mboard);
}

void usrp_block_impl::set_clock_rate(double rate, size_t mboard)
{
    return _dev->set_master_clock_rate(rate, mboard);
}

::uhd::time_spec_t usrp_block_impl::get_time_now(size_t mboard)
{
    return _dev->get_time_now(mboard);
}

::uhd::time_spec_t usrp_block_impl::get_time_last_pps(size_t mboard)
{
    return _dev->get_time_last_pps(mboard);
}

std::vector<std::string> usrp_block_impl::get_gpio_banks(const size_t mboard)
{
    return _dev->get_gpio_banks(mboard);
}

uint32_t usrp_block_impl::get_gpio_attr(const std::string& bank,
                                        const std::string& attr,
                                        const size_t mboard)
{
    return _dev->get_gpio_attr(bank, attr, mboard);
}

void usrp_block_impl::set_time_now(const ::uhd::time_spec_t& time_spec, size_t mboard)
{
    return _dev->set_time_now(time_spec, mboard);
}

void usrp_block_impl::set_time_next_pps(const ::uhd::time_spec_t& time_spec)
{
    return _dev->set_time_next_pps(time_spec);
}

void usrp_block_impl::set_time_unknown_pps(const ::uhd::time_spec_t& time_spec)
{
    return _dev->set_time_unknown_pps(time_spec);
}

void usrp_block_impl::set_command_time(const ::uhd::time_spec_t& time_spec, size_t mboard)
{
    return _dev->set_command_time(time_spec, mboard);
}

void usrp_block_impl::clear_command_time(size_t mboard)
{
    return _dev->clear_command_time(mboard);
}

void usrp_block_impl::set_user_register(const uint8_t addr,
                                        const uint32_t data,
                                        size_t mboard)
{
    _dev->set_user_register(addr, data, mboard);
}

void usrp_block_impl::set_gpio_attr(const std::string& bank,
                                    const std::string& attr,
                                    const uint32_t value,
                                    const uint32_t mask,
                                    const size_t mboard)
{
    return _dev->set_gpio_attr(bank, attr, value, mask, mboard);
}

::uhd::usrp::multi_usrp::sptr usrp_block_impl::get_device(void) { return _dev; }

size_t usrp_block_impl::get_num_mboards() { return _dev->get_num_mboards(); }

/**********************************************************************
 * External Interfaces
 *********************************************************************/

void usrp_block_impl::msg_handler_command(pmtf::pmt msg)
{
    auto map = pmtf::map(msg);

    /*** Start the actual message processing *************************/
    /// 1) Check if there's a time stamp
    if (map.count(cmd_time_key())) {
        size_t mboard_index = !map.count(cmd_mboard_key())
                                  ? ::uhd::usrp::multi_usrp::ALL_MBOARDS
                                  : pmtf::get_as<size_t>(map[cmd_mboard_key()]);
        if (map.count(cmd_time_key())) {
            auto timespec_p = pmtf::vector<pmtf::pmt>(map[cmd_time_key()]);
            ::uhd::time_spec_t timespec(
                time_t(pmtf::get_as<long>(timespec_p[0])), // Full secs
                pmtf::get_as<double>(timespec_p[1])        // Frac secs
            );
            d_debug_logger->debug("Setting command time on mboard {}", mboard_index);
            set_command_time(timespec, mboard_index);
        } else {
            clear_command_time(mboard_index);
        }
    }

    /// 2) Read chan value
    size_t chan =
        !map.count(cmd_chan_key()) ? -1 : pmtf::get_as<size_t>(map[cmd_chan_key()]);


    /// 3) If a direction key was specified, force the block to tune - see issue #1814
    _force_tune = map.count(cmd_direction_key()) > 0;

    /// 4) Loop through all the values
    // d_debug_logger->debug("Processing command message {}", pmt::write_string(msg));
    for (const auto& [key, value] : map) {
        try {
            dispatch_msg_cmd_handler(pmtf::get_as<std::string>(key), value, chan, msg);
        } catch (const char* message) {
        }
        // FIXME: Handle exceptions
        // catch (pmt::wrong_type& e) {
        //     // d_logger->alert("Invalid command value for key {}: {}",
        //     //                 pmt::write_string(pmt::car(pmt::nth(i, msg_items))),
        //     //                 pmt::write_string(pmt::cdr(pmt::nth(i, msg_items))));
        //     break;
        // }
    }

    /// 5) Check if we need to re-tune
    _set_center_freq_from_internals_allchans();
    _force_tune = false;
}


void usrp_block_impl::dispatch_msg_cmd_handler(const std::string& cmd,
                                               const pmtf::pmt& val,
                                               int chan,
                                               pmtf::pmt& msg)
{
    if (_msg_cmd_handlers.has_key(cmd)) {
        _msg_cmd_handlers[cmd](val, chan, msg);
    }
}

void usrp_block_impl::register_msg_cmd_handler(const std::string& cmd,
                                               cmd_handler_t handler)
{
    _msg_cmd_handlers[cmd] = handler;
}

void usrp_block_impl::_update_curr_tune_req(::uhd::tune_request_t& tune_req,
                                            int chan,
                                            const std::string& direction)
{
    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            _update_curr_tune_req(tune_req, int(i), direction);
        }
        return;
    }

    if (direction == direction_rx()) {
        if (tune_req.target_freq != _curr_rx_tune_req[chan].target_freq ||
            tune_req.rf_freq_policy != _curr_rx_tune_req[chan].rf_freq_policy ||
            tune_req.rf_freq != _curr_rx_tune_req[chan].rf_freq ||
            tune_req.dsp_freq != _curr_rx_tune_req[chan].dsp_freq ||
            tune_req.dsp_freq_policy != _curr_rx_tune_req[chan].dsp_freq_policy ||
            _force_tune) {
            _curr_rx_tune_req[chan] = tune_req;
            _rx_chans_to_tune[chan] = true;
        }
    } else {
        if (tune_req.target_freq != _curr_tx_tune_req[chan].target_freq ||
            tune_req.rf_freq_policy != _curr_tx_tune_req[chan].rf_freq_policy ||
            tune_req.rf_freq != _curr_tx_tune_req[chan].rf_freq ||
            tune_req.dsp_freq != _curr_tx_tune_req[chan].dsp_freq ||
            tune_req.dsp_freq_policy != _curr_tx_tune_req[chan].dsp_freq_policy ||
            _force_tune) {
            _curr_tx_tune_req[chan] = tune_req;
            _tx_chans_to_tune[chan] = true;
        }
    }
}

// Default handlers:
void usrp_block_impl::_cmd_handler_freq(const pmtf::pmt& freq_,
                                        int chan,
                                        const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);

    double freq = pmtf::get_as<double>(freq_);
    ::uhd::tune_request_t new_tune_request(freq);
    auto map = pmtf::map(msg);
    if (map.count(cmd_lo_offset_key())) {
        double lo_offset = pmtf::get_as<double>(map[cmd_lo_offset_key()]);
        new_tune_request = ::uhd::tune_request_t(freq, lo_offset);
    }

    _update_curr_tune_req(new_tune_request, chan, direction);
}

void usrp_block_impl::_cmd_handler_looffset(const pmtf::pmt& lo_offset,
                                            int chan,
                                            const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);

    auto map = pmtf::map(msg);
    if (map.count(cmd_freq_key())) {
        // Then it's already taken care of
        return;
    }

    double lo_offs = pmtf::get_as<double>(lo_offset);
    ::uhd::tune_request_t new_tune_request;
    if (direction == direction_rx()) {
        new_tune_request = _curr_rx_tune_req[chan];
    } else {
        new_tune_request = _curr_tx_tune_req[chan];
    }

    new_tune_request.rf_freq = new_tune_request.target_freq + lo_offs;
    new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;
    new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_AUTO;

    _update_curr_tune_req(new_tune_request, chan, direction);
}

void usrp_block_impl::_cmd_handler_gain(const pmtf::pmt& gain_,
                                        int chan,
                                        const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);

    double gain = pmtf::get_as<double>(gain_);
    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            set_gain(gain, i, direction);
        }
        return;
    }

    set_gain(gain, chan, direction);
}

void usrp_block_impl::_cmd_handler_power(const pmtf::pmt& power_dbm_,
                                         int chan,
                                         const pmtf::pmt& msg)
{
    double power_dbm = pmtf::get_as<double>(power_dbm_);
    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            set_power_reference(power_dbm, i);
        }
        return;
    }

    set_power_reference(power_dbm, chan);
}

void usrp_block_impl::_cmd_handler_antenna(const pmtf::pmt& ant,
                                           int chan,
                                           const pmtf::pmt& msg)
{
    const auto antenna = pmtf::get_as<std::string>(ant);
    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            set_antenna(antenna, i);
        }
        return;
    }

    set_antenna(antenna, chan);
}

void usrp_block_impl::_cmd_handler_gpio(const pmtf::pmt& gpio_attr,
                                        int chan,
                                        const pmtf::pmt& msg)
{
    auto map = pmtf::map(msg);
    auto mboard =
        pmtf::get_as<size_t>(!map.count(cmd_mboard_key()) ? 0 : map[cmd_mboard_key()]);

    // will throw if not a map
    auto gpio_attr_m = pmtf::map(gpio_attr);

    if (!(gpio_attr_m.count("bank") && gpio_attr_m.count("attr") &&
          gpio_attr_m.count("value") && gpio_attr_m.count("mask"))) {
        d_logger->error("gpio_attr message must include bank, attr, value and mask");
        return;
    }

    auto bank = pmtf::get_as<std::string>(gpio_attr_m["bank"]);
    auto attr = pmtf::get_as<std::string>(gpio_attr_m["attr"]);
    auto value = pmtf::get_as<uint32_t>(gpio_attr_m["value"]);
    auto mask = pmtf::get_as<uint32_t>(gpio_attr_m["mask"]);

    set_gpio_attr(bank, attr, value, mask, mboard);
}

void usrp_block_impl::_cmd_handler_rate(const pmtf::pmt& rate_, int, const pmtf::pmt&)
{
    const auto rate = pmtf::get_as<double>(rate_);
    set_samp_rate(rate);
}

void usrp_block_impl::_cmd_handler_pc_clock_resync(const pmtf::pmt&,
                                                   int,
                                                   const pmtf::pmt&)
{
    const uint64_t ticks =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    const ::uhd::time_spec_t& time_spec = ::uhd::time_spec_t::from_ticks(ticks, 1.0e9);
    set_time_now(time_spec, ::uhd::usrp::multi_usrp::ALL_MBOARDS);
}

void usrp_block_impl::_cmd_handler_tune(const pmtf::pmt& tune,
                                        int chan,
                                        const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);

    auto tune_v = pmtf::vector<pmtf::pmt>(tune);
    double freq = pmtf::get_as<double>(tune_v[0]);
    double lo_offset = pmtf::get_as<double>(tune_v[1]);
    ::uhd::tune_request_t new_tune_request(freq, lo_offset);
    _update_curr_tune_req(new_tune_request, chan, direction);
}

void usrp_block_impl::_cmd_handler_mtune(const pmtf::pmt& tune,
                                         int chan,
                                         const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);
    auto tune_m = pmtf::map(tune);
    ::uhd::tune_request_t new_tune_request;
    if (tune_m.count("dsp_freq")) {
        new_tune_request.dsp_freq = pmtf::get_as<double>(tune_m["dsp_freq"]);
    }
    if (tune_m.count("rf_freq")) {
        new_tune_request.rf_freq = pmtf::get_as<double>(tune_m["rf_freq"]);
    }
    if (tune_m.count("target_freq")) {
        new_tune_request.target_freq = pmtf::get_as<double>(tune_m["target_freq"]);
    }
    if (tune_m.count("dsp_freq_policy")) {
        std::string policy = pmtf::get_as<std::string>(tune_m["dsp_freq_policy"]);
        if (policy == "M") {
            new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;
        } else if (policy == "A") {
            new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_AUTO;
        } else {
            new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_NONE;
        }
    }
    if (tune_m.count("rf_freq_policy")) {
        std::string policy = pmtf::get_as<std::string>(tune_m["rf_freq_policy"]);
        if (policy == "M") {
            new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;
        } else if (policy == "A") {
            new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_AUTO;
        } else {
            new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_NONE;
        }
    }
    if (tune_m.count("args")) {
        new_tune_request.args =
            ::uhd::device_addr_t(pmtf::get_as<std::string>(tune_m["args"]));
    }

    _update_curr_tune_req(new_tune_request, chan, direction);
}

void usrp_block_impl::_cmd_handler_bw(const pmtf::pmt& bw, int chan, const pmtf::pmt& msg)
{
    double bandwidth = pmtf::get_as<double>(bw);
    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            set_bandwidth(bandwidth, i);
        }
        return;
    }

    set_bandwidth(bandwidth, chan);
}

void usrp_block_impl::_cmd_handler_lofreq(const pmtf::pmt& lofreq,
                                          int chan,
                                          const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);

    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            _cmd_handler_lofreq(lofreq, int(i), msg);
        }
        return;
    }

    ::uhd::tune_request_t new_tune_request;
    if (direction == direction_rx()) {
        new_tune_request = _curr_rx_tune_req[chan];
    } else {
        new_tune_request = _curr_tx_tune_req[chan];
    }

    auto map = pmtf::map(msg);
    new_tune_request.rf_freq = pmtf::get_as<double>(lofreq);
    if (map.count(cmd_dsp_freq_key())) {
        new_tune_request.dsp_freq = pmtf::get_as<double>(map[cmd_dsp_freq_key()]);
    }
    new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;
    new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;

    _update_curr_tune_req(new_tune_request, chan, direction);
}

void usrp_block_impl::_cmd_handler_dspfreq(const pmtf::pmt& dspfreq,
                                           int chan,
                                           const pmtf::pmt& msg)
{
    // Get the direction key
    const auto direction = get_cmd_or_default_direction(msg);
    auto map = pmtf::map(msg);

    if (map.count(cmd_lo_freq_key())) {
        // Then it's already dealt with
        return;
    }

    if (chan == -1) {
        for (size_t i = 0; i < _nchan; i++) {
            _cmd_handler_dspfreq(dspfreq, int(i), msg);
        }
        return;
    }

    ::uhd::tune_request_t new_tune_request;
    if (direction == direction_rx()) {
        new_tune_request = _curr_rx_tune_req[chan];
    } else {
        new_tune_request = _curr_tx_tune_req[chan];
    }

    new_tune_request.dsp_freq = pmtf::get_as<double>(dspfreq);
    new_tune_request.rf_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;
    new_tune_request.dsp_freq_policy = ::uhd::tune_request_t::POLICY_MANUAL;

    _update_curr_tune_req(new_tune_request, chan, direction);
}


const std::string
usrp_block_impl::get_cmd_or_default_direction(const pmtf::pmt& cmd) const
{
    auto map = pmtf::map(cmd);
    if (map.count(cmd_direction_key())) {
        auto str = pmtf::get_as<std::string>(map[cmd_direction_key()]);
        if (str == direction_rx() || str == direction_tx()) {
            return str;
        }
    }
    return _direction();
}
