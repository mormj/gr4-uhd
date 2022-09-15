/* -*- c++ -*- */
/*
 * Copyright 2015-2016,2019 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_GR_UHD_BLOCK_IMPL_H
#define INCLUDED_GR_UHD_BLOCK_IMPL_H

#include <gnuradio/uhd/usrp_block.h>
#include <pmtf/wrap.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <functional>


namespace gr {
namespace uhd {

static const std::string ALL_GAINS = ::uhd::usrp::multi_usrp::ALL_GAINS;

#ifdef UHD_USRP_MULTI_USRP_LO_CONFIG_API
static const std::string ALL_LOS = ::uhd::usrp::multi_usrp::ALL_LOS;
#else
static const std::string ALL_LOS;
#endif
class usrp_block_impl : virtual public usrp_block
{
public:
    typedef std::function<::uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;
    typedef std::function<void(const pmtf::pmt&, int, const pmtf::pmt&)> cmd_handler_t;

    /**********************************************************************
     * Public API calls (see usrp_block.h for docs)
     **********************************************************************/
    // Getters
    ::uhd::sensor_value_t get_mboard_sensor(const std::string& name,
                                            size_t mboard) override;
    std::vector<std::string> get_mboard_sensor_names(size_t mboard) override;
    std::string get_time_source(const size_t mboard) override;
    std::vector<std::string> get_time_sources(const size_t mboard) override;
    std::string get_clock_source(const size_t mboard) override;
    std::vector<std::string> get_clock_sources(const size_t mboard) override;
    double get_clock_rate(size_t mboard) override;
    ::uhd::time_spec_t get_time_now(size_t mboard = 0) override;
    ::uhd::time_spec_t get_time_last_pps(size_t mboard) override;
    ::uhd::usrp::multi_usrp::sptr get_device(void) override;
    std::vector<std::string> get_gpio_banks(const size_t mboard) override;
    uint32_t get_gpio_attr(const std::string& bank,
                           const std::string& attr,
                           const size_t mboard = 0) override;
    size_t get_num_mboards() override;

    // Setters
    void set_time_source(const std::string& source, const size_t mboard) override;
    void set_clock_source(const std::string& source, const size_t mboard) override;
    void set_clock_rate(double rate, size_t mboard) override;
    void set_time_now(const ::uhd::time_spec_t& time_spec, size_t mboard) override;
    void set_time_next_pps(const ::uhd::time_spec_t& time_spec) override;
    void set_time_unknown_pps(const ::uhd::time_spec_t& time_spec) override;
    void set_command_time(const ::uhd::time_spec_t& time_spec, size_t mboard) override;
    void
    set_user_register(const uint8_t addr, const uint32_t data, size_t mboard) override;
    void clear_command_time(size_t mboard) override;
    void set_gpio_attr(const std::string& bank,
                       const std::string& attr,
                       const uint32_t value,
                       const uint32_t mask,
                       const size_t mboard) override;


    /**********************************************************************
     * Structors
     * ********************************************************************/
    ~usrp_block_impl() override;

protected:
    /*! \brief Components common to USRP sink and source.
     *
     * \param device_addr Device address + options
     * \param stream_args Stream args (cpu format, otw format...)
     * \param ts_tag_name If this block produces or consumes stream tags, enter the
     * corresponding tag name here
     */
    usrp_block_impl(const ::uhd::device_addr_t& device_addr,
                    const ::uhd::stream_args_t& stream_args,
                    const std::string& ts_tag_name);

    /**********************************************************************
     * Command Interface
     **********************************************************************/
    //! Receives commands and handles them
    void msg_handler_command(pmtf::pmt msg);

    //! For a given argument, call the associated handler, or if none exists,
    // show a warning through the logging interface.
    void dispatch_msg_cmd_handler(const std::string& cmd,
                                  const pmtf::pmt& val,
                                  int chan,
                                  pmtf::pmt& msg);

    //! Register a new handler for command key \p cmd
    void register_msg_cmd_handler(const std::string& cmd, cmd_handler_t handler);

    // Default handlers
    void _cmd_handler_freq(const pmtf::pmt& freq, int chan, const pmtf::pmt& msg);
    void
    _cmd_handler_looffset(const pmtf::pmt& lo_offset, int chan, const pmtf::pmt& msg);
    void _cmd_handler_gain(const pmtf::pmt& gain, int chan, const pmtf::pmt& msg);
    void _cmd_handler_power(const pmtf::pmt& power_dbm, int chan, const pmtf::pmt& msg);
    void _cmd_handler_antenna(const pmtf::pmt& ant, int chan, const pmtf::pmt& msg);
    void _cmd_handler_rate(const pmtf::pmt& rate, int chan, const pmtf::pmt& msg);
    void _cmd_handler_tune(const pmtf::pmt& tune, int chan, const pmtf::pmt& msg);
    void _cmd_handler_mtune(const pmtf::pmt& tune, int chan, const pmtf::pmt& msg);
    void _cmd_handler_bw(const pmtf::pmt& bw, int chan, const pmtf::pmt& msg);
    void _cmd_handler_lofreq(const pmtf::pmt& lofreq, int chan, const pmtf::pmt& msg);
    void _cmd_handler_dspfreq(const pmtf::pmt& dspfreq, int chan, const pmtf::pmt& msg);
    void _cmd_handler_gpio(const pmtf::pmt& gpio_attr, int chan, const pmtf::pmt& msg);
    void _cmd_handler_pc_clock_resync(const pmtf::pmt& timespec,
                                      int chan,
                                      const pmtf::pmt& msg);

    /**********************************************************************
     * Helpers
     **********************************************************************/
    bool _check_mboard_sensors_locked();

    void _update_stream_args(const ::uhd::stream_args_t& stream_args_);

    // should be const, doesn't work though 'cause missing operator=() for tune_request_t
    void _update_curr_tune_req(::uhd::tune_request_t& tune_req,
                               int chan,
                               const std::string& direction = "");

    /*! \brief Wait until a timeout or a sensor returns 'locked'.
     *
     * If a given sensor is not found, this still returns 'true', so we don't throw
     * errors or warnings if a sensor wasn't implemented.
     *
     * \returns true if the sensor locked in time or doesn't exist
     */
    bool _wait_for_locked_sensor(std::vector<std::string> sensor_names,
                                 const std::string& sensor_name,
                                 get_sensor_fn_t get_sensor_fn);

    //! Like set_center_freq(), but uses _curr_freq and _curr_lo_offset
    virtual ::uhd::tune_result_t
    _set_center_freq_from_internals(size_t chan, const std::string& direction) = 0;

    //! Calls _set_center_freq_from_internals() on all channels
    void _set_center_freq_from_internals_allchans();

    /**********************************************************************
     * Members
     *********************************************************************/
    //! Shared pointer to the underlying multi_usrp object
    ::uhd::usrp::multi_usrp::sptr _dev;
    ::uhd::stream_args_t _stream_args;
    //! Number of channels (i.e. number of in- or outputs)
    size_t _nchan;
    bool _stream_now;
    ::uhd::time_spec_t _start_time;
    bool _start_time_set;
    bool _force_tune;

    /****** Command interface related **********/
    //! Stores a list of commands for later execution
    std::vector<pmtf::pmt> _pending_cmds;
    //! Shadows the last value we told the USRP to tune to for every channel
    // (this is not necessarily the true value the USRP is currently tuned to!).
    std::vector<::uhd::tune_request_t> _curr_tx_tune_req;
    std::vector<::uhd::tune_request_t> _curr_rx_tune_req;
    std::vector<bool> _tx_chans_to_tune;
    std::vector<bool> _rx_chans_to_tune;

    //! Stores the individual command handlers
    ::uhd::dict<std::string, cmd_handler_t> _msg_cmd_handlers;

    //! Will check a command for a direction key, if it does not exist this will use
    // the default value for the block via _direction() defined below
    const std::string get_cmd_or_default_direction(const pmtf::pmt& cmd) const;
    //! Block direction overloaded by block impl to return "RX"/"TX" for source/sink
    virtual const std::string& _direction() const = 0;
};

} /* namespace uhd */
} /* namespace gr */

#endif /* INCLUDED_GR_UHD_BLOCK_IMPL_H */
