#pragma once

namespace simuav {

inline SimConfig loadConfig(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("ConfigLoader: cannot open '" + path + "'");

    const nlohmann::json j = nlohmann::json::parse(f);

    SimConfig cfg;

    cfg.dt             = j.value("dt",             cfg.dt);
    cfg.mavlink_host   = j.value("mavlink_host",   cfg.mavlink_host);
    cfg.mavlink_port        = j.value("mavlink_port",        cfg.mavlink_port);
    cfg.mavlink_local_port  = j.value("mavlink_local_port",  cfg.mavlink_local_port);
    cfg.json_log_path       = j.value("json_log_path",       cfg.json_log_path);
    cfg.ulog_path      = j.value("ulog_path",      cfg.ulog_path);
    cfg.status_port    = j.value("status_port",    cfg.status_port);

    {
        const std::string ft = j.value("firmware_target", std::string("px4"));
        cfg.firmware_target = (ft == "ardupilot") ? FirmwareTarget::ArduPilot
                                                   : FirmwareTarget::PX4;
    }

    if (j.contains("quad_params")) {
        const auto& q      = j.at("quad_params");
        auto& p            = cfg.quad_params;
        p.mass             = q.value("mass",            p.mass);
        p.arm_length       = q.value("arm_length",      p.arm_length);
        p.k_thrust         = q.value("k_thrust",        p.k_thrust);
        p.k_drag           = q.value("k_drag",          p.k_drag);
        p.aero_drag        = q.value("aero_drag",       p.aero_drag);
        p.max_motor_speed  = q.value("max_motor_speed", p.max_motor_speed);
        p.esc_exponent     = q.value("esc_exponent",    p.esc_exponent);
        p.motor_spin_min          = q.value("motor_spin_min",          p.motor_spin_min);
        p.use_rk4                 = q.value("use_rk4",                 p.use_rk4);
        p.motor_time_constant_s   = q.value("motor_time_constant_s",   p.motor_time_constant_s);
        if (q.contains("inertia_diag") && q.at("inertia_diag").size() == 3) {
            auto& arr = q.at("inertia_diag");
            p.inertia_diag = {arr[0].get<double>(),
                              arr[1].get<double>(),
                              arr[2].get<double>()};
        }
    }

    if (j.contains("wind_params")) {
        const auto& w  = j.at("wind_params");
        auto& p        = cfg.wind_params;
        p.gust_probability  = w.value("gust_probability",  p.gust_probability);
        p.gust_magnitude    = w.value("gust_magnitude",    p.gust_magnitude);
        p.use_dryden        = w.value("use_dryden",        p.use_dryden);
        p.turbulence_std    = w.value("turbulence_std",    p.turbulence_std);
        p.dryden_airspeed   = w.value("dryden_airspeed",  p.dryden_airspeed);
        p.dryden_length_u   = w.value("dryden_length_u",  p.dryden_length_u);
        p.dryden_length_v   = w.value("dryden_length_v",  p.dryden_length_v);
        p.dryden_length_w   = w.value("dryden_length_w",  p.dryden_length_w);
        p.dryden_sigma_u    = w.value("dryden_sigma_u",   p.dryden_sigma_u);
        p.dryden_sigma_v    = w.value("dryden_sigma_v",   p.dryden_sigma_v);
        p.dryden_sigma_w    = w.value("dryden_sigma_w",   p.dryden_sigma_w);
        if (w.contains("mean_ned") && w.at("mean_ned").size() == 3) {
            auto& arr = w.at("mean_ned");
            p.mean_ned = {arr[0].get<double>(),
                          arr[1].get<double>(),
                          arr[2].get<double>()};
        }
    }

    if (j.contains("imu_params")) {
        const auto& i  = j.at("imu_params");
        auto& p        = cfg.imu_params;
        p.accel_arw_std                = i.value("accel_arw_std",                p.accel_arw_std);
        p.accel_bias_instability       = i.value("accel_bias_instability",       p.accel_bias_instability);
        p.accel_bias_instability_tc_s  = i.value("accel_bias_instability_tc_s",  p.accel_bias_instability_tc_s);
        p.gyro_arw_std                 = i.value("gyro_arw_std",                 p.gyro_arw_std);
        p.gyro_bias_instability        = i.value("gyro_bias_instability",        p.gyro_bias_instability);
        p.gyro_bias_instability_tc_s   = i.value("gyro_bias_instability_tc_s",   p.gyro_bias_instability_tc_s);
        p.vibration_amplitude_mps2     = i.value("vibration_amplitude_mps2",     p.vibration_amplitude_mps2);
        p.vibration_frequency_hz       = i.value("vibration_frequency_hz",       p.vibration_frequency_hz);
    }

    if (j.contains("gps_params")) {
        const auto& g  = j.at("gps_params");
        auto& p        = cfg.gps_params;
        p.lat_ref_deg      = g.value("lat_ref_deg",      p.lat_ref_deg);
        p.lon_ref_deg      = g.value("lon_ref_deg",      p.lon_ref_deg);
        p.alt_ref_m        = g.value("alt_ref_m",        p.alt_ref_m);
        p.pos_noise_std_m  = g.value("pos_noise_std_m",  p.pos_noise_std_m);
        p.alt_noise_std_m  = g.value("alt_noise_std_m",  p.alt_noise_std_m);
        p.vel_noise_std    = g.value("vel_noise_std",    p.vel_noise_std);
        p.update_rate_hz   = g.value("update_rate_hz",   p.update_rate_hz);
        p.num_sats         = g.value("num_sats",         p.num_sats);
        p.fix_type         = g.value("fix_type",         p.fix_type);
    }

    if (j.contains("baro_params")) {
        const auto& b  = j.at("baro_params");
        auto& p        = cfg.baro_params;
        p.alt_ref_m            = b.value("alt_ref_m",            p.alt_ref_m);
        p.noise_std_m          = b.value("noise_std_m",          p.noise_std_m);
        p.ground_temp_offset_c = b.value("ground_temp_offset_c", p.ground_temp_offset_c);
    }

    if (j.contains("mag_params")) {
        const auto& m  = j.at("mag_params");
        auto& p        = cfg.mag_params;
        p.noise_std    = m.value("noise_std", p.noise_std);
        if (m.contains("earth_field_ned") && m.at("earth_field_ned").size() == 3) {
            auto& arr = m.at("earth_field_ned");
            p.earth_field_ned = {arr[0].get<double>(),
                                 arr[1].get<double>(),
                                 arr[2].get<double>()};
        }
    }

    return cfg;
}

inline SimConfig tryLoadConfig(const std::string& path) {
    try {
        return loadConfig(path);
    } catch (const std::runtime_error&) {
        // file not found — silently use defaults
        return SimConfig{};
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Config parse error in '%s': %s — using defaults\n",
                     path.c_str(), e.what());
        return SimConfig{};
    }
}

} // namespace simuav
