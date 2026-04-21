namespace constants {

    constexpr double LOW_BATTERY_THRESHOLD = 20.0;
    constexpr int64_t HEARTBEAT_TIMEOUT = 500000000; // 500 ms
}

enum class FleetState {
    DISARMED,
    INITIATING_EXPERIMENT,
    TAKEOFF,
    LOITERING,
    IN_EXPERIMENT,
    ABORTING,
    LANDING
};

enum AgentState {
    undefined = 0,
    on_ground,
    in_air,
    takeoff,
    landing 
};

struct AgentStatus {
    std::string agent_id;
    AgentState agent_state;
    bool comms_healthy;
    builtin_interfaces::msg::Time last_heartbeat;
    double battery;
};

